use std::net::TcpStream;
use std::task::{Context, Poll};
use std::time::{Duration, Instant};

use clap::Parser;
use futures::stream::StreamExt;
use log::{error, info, warn, LevelFilter};
use r2r::geometry_msgs::msg::Twist;
use r2r::nav_msgs::msg::Odometry;
use r2r::std_msgs::msg::Bool;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::{Clock, ClockType, QosProfile};
use serde::Deserialize;
use tungstenite::stream::MaybeTlsStream;

// ── JSON messages from hls_canopen WebSocket ────────────────────────

#[derive(Deserialize, Debug)]
struct WsOdom {
    #[serde(default)]
    linear: f64,
    #[serde(default)]
    angular: f64,
    /// Raw 0x6064 position counts (0.1 rad/count) — diagnostics only
    #[serde(default)]
    left_pos: i64,
    #[serde(default)]
    right_pos: i64,
    pose: WsPose,
    #[serde(default)]
    status: String,
    #[serde(default)]
    error: String,
}

#[derive(Deserialize, Debug)]
struct WsPose {
    x: f64,
    y: f64,
    theta: f64,
}

// ── CLI arguments ───────────────────────────────────────────────────

#[derive(Parser, Debug)]
#[command(name = "canopen_ros2", version)]
struct Args {
    /// hls_canopen WebSocket URL
    #[arg(long, default_value = "ws://127.0.0.1:9090")]
    ws_url: String,

    /// Child frame ID (robot base)
    #[arg(long, default_value = "base_link")]
    frame_id: String,

    /// Odom frame ID
    #[arg(long, default_value = "odom")]
    odom_frame_id: String,

    /// Control cycle (ms)
    #[arg(long, default_value_t = 50)]
    cycle_ms: u64,

    /// Watchdog: send zero vel if no /cmd_vel for this long (ms)
    #[arg(long, default_value_t = 500)]
    watchdog_ms: u64,

    /// Pose covariance diagonal value
    #[arg(long, default_value_t = 0.001)]
    pose_cov: f64,

    /// Twist covariance diagonal value
    #[arg(long, default_value_t = 0.01)]
    twist_cov: f64,

    /// Yaw (rotation) covariance — wheel odom rotation is much worse
    /// than translation on this front-diff chassis (caster drag, slip).
    /// Applied to pose[35] and twist[35].
    #[arg(long, default_value_t = 0.05)]
    yaw_cov: f64,

    /// Publish odom→base_link TF. Set to false when robot_localization
    /// EKF owns the TF (only one node may broadcast odom→base_link).
    #[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
    publish_tf: bool,

    /// Enable debug logging
    #[arg(long, default_value_t = false)]
    debug: bool,
}

// ── Helpers ─────────────────────────────────────────────────────────

fn set_ws_read_timeout(
    socket: &mut tungstenite::WebSocket<MaybeTlsStream<TcpStream>>,
    timeout: Option<Duration>,
) -> std::io::Result<()> {
    match socket.get_mut() {
        MaybeTlsStream::Plain(s) => s.set_read_timeout(timeout),
        _ => Ok(()),
    }
}

fn build_odom(
    ws: &WsOdom,
    stamp: r2r::builtin_interfaces::msg::Time,
    args: &Args,
) -> Odometry {
    let theta_half = ws.pose.theta / 2.0;

    let mut odom = Odometry::default();
    odom.header.frame_id = args.odom_frame_id.clone();
    odom.header.stamp = stamp;
    odom.child_frame_id = args.frame_id.clone();

    odom.pose.pose.position.x = ws.pose.x;
    odom.pose.pose.position.y = ws.pose.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.z = theta_half.sin();
    odom.pose.pose.orientation.w = theta_half.cos();

    odom.twist.twist.linear.x = ws.linear;
    odom.twist.twist.angular.z = ws.angular;

    for i in (0..36).step_by(7) {
        odom.pose.covariance[i] = args.pose_cov;
        odom.twist.covariance[i] = args.twist_cov;
    }
    // Yaw is the weak axis of wheel odometry on this chassis —
    // tell the EKF to trust IMU gyro more than wheel-derived yaw.
    odom.pose.covariance[35] = args.yaw_cov;
    odom.twist.covariance[35] = args.yaw_cov;

    odom
}

fn build_tf(
    ws: &WsOdom,
    stamp: r2r::builtin_interfaces::msg::Time,
    args: &Args,
) -> TFMessage {
    let theta_half = ws.pose.theta / 2.0;

    let mut transform =
        r2r::geometry_msgs::msg::TransformStamped::default();
    transform.header.frame_id = args.odom_frame_id.clone();
    transform.header.stamp = stamp;
    transform.child_frame_id = args.frame_id.clone();
    transform.transform.translation.x = ws.pose.x;
    transform.transform.translation.y = ws.pose.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.z = theta_half.sin();
    transform.transform.rotation.w = theta_half.cos();

    let mut tf = TFMessage::default();
    tf.transforms.push(transform);
    tf
}

// ── main ────────────────────────────────────────────────────────────

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    env_logger::Builder::new()
        .filter_level(if args.debug {
            LevelFilter::Debug
        } else {
            LevelFilter::Info
        })
        .init();

    info!("canopen_ros2 starting — ws={}", args.ws_url);

    // ── ROS2 setup ──────────────────────────────────────────────────
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "canopen_ros2", "")?;

    // Subscriber returns a Stream — poll it synchronously in the loop
    let mut cmd_vel_stream = node.subscribe::<Twist>(
        "/cmd_vel",
        QosProfile::default(),
    )?;

    let mut motor_enable_stream = node.subscribe::<Bool>(
        "/motor_enable",
        QosProfile::default(),
    )?;

    // Publishers
    let odom_pub =
        node.create_publisher::<Odometry>("/odom", QosProfile::default())?;
    let tf_pub =
        node.create_publisher::<TFMessage>("/tf", QosProfile::default())?;

    let mut clock = Clock::create(ClockType::RosTime)?;

    info!("ROS2 node ready: /cmd_vel /motor_enable → WS, WS → /odom + /tf");

    // ── WebSocket connection loop ───────────────────────────────────
    let cycle = Duration::from_millis(args.cycle_ms);
    let watchdog = Duration::from_millis(args.watchdog_ms);

    let mut backoff = Duration::from_secs(1);
    let max_backoff = Duration::from_secs(30);

    let waker = futures::task::noop_waker();
    let mut cx = Context::from_waker(&waker);

    loop {
        // Connect (use &str, not Url — tungstenite 0.26)
        let (mut socket, _resp) =
            match tungstenite::connect(args.ws_url.as_str()) {
                Ok(conn) => {
                    info!("Connected to hls_canopen at {}", args.ws_url);
                    backoff = Duration::from_secs(1);
                    conn
                }
                Err(e) => {
                    error!(
                        "WebSocket connect failed: {} — retry in {:?}",
                        e, backoff
                    );
                    std::thread::sleep(backoff);
                    backoff = std::cmp::min(backoff * 2, max_backoff);
                    continue;
                }
            };

        if let Err(e) = set_ws_read_timeout(&mut socket, Some(cycle)) {
            error!("set_read_timeout: {}", e);
            std::thread::sleep(Duration::from_secs(1));
            continue;
        }

        let mut last_cmd_time = Instant::now();
        info!("Bridge active");

        // ── Inner loop ──────────────────────────────────────────
        'inner: loop {
            // Spin ROS2 to push incoming messages into subscriber stream
            node.spin_once(Duration::from_millis(1));

            // Poll for new /cmd_vel from subscriber stream
            loop {
                match cmd_vel_stream.poll_next_unpin(&mut cx) {
                    Poll::Ready(Some(twist)) => {
                        let json = serde_json::json!({
                            "linear": twist.linear.x,
                            "angular": twist.angular.z,
                        });
                        let text = json.to_string();
                        if let Err(e) = socket
                            .send(tungstenite::Message::Text(text.into()))
                        {
                            error!("WS send cmd_vel failed: {}", e);
                            break 'inner;
                        }
                        last_cmd_time = Instant::now();
                        // Consume all queued cmd_vel — keep polling
                        continue;
                    }
                    Poll::Ready(None) => {
                        // Stream closed — shouldn't happen
                        warn!("cmd_vel stream closed");
                        break 'inner;
                    }
                    Poll::Pending => {
                        // No more messages — done polling
                        break;
                    }
                }
            }

            // Poll for /motor_enable commands
            loop {
                match motor_enable_stream.poll_next_unpin(&mut cx) {
                    Poll::Ready(Some(msg)) => {
                        let cmd = if msg.data { "enable" } else { "disable" };
                        let json = serde_json::json!({"cmd": cmd});
                        let text = json.to_string();
                        if let Err(e) = socket
                            .send(tungstenite::Message::Text(text.into()))
                        {
                            error!("WS send motor_enable failed: {}", e);
                            break 'inner;
                        }
                        info!("Motor {}", cmd);
                        continue;
                    }
                    Poll::Ready(None) => {
                        warn!("motor_enable stream closed");
                        break 'inner;
                    }
                    Poll::Pending => break,
                }
            }

            // Watchdog: send zero velocity to keep motors enabled
            if last_cmd_time.elapsed() > watchdog {
                let json =
                    serde_json::json!({"linear": 0.0, "angular": 0.0});
                let text = json.to_string();
                let _ = socket
                    .send(tungstenite::Message::Text(text.into()));
                last_cmd_time = Instant::now();
            }

            // Read odom from WebSocket
            match socket.read() {
                Ok(tungstenite::Message::Text(text)) => {
                    match serde_json::from_str::<WsOdom>(&text) {
                        Ok(ws_odom) => {
                            if !ws_odom.error.is_empty() {
                                warn!(
                                    "hls_canopen fault: {} — freezing odom",
                                    ws_odom.error
                                );
                                continue;
                            }

                            let now = clock.get_now()?;
    let stamp = r2r::Clock::to_builtin_time(&now);
                            let odom = build_odom(&ws_odom, stamp.clone(), &args);
                            odom_pub.publish(&odom)?;

                            // EKF mode: robot_localization owns
                            // odom→base_link, we only publish /odom data
                            if args.publish_tf {
                                let tf = build_tf(&ws_odom, stamp, &args);
                                tf_pub.publish(&tf)?;
                            }
                        }
                        Err(e) => {
                            warn!("Bad JSON from WS: {} — {}", e, text);
                        }
                    }
                }
                Ok(tungstenite::Message::Ping(data)) => {
                    socket
                        .send(tungstenite::Message::Pong(data))
                        .ok();
                }
                Ok(tungstenite::Message::Close(_)) => {
                    info!("WebSocket closed — reconnecting...");
                    break 'inner;
                }
                Err(tungstenite::Error::Io(ref e))
                    if e.kind() == std::io::ErrorKind::WouldBlock =>
                {
                    // Timeout — no WS message, spin again
                }
                Err(e) => {
                    error!("WS read error: {}", e);
                    break 'inner;
                }
                _ => {}
            }
        }

        info!("Reconnecting in {:?}...", backoff);
        std::thread::sleep(backoff);
        backoff = std::cmp::min(backoff * 2, max_backoff);
    }
}

mod protocol;
mod transport;

use clap::Parser;
use log::{error, info, warn, LevelFilter};
use protocol::ls1207de_parser::{self, ParserConfig};
use r2r::sensor_msgs::msg::LaserScan;
use r2r::{Clock, ClockType, QosProfile};
use std::time::Duration;
use transport::LidarTransport;

/// Keli LSE-2027DE LiDAR ROS2 driver (Rust + r2r)
#[derive(Parser, Debug)]
#[command(name = "keli_lidar", version)]
struct Args {
    /// LiDAR hostname or IP address
    #[arg(long, default_value = "192.168.8.11")]
    hostname: String,

    /// LiDAR UDP port
    #[arg(long, default_value_t = 2112)]
    port: u16,

    /// Frame ID for published LaserScan messages
    #[arg(long, default_value = "laser")]
    frame_id: String,

    /// Minimum range (meters)
    #[arg(long, default_value_t = 0.05)]
    range_min: f64,

    /// Maximum range (meters)
    #[arg(long, default_value_t = 10.0)]
    range_max: f64,

    /// Time increment override (seconds)
    #[arg(long)]
    time_increment: Option<f64>,

    /// Time offset (seconds)
    #[arg(long, default_value_t = 0.0)]
    time_offset: f64,

    /// Minimum angle (radians)
    #[arg(long, default_value_t = -std::f64::consts::PI)]
    min_ang: f64,

    /// Maximum angle (radians)
    #[arg(long, default_value_t = std::f64::consts::PI)]
    max_ang: f64,

    /// Enable intensity data
    #[arg(long, default_value_t = true)]
    intensity: bool,

    /// UDP read timeout (seconds)
    #[arg(long, default_value_t = 5)]
    timeout: u32,

    /// Skip every N frames (0 = no skip)
    #[arg(long, default_value_t = 0)]
    skip: u32,

    /// Enable debug logging
    #[arg(long, default_value_t = false)]
    debug: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    // Log level
    env_logger::Builder::new()
        .filter_level(if args.debug {
            LevelFilter::Debug
        } else {
            LevelFilter::Info
        })
        .init();

    info!(
        "keli_lidar starting — host={}:{} frame_id={}",
        args.hostname, args.port, args.frame_id
    );

    // Parser config
    let mut parser_config = ParserConfig {
        frame_id: args.frame_id.clone(),
        range_min: args.range_min,
        range_max: args.range_max,
        min_ang: args.min_ang,
        max_ang: args.max_ang,
        intensity: args.intensity,
        time_offset: args.time_offset,
        debug_mode: args.debug,
        ..Default::default()
    };
    if let Some(ti) = args.time_increment {
        parser_config.time_increment = ti;
    }

    // Initialize r2r context and node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "keli_lidar", "")?;
    let scan_publisher =
        node.create_publisher::<LaserScan>("/scan", QosProfile::default())?;

    // Create a ROS clock for timestamps
    let mut clock = Clock::create(ClockType::RosTime)?;

    info!("ROS2 node 'keli_lidar' created, publishing to /scan");

    // Main connection loop
    loop {
        // Create transport
        let mut transport = LidarTransport::new(&args.hostname, args.port, args.timeout);
        transport.set_skip(args.skip);

        // Connect
        if let Err(e) = transport.connect() {
            error!("connect failed: {}", e);
            std::thread::sleep(Duration::from_secs(2));
            continue;
        }

        // Send start stream command
        if let Err(e) = transport.send_command(&protocol::constants::CMD_START_STREAM) {
            error!("start stream command failed: {}", e);
            transport.disconnect();
            std::thread::sleep(Duration::from_secs(2));
            continue;
        }

        info!("Connected to LiDAR, streaming started");

        // Data loop
        let mut publish_count = 0u64;

        loop {
            // Spin once to handle service calls, etc.
            node.spin_once(Duration::from_millis(1));

            // Try to read and reassemble a frame
            match transport.read_and_reassemble() {
                Ok(Some(frame_data)) => {
                    // Parse into LaserScan
                    if let Some(mut msg) =
                        ls1207de_parser::parse_scan(&frame_data, &parser_config)
                    {
                        // Set timestamp: last scan point = now
                        // → first point = now - data_count * time_increment
                        let now = clock.get_now()?;
                        let data_count = frame_data.len() / 2;
                        let time_inc_ns =
                            (parser_config.time_increment * 1e9) as u64;
                        let stamp_nanos = now.as_nanos() as i64
                            - (data_count as i64 * time_inc_ns as i64);
                        // Shift forward to first published scan point
                        let offset_ns =
                            (parser_config.time_offset * 1e9) as i64;
                        let total_ns = stamp_nanos + offset_ns;

                        let sec = (total_ns / 1_000_000_000) as i32;
                        let nanosec = (total_ns % 1_000_000_000) as u32;
                        msg.header.stamp = r2r::builtin_interfaces::msg::Time {
                            sec: if sec < 0 { 0 } else { sec },
                            nanosec,
                        };

                        // Publish
                        scan_publisher.publish(&msg)?;
                        publish_count += 1;

                        if args.debug && publish_count % 100 == 0 {
                            info!("Published {} scans", publish_count);
                        }
                    } else {
                        warn!("parse_scan returned None — invalid frame");
                    }
                }
                Ok(None) => {
                    // Incomplete reassembly, keep going
                    continue;
                }
                Err(e) => {
                    error!("transport error: {}", e);
                    // Attempt reconnection
                    break;
                }
            }
        }

        // Stop scanner before reconnect
        let _ = transport.send_command(&protocol::constants::CMD_STOP_STREAM);
        transport.disconnect();

        info!("Reconnecting in 2 seconds...");
        std::thread::sleep(Duration::from_secs(2));
    }
}

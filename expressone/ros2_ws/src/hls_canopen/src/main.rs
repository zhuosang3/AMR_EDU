// HLS_canopen — AMR motor control via CANopen SDO
//
// This binary: WebSocket cmd_vel interface
//   ws://0.0.0.0:9090
//   input:  {"linear": 0.4, "angular": 0.1}
//   output: {"linear": 0.45, "angular": 0.09, "left_rpm": 450, "right_rpm": -442, "status": "enabled"}
//
// For CLI debugging, use: cargo run --bin hls_cli

use anyhow::{Context, Result};
use hls_canopen::can_driver::SocketCanBus;
use hls_canopen::can_setup;
use hls_canopen::kinematics::{
    integrate_displacement, position_delta_to_displacement, DiffDrive,
};
use hls_canopen::motor::{Motor, PV_MODE, LEFT_WHEEL_ID, RIGHT_WHEEL_ID};
use hls_canopen::sdo_client::SdoClient;
use serde::Deserialize;
use std::net::TcpListener;
use std::time::{Duration, Instant};

// ── Physical parameters (EXP_EDU / EXP1 chassis) ────────────────────

const WHEEL_RADIUS: f64 = 0.0845;     // ø169 mm → radius 84.5 mm
const WHEEL_BASE: f64 = 0.418;        // 物理轮距 418mm (2026-07-18 实物测量 41.8cm)
const RIGHT_INVERTED: bool = true;    // right motor wired opposite direction

const WS_BIND: &str = "0.0.0.0:9090";
const CYCLE_MS: u64 = 50;            // 20 Hz control loop
const WATCHDOG_MS: u64 = 1000;       // disable motors if no cmd for 1 s

// ── Drive-side CAN disconnection watchdog ──────────────────────────
//
/// Set to `true` to configure the DS20270DA internal CAN watchdog
/// at startup.  When enabled, the drive will automatically disable
/// the motor if CAN communication is lost for the configured timeout.
///
/// Parameters written (SDO, both wheels):
///   0x450B ← timeout_ms   (default 0 = disabled)
///   0x450C ← action        (0=alarm  1=disable  2=zero-speed)
const CAN_WATCHDOG: bool = false;
const CAN_WATCHDOG_TIMEOUT_MS: u32 = 500;
const CAN_WATCHDOG_ACTION: u32 = 1; // 1 = disable motors

// ── JSON protocol ───────────────────────────────────────────────────

#[derive(Deserialize)]
struct CmdVel {
    #[serde(default)]
    linear: f64,
    #[serde(default)]
    angular: f64,
    #[serde(default)]
    cmd: Option<String>,
}

// ── main ────────────────────────────────────────────────────────────

fn main() -> Result<()> {
    println!("╔══════════════════════════════════════════════════════╗");
    println!("║  HLS_canopen v0.1.0 — CANopen Motor Control         ║");
    println!("║  DS20270DA · WebSocket cmd_vel · ws://{}  ║", WS_BIND);
    println!("╚══════════════════════════════════════════════════════╝");
    println!();

    // ── Hardware init ────────────────────────────────────────────────
    println!("[init] Configuring CAN interface (can0, 1 Mbps)...");
    can_setup::setup_can("can0", 1_000_000)?;

    println!("[init] Opening CAN socket...");
    let can = SocketCanBus::open("can0")?;

    let sdo = SdoClient::new(can);
    let mut motor = Motor::new(sdo);
    let diff = DiffDrive::new(WHEEL_RADIUS, WHEEL_BASE, RIGHT_INVERTED);

    println!("[init] NMT start + PV mode + accel...");
    for &id in &[LEFT_WHEEL_ID, RIGHT_WHEEL_ID] {
        motor.nmt_start(id)?;
    }
    for &id in &[LEFT_WHEEL_ID, RIGHT_WHEEL_ID] {
        motor.set_mode(id, PV_MODE)?;
        motor.set_accel(id, 1000, 1000)?;
        motor.set_velocity(id, 0)?;
    }

    println!("[init] Enabling motors (CiA 402 state machine)...");
    motor.enable(LEFT_WHEEL_ID).context("left motor enable")?;
    motor.enable(RIGHT_WHEEL_ID).context("right motor enable")?;

    // ── Configure drive-side CAN disconnection watchdog ─────────
    if CAN_WATCHDOG {
        println!(
            "[init] Drive CAN watchdog: {}ms timeout → action={} (disable)",
            CAN_WATCHDOG_TIMEOUT_MS, CAN_WATCHDOG_ACTION
        );
        for &id in &[LEFT_WHEEL_ID, RIGHT_WHEEL_ID] {
            motor
                .set_can_watchdog(id, CAN_WATCHDOG_TIMEOUT_MS, CAN_WATCHDOG_ACTION)
                .context("CAN watchdog")?;
        }
    }

    println!();
    println!("✓ Motors ready. wheel Ø={}mm base={}mm right_inverted={}",
        (WHEEL_RADIUS * 2000.0) as u32,
        (WHEEL_BASE * 1000.0) as u32,
        RIGHT_INVERTED,
    );
    println!("✓ Listening on ws://{}", WS_BIND);
    println!();

    // ── WebSocket + Control Loop ─────────────────────────────────────
    let listener = TcpListener::bind(WS_BIND)
        .context("Failed to bind WebSocket port")?;
    listener
        .set_nonblocking(true)
        .context("Failed to set nonblocking")?;

    let cycle = Duration::from_millis(CYCLE_MS);
    let watchdog = Duration::from_millis(WATCHDOG_MS);
    let mut conn_count: u64 = 0;

    loop {
        // ── Accept new connection ──────────────────────────────────
        match listener.accept() {
            Ok((stream, addr)) => {
                conn_count += 1;
                println!("[ws] Client connected: {} (#{})", addr, conn_count);

                stream
                    .set_read_timeout(Some(cycle))
                    .context("set_read_timeout")?;

                let mut ws = match tungstenite::accept(stream) {
                    Ok(ws) => ws,
                    Err(e) => {
                        eprintln!("[ws] Handshake failed: {}", e);
                        continue;
                    }
                };

                // ── Re-enable motors (they were disabled on
                //     previous disconnect) ──────────────────────────
                if let Err(e) = motor.enable(LEFT_WHEEL_ID) {
                    eprintln!("[ws] Re-enable left motor failed: {}", e);
                    continue;
                }
                if let Err(e) = motor.enable(RIGHT_WHEEL_ID) {
                    eprintln!("[ws] Re-enable right motor failed: {}", e);
                    let _ = motor.disable(LEFT_WHEEL_ID);
                    continue;
                }

                let mut last_cmd = Instant::now();
                let mut stopped = false;

                // ── Pose integration state ─────────────────────────
                let mut pose_x: f64 = 0.0;
                let mut pose_y: f64 = 0.0;
                let mut pose_theta: f64 = 0.0;
                // Previous 0x6063 raw counts (None until first read) —
                // position-delta odometry has no RPM quantization loss
                let mut last_counts: Option<(i32, i32)> = None;
                // Measured cycle time, used only for twist (velocity)
                // reporting — pose no longer depends on dt
                let mut last_odom = Instant::now();

                // Consecutive CAN errors (odom) — exit on 10+ for restart
                let mut can_errors: u32 = 0;

                // ── Inner loop: cmd_vel → CAN ──────────────────────
                loop {
                    match ws.read() {
                        Ok(tungstenite::Message::Text(text)) => {
                            match serde_json::from_str::<CmdVel>(&text) {
                                Ok(cmd) => {
                                    // ── Command dispatch ──────────────
                                    match cmd.cmd.as_deref() {
                                        Some("disable") => {
                                            println!("[ws] Disable command received");
                                            disable(&mut motor);
                                            stopped = true;
                                            last_cmd = Instant::now();
                                        }
                                        Some("enable") => {
                                            println!("[ws] Enable command received");
                                            if let Err(e) =
                                                motor.enable(LEFT_WHEEL_ID)
                                            {
                                                eprintln!("[can] left enable: {}", e);
                                            }
                                            if let Err(e) =
                                                motor.enable(RIGHT_WHEEL_ID)
                                            {
                                                eprintln!("[can] right enable: {}", e);
                                            }
                                            stopped = false;
                                            last_cmd = Instant::now();
                                        }
                                        _ => {
                                            // ── Velocity command ──────
                                            let (l_rpm, r_rpm) =
                                                diff.inverse(cmd.linear, cmd.angular);

                                            let l = l_rpm.round() as i32;
                                            let r = r_rpm.round() as i32;

                                            if let Err(e) =
                                                motor.set_velocity(LEFT_WHEEL_ID, l)
                                            {
                                                eprintln!("[can] left set_velocity: {}", e);
                                                can_errors += 1;
                                            }
                                            if let Err(e) =
                                                motor.set_velocity(RIGHT_WHEEL_ID, r)
                                            {
                                                eprintln!("[can] right set_velocity: {}", e);
                                                can_errors += 1;
                                            }

                                            last_cmd = Instant::now();
                                            stopped = false;
                                        }
                                    }
                                }
                                Err(e) => {
                                    eprintln!("[ws] Bad JSON: {} — raw: {}", e, text);
                                }
                            }
                        }

                        // Ping/Pong
                        Ok(tungstenite::Message::Ping(data)) => {
                            ws.send(tungstenite::Message::Pong(data))?;
                        }
                        Ok(tungstenite::Message::Close(_)) => {
                            println!("[ws] Client disconnected (#{})", conn_count);
                            break;
                        }

                        // Timeout — no message this cycle
                        Err(tungstenite::Error::Io(ref e))
                            if e.kind() == std::io::ErrorKind::WouldBlock => {}

                        // Real error
                        Err(e) => {
                            eprintln!("[ws] Read error: {}", e);
                            disable(&mut motor);
                            break;
                        }

                        _ => {} // binary frames ignored
                    }

                    // ── Odom: every cycle (20 Hz), 0x6063 position delta ──
                    let dt = last_odom.elapsed().as_secs_f64();
                    last_odom = Instant::now();
                    send_odom(
                        &mut ws,
                        &mut motor,
                        &mut pose_x,
                        &mut pose_y,
                        &mut pose_theta,
                        &mut last_counts,
                        &mut can_errors,
                        dt,
                    )?;

                    if can_errors >= 10 {
                        eprintln!(
                            "[can] {} consecutive CAN errors — attempting reconnect...",
                            can_errors
                        );
                        match motor.reinit_after_reconnect(30) {
                            Ok(()) => {
                                println!("[can] ✓ Reconnected, resuming operation");
                                can_errors = 0;
                                last_cmd = Instant::now();
                                stopped = false;
                                continue;
                            }
                            Err(e) => {
                                eprintln!("[can] Reconnect failed: {} — exiting", e);
                                disable(&mut motor);
                                break;
                            }
                        }
                    }

                    // ── Watchdog: disable if no cmd for too long ────
                    if !stopped && last_cmd.elapsed() > watchdog {
                        println!(
                            "[watchdog] No cmd_vel for {:?} — disabling motors",
                            WATCHDOG_MS
                        );
                        disable(&mut motor);
                        stopped = true;
                    }
                }

                // Connection lost — ensure motors disabled
                if !stopped {
                    println!("[ws] Connection lost — disabling motors");
                    disable(&mut motor);
                }
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                std::thread::sleep(cycle);
            }
            Err(e) => {
                eprintln!("[ws] Accept error: {}", e);
                std::thread::sleep(Duration::from_secs(1));
            }
        }
    }
}

/// Attempt to disable both motors, silently ignoring failure.
fn disable(motor: &mut Motor<SocketCanBus>) {
    let _ = motor.disable(LEFT_WHEEL_ID);
    let _ = motor.disable(RIGHT_WHEEL_ID);
}

/// Odom feedback (20 Hz), position-delta based.
///
/// Reads 0x6063 (position actual, 5600 counts/rev) from both wheels and
/// integrates the wrap-safe count deltas into the pose. Unlike the old
/// RPM (0x606C) integration, position deltas carry no quantization
/// loss — at 0.15 rad/s the RPM read was only ±3.8 RPM with ±0.5 RPM
/// integer error (~13%), which made rotation drift badly.
///
/// Twist (linear/angular) is derived from the same displacement over
/// measured dt — no extra 0x606C read, pose never depends on velocity.
///
/// On CAN fault: freezes pose AND resets the count baseline so a
/// reconnect (possibly with reset counters) can't inject a huge jump.
fn send_odom(
    ws: &mut tungstenite::WebSocket<std::net::TcpStream>,
    motor: &mut Motor<SocketCanBus>,
    x: &mut f64,
    y: &mut f64,
    theta: &mut f64,
    last_counts: &mut Option<(i32, i32)>,
    can_errors: &mut u32,
    dt: f64,
) -> Result<()> {
    let mut fault = false;

    // ── Position (pose source) ──────────────────────────────────────
    let l_pos = match motor.read_position_raw(LEFT_WHEEL_ID) {
        Ok(v) => v,
        Err(e) => {
            fault = true;
            eprintln!("[can] odom read_position(L): {e}");
            0
        }
    };
    let r_pos = match motor.read_position_raw(RIGHT_WHEEL_ID) {
        Ok(v) => v,
        Err(e) => {
            fault = true;
            eprintln!("[can] odom read_position(R): {e}");
            0
        }
    };

    let l_status = match motor.read_status(LEFT_WHEEL_ID) {
        Ok(s) => s,
        Err(e) => {
            fault = true;
            eprintln!("[can] odom read_status: {e}");
            0
        }
    };

    // ── Error counter: reset on healthy cycle, increment on fault ──
    if fault {
        *can_errors += 1;
        // Invalidate baseline — counters may reset across reconnect
        *last_counts = None;
    } else {
        *can_errors = 0;
    }

    let enabled = (l_status & 0x0007) == 0x0007;
    let status = if enabled { "enabled" } else { "disabled" };

    // ── Pose: integrate position delta (wrap-safe) ──────────────────
    let mut d_linear = 0.0;
    let mut d_theta = 0.0;
    if !fault {
        if let Some((pl, pr)) = *last_counts {
            let dl = l_pos.wrapping_sub(pl);
            let dr = r_pos.wrapping_sub(pr);
            let (dlin, dth) = position_delta_to_displacement(
                dl, dr, WHEEL_RADIUS, WHEEL_BASE, RIGHT_INVERTED,
            );
            d_linear = dlin;
            d_theta = dth;
            integrate_displacement(d_linear, d_theta, x, y, theta);
        }
        *last_counts = Some((l_pos, r_pos));
    }

    // ── Twist: derive from displacement / measured dt ───────────────
    // (smoother than the quantized 0x606C read, and one less SDO call)
    let (linear, angular) = if dt > 1e-4 {
        (d_linear / dt, d_theta / dt)
    } else {
        (0.0, 0.0)
    };

    let msg = if fault {
        format!(
            "{{\"error\":\"CAN down\",\"status\":\"disabled\",\"pose\":{{\"x\":{:.4},\"y\":{:.4},\"theta\":{:.4}}}}}",
            x, y, theta,
        )
    } else {
        format!(
            "{{\"linear\":{:.4},\"angular\":{:.4},\"left_pos\":{},\"right_pos\":{},\"pose\":{{\"x\":{:.4},\"y\":{:.4},\"theta\":{:.4}}},\"status\":\"{}\"}}",
            linear, angular, l_pos, r_pos,
            x, y, theta,
            status,
        )
    };

    ws.send(tungstenite::Message::Text(msg.into()))?;
    Ok(())
}

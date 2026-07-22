use anyhow::{Context, Result};
use hls_canopen::can_driver::SocketCanBus;
use hls_canopen::can_setup;
use hls_canopen::kinematics::DiffDrive;
use hls_canopen::motor::{Motor, PV_MODE, LEFT_WHEEL_ID, RIGHT_WHEEL_ID};
use hls_canopen::sdo_client::SdoClient;
use std::io::{self, BufRead, Write};

const BANNER: &str = r#"
╔══════════════════════════════════════════════════╗
║  HLS_canopen v0.1.0 — CANopen Motor Control     ║
║  DS20270DA 一拖二低压伺服驱动器                  ║
╚══════════════════════════════════════════════════╝
"#;

const HELP: &str = r#"
Commands:
  enable           Enable both motors (state machine)
  disable          Disable both motors
  go <rpm>         Forward: both wheels at <rpm> (uses kinematics)
  back <rpm>       Backward: both wheels at <rpm>
  left <rpm>       In-place CCW turn at <rpm> wheel speed
  right <rpm>      In-place CW turn at <rpm> wheel speed
  stop             Stop both motors (velocity → 0)
  status           Show motor status (enabled, RPM, status word)
  help             Show this help
  quit / exit      Exit
"#;

// ── Physical parameters (EXP_EDU / EXP1 chassis) ────────────────────
const WHEEL_RADIUS: f64 = 0.0845; // ø169 mm → radius 84.5 mm
const WHEEL_BASE: f64 = 0.45;     // 物理轮距 450mm (2026-06-11 重新标定确认, RAD_PER_COUNT=5600 后回归真实值)
const RIGHT_INVERTED: bool = true; // right motor wired opposite direction

fn main() -> Result<()> {
    println!("{}", BANNER);

    // ── Hardware initialization ──────────────────────────────────────
    println!("[init] Configuring CAN interface...");
    can_setup::setup_can("can0", 1_000_000)?;

    println!("[init] Opening CAN socket...");
    let can = SocketCanBus::open("can0")?;

    let sdo = SdoClient::new(can);
    let mut motor = Motor::new(sdo);
    let diff = DiffDrive::new(WHEEL_RADIUS, WHEEL_BASE, RIGHT_INVERTED);

    println!(
        "[init] NMT start left motor (node 0x{:02X})...",
        LEFT_WHEEL_ID
    );
    motor.nmt_start(LEFT_WHEEL_ID)?;

    println!(
        "[init] NMT start right motor (node 0x{:02X})...",
        RIGHT_WHEEL_ID
    );
    motor.nmt_start(RIGHT_WHEEL_ID)?;

    println!("[init] Setting PV mode...");
    motor.set_mode(LEFT_WHEEL_ID, PV_MODE)?;
    motor.set_mode(RIGHT_WHEEL_ID, PV_MODE)?;

    println!("[init] Setting acceleration/deceleration...");
    motor.set_accel(LEFT_WHEEL_ID, 1000, 1000)?;
    motor.set_accel(RIGHT_WHEEL_ID, 1000, 1000)?;

    println!("[init] Setting initial velocity to 0...");
    motor.set_velocity(LEFT_WHEEL_ID, 0)?;
    motor.set_velocity(RIGHT_WHEEL_ID, 0)?;

    println!("[init] Enabling motors (CiA 402 state machine)...");
    motor
        .enable(LEFT_WHEEL_ID)
        .context("Failed to enable left motor")?;
    motor
        .enable(RIGHT_WHEEL_ID)
        .context("Failed to enable right motor")?;

    println!("\n✓ Initialization complete. Motors ready.");
    println!(
        "  wheel Ø={}mm  base={}mm  right_inverted={}",
        (WHEEL_RADIUS * 2000.0) as u32,
        (WHEEL_BASE * 1000.0) as u32,
        RIGHT_INVERTED
    );
    println!();

    // ── CLI REPL ─────────────────────────────────────────────────────
    println!("{}", HELP);
    let stdin = io::stdin();
    let mut stdout = io::stdout();

    loop {
        print!("hls-canopen> ");
        stdout.flush()?;

        let mut line = String::new();
        if stdin.lock().read_line(&mut line).is_err() {
            break;
        }

        let line = line.trim();
        if line.is_empty() {
            continue;
        }

        let parts: Vec<&str> = line.split_whitespace().collect();
        let cmd = parts[0].to_lowercase();

        match cmd.as_str() {
            "quit" | "exit" | "q" => {
                println!("Exiting. Motors remain in current state.");
                break;
            }
            "help" | "?" => {
                println!("{}", HELP);
            }
            "enable" => match do_enable(&mut motor) {
                Ok(()) => println!("  Motors enabled OK"),
                Err(e) => {
                    safety_disable(&mut motor);
                    println!("  ERROR: {}", e);
                }
            },
            "disable" => match do_disable(&mut motor) {
                Ok(()) => println!("  Motors disabled"),
                Err(e) => {
                    safety_disable(&mut motor);
                    println!("  ERROR: {}", e);
                }
            },
            "stop" => match do_stop(&mut motor) {
                Ok(()) => println!("  Motors stopped"),
                Err(e) => {
                    safety_disable(&mut motor);
                    println!("  ERROR: {}", e);
                }
            },
            "go" => {
                let rpm = parse_rpm(parts.get(1));
                match rpm {
                    Some(r) => match do_move(&mut motor, &diff, r, 0) {
                        Ok((l, rp)) => {
                            println!("  v={:.2} m/s  L={} r/min  R={} r/min",
                                rpm_to_ms(r, WHEEL_RADIUS), l, rp)
                        }
                        Err(e) => {
                    safety_disable(&mut motor);
                    println!("  ERROR: {}", e);
                }
                    },
                    None => println!("  Usage: go <rpm>"),
                }
            }
            "back" => {
                let rpm = parse_rpm(parts.get(1));
                match rpm {
                    Some(r) => match do_move(&mut motor, &diff, -r, 0) {
                        Ok((l, rp)) => {
                            println!("  v={:.2} m/s  L={} r/min  R={} r/min",
                                rpm_to_ms(-r, WHEEL_RADIUS), l, rp)
                        }
                        Err(e) => {
                    safety_disable(&mut motor);
                    println!("  ERROR: {}", e);
                }
                    },
                    None => println!("  Usage: back <rpm>"),
                }
            }
            "left" => {
                let rpm = parse_rpm(parts.get(1));
                match rpm {
                    Some(r) => {
                        let omega = wheel_rpm_to_omega(r, WHEEL_RADIUS, WHEEL_BASE);
                        match do_move(&mut motor, &diff, 0, r) {
                            Ok((l, rp)) => {
                                println!("  ω={:.2} rad/s  L={} r/min  R={} r/min",
                                    omega, l, rp)
                            }
                            Err(e) => {
                    safety_disable(&mut motor);
                    println!("  ERROR: {}", e);
                }
                        }
                    }
                    None => println!("  Usage: left <rpm>"),
                }
            }
            "right" => {
                let rpm = parse_rpm(parts.get(1));
                match rpm {
                    Some(r) => {
                        let omega = -wheel_rpm_to_omega(r, WHEEL_RADIUS, WHEEL_BASE);
                        match do_move(&mut motor, &diff, 0, -r) {
                            Ok((l, rp)) => {
                                println!("  ω={:.2} rad/s  L={} r/min  R={} r/min",
                                    omega, l, rp)
                            }
                            Err(e) => {
                    safety_disable(&mut motor);
                    println!("  ERROR: {}", e);
                }
                        }
                    }
                    None => println!("  Usage: right <rpm>"),
                }
            }
            "status" => match do_status(&mut motor) {
                Ok(()) => {}
                Err(e) => {
                    safety_disable(&mut motor);
                    println!("  ERROR: {}", e);
                }
            },
            _ => {
                println!(
                    "  Unknown command: '{}'. Type 'help' for available commands.",
                    cmd
                );
            }
        }
    }

    Ok(())
}

// ── Command Handlers ─────────────────────────────────────────────────────

fn parse_rpm(arg: Option<&&str>) -> Option<i32> {
    arg?.parse::<i32>().ok()
}

/// Convert wheel RPM to linear velocity (m/s).
fn rpm_to_ms(rpm: i32, wheel_radius: f64) -> f64 {
    rpm as f64 * std::f64::consts::PI * wheel_radius / 30.0
}

/// In-place rotation: convert outer-wheel RPM to angular velocity (rad/s).
/// Both wheels at ±rpm, opposite directions.
fn wheel_rpm_to_omega(rpm: i32, wheel_radius: f64, wheel_base: f64) -> f64 {
    let v_wheel = rpm as f64 * std::f64::consts::PI * wheel_radius / 30.0;
    2.0 * v_wheel / wheel_base
}

fn do_enable(motor: &mut Motor<SocketCanBus>) -> Result<()> {
    motor.enable(LEFT_WHEEL_ID)?;
    motor.enable(RIGHT_WHEEL_ID)?;
    Ok(())
}

fn do_disable(motor: &mut Motor<SocketCanBus>) -> Result<()> {
    motor.disable(LEFT_WHEEL_ID)?;
    motor.disable(RIGHT_WHEEL_ID)?;
    Ok(())
}

/// Use DiffDrive for all movement — forward, backward, in-place turns.
/// Sign convention:
///   `go_rpm > 0` → forward (v > 0)
///   `turn_rpm > 0` → CCW (omega > 0), `turn_rpm < 0` → CW (omega < 0)
fn do_move(
    motor: &mut Motor<SocketCanBus>,
    diff: &DiffDrive,
    go_rpm: i32,
    turn_rpm: i32,
) -> Result<(i32, i32)> {
    let v = rpm_to_ms(go_rpm, WHEEL_RADIUS);
    let omega = if turn_rpm != 0 {
        wheel_rpm_to_omega(turn_rpm, WHEEL_RADIUS, WHEEL_BASE)
    } else {
        0.0
    };

    let (left_rpm, right_rpm) = diff.inverse(v, omega);

    let l = left_rpm.round() as i32;
    let r = right_rpm.round() as i32;

    motor.set_velocity(LEFT_WHEEL_ID, l)?;
    motor.set_velocity(RIGHT_WHEEL_ID, r)?;

    Ok((l, r))
}

fn do_stop(motor: &mut Motor<SocketCanBus>) -> Result<()> {
    motor.stop(LEFT_WHEEL_ID)?;
    motor.stop(RIGHT_WHEEL_ID)?;
    Ok(())
}

/// Safety disable: try to disable both motors, silently ignore failures.
/// Called after any fatal CAN error to ensure motors are not left running
/// when communication is lost.
fn safety_disable(motor: &mut Motor<SocketCanBus>) {
    let _ = motor.disable(LEFT_WHEEL_ID);
    let _ = motor.disable(RIGHT_WHEEL_ID);
}

fn do_status(motor: &mut Motor<SocketCanBus>) -> Result<()> {
    let l_status = motor.read_status(LEFT_WHEEL_ID)?;
    let r_status = motor.read_status(RIGHT_WHEEL_ID)?;
    let l_vel = motor.read_velocity(LEFT_WHEEL_ID)?;
    let r_vel = motor.read_velocity(RIGHT_WHEEL_ID)?;

    let enabled = |sw: u16| (sw & 0x0007) == 0x0007; // bits 0-2 all set

    println!(
        "  Left:  {} r/min, status=0x{:04X} ({})",
        l_vel,
        l_status,
        if enabled(l_status) {
            "enabled"
        } else {
            "disabled"
        }
    );
    println!(
        "  Right: {} r/min, status=0x{:04X} ({})",
        r_vel,
        r_status,
        if enabled(r_status) {
            "enabled"
        } else {
            "disabled"
        }
    );

    Ok(())
}

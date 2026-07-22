/// High-level motor control for DS20270DA servo drives via CANopen SDO.
///
/// Wraps an `SdoClient` to provide NMT commands, PV mode control, and the
/// CiA 402 drive state machine (enable/disable sequence with status polling).
///
/// Generic over `CanBus` — works with real hardware and mock for testing.
use crate::can_driver::CanBus;
use crate::can_driver::SocketCanBus;
use crate::sdo_client::{SdoClient, od};
use anyhow::{Result, bail};
use std::time::{Duration, Instant};

// ── Constants ────────────────────────────────────────────────────────────

/// PV (Profile Velocity) mode
pub const PV_MODE: u8 = 3;

/// NMT (Network Management) command specifier
const NMT_START_NODE: u8 = 0x01;
const NMT_COB_ID: u32 = 0x000;

/// DS20270DA node IDs for left and right wheels on a shared CAN bus
pub const LEFT_WHEEL_ID: u8 = 0x01;
pub const RIGHT_WHEEL_ID: u8 = 0x02;

/// Default timeout for status-polling operations (ms)
const STATUS_POLL_TIMEOUT_MS: u64 = 2000;
/// Interval between status reads during polling (ms)
const STATUS_POLL_INTERVAL_MS: u64 = 20;

// ── Motor ────────────────────────────────────────────────────────────────

/// CANopen motor controller for a single axis (node).
///
/// Use one instance per motor node, or create a higher-level abstraction
/// that manages both wheels together.
pub struct Motor<B: CanBus> {
    sdo: SdoClient<B>,
}

impl<B: CanBus> Motor<B> {
    /// Create a new motor controller wrapping an existing SDO client.
    pub fn new(sdo: SdoClient<B>) -> Self {
        Self { sdo }
    }

    // ── NMT ────────────────────────────────────────────────────────

    /// Send NMT "Start Node" command to bring a node into Operational state.
    ///
    /// COB-ID 0x000, data: [0x01, node_id]
    pub fn nmt_start(&mut self, node_id: u8) -> Result<()> {
        let mut buf = [0u8; 8];
        buf[0] = NMT_START_NODE;
        buf[1] = node_id;

        use socketcan::{CanFrame, EmbeddedFrame, Id, StandardId};
        let id = Id::Standard(StandardId::new(NMT_COB_ID as u16).unwrap());
        let frame = CanFrame::new(id, &buf).expect("valid NMT frame");
        self.sdo.can.write_frame(&frame)?;

        // Allow time for node to transition
        std::thread::sleep(Duration::from_millis(50));
        Ok(())
    }

    // ── Mode Configuration ─────────────────────────────────────────

    /// Set the operating mode (0x6060).
    ///
    /// Typically `PV_MODE` (3) for velocity control.
    pub fn set_mode(&mut self, node_id: u8, mode: u8) -> Result<()> {
        self.sdo
            .write_od(node_id, od::MODES_OF_OPERATION, 0, mode as u32)
    }

    /// Set acceleration (0x6083) and deceleration (0x6084).
    ///
    /// Units are drive-specific (typically rpm/s for DS20270DA).
    pub fn set_accel(&mut self, node_id: u8, accel: u32, decel: u32) -> Result<()> {
        self.sdo
            .write_od(node_id, od::PROFILE_ACCELERATION, 0, accel)?;
        self.sdo
            .write_od(node_id, od::PROFILE_DECELERATION, 0, decel)
    }

    // ── Drive State Machine ────────────────────────────────────────

    /// Enable the motor via CiA 402 state machine.
    ///
    /// State-aware: reads current status first, then only executes
    /// the transitions needed to reach Operation Enabled.
    ///
    /// DS20270DA status word bit layout (observed):
    ///   bit 0: Ready to switch on
    ///   bit 1: Switched on
    ///   bit 2: Operation enabled
    ///   bit 3: Fault
    ///   bit 4: Voltage enabled
    ///   bit 5: Quick stop (1 = no quick stop)
    ///   bit 6: Switch on disabled (0 = in SOD state)
    ///   bit 9: Remote (1 = CANopen control)
    ///
    /// States (from DS20270DA observations):
    ///   0x0240 — Switch On Disabled (bit6=0)
    ///   0x0231 — Ready to Switch On
    ///   0x0233 — Switched On
    ///   0x0237 — Operation Enabled
    pub fn enable(&mut self, node_id: u8) -> Result<()> {
        let sw = self.read_status(node_id)?;

        // Already enabled? Nothing to do.
        if (sw & 0x0007) == 0x0007 {
            // bit0=1, bit1=1, bit2=1 → Operation Enabled
            return Ok(());
        }

        // Fault? Clear it first.
        if (sw & 0x0008) != 0 {
            self.sdo.write_od(node_id, od::CONTROL_WORD, 0, 0x0080)?; // Fault reset
            std::thread::sleep(Duration::from_millis(100));
        }

        // ── Determine current state and take shortest path ──────────

        let is_switched_on = (sw & 0x0003) == 0x0003; // bit0=1, bit1=1
        let is_ready = (sw & 0x0001) == 0x0001; // bit0=1, bit1=0

        if is_switched_on {
            // Switched On → just need to enable operation
            // Step 3: Enable Operation
            self.sdo
                .write_od(node_id, od::CONTROL_WORD, 0, 0x000F)?;
            self.wait_status(
                node_id,
                |sw| (sw & 0x0007) == 0x0007, // bit0=1, bit1=1, bit2=1
                STATUS_POLL_TIMEOUT_MS,
                "Operation Enabled",
            )?;
        } else if is_ready {
            // Ready to Switch On → Switch On → Enable Operation
            // Step 2: Switch On
            self.sdo
                .write_od(node_id, od::CONTROL_WORD, 0, 0x0007)?;
            self.wait_status(
                node_id,
                |sw| (sw & 0x0003) == 0x0003, // bit0=1, bit1=1
                STATUS_POLL_TIMEOUT_MS,
                "Switched On",
            )?;

            // Step 3: Enable Operation
            self.sdo
                .write_od(node_id, od::CONTROL_WORD, 0, 0x000F)?;
            self.wait_status(
                node_id,
                |sw| (sw & 0x0007) == 0x0007, // bit0=1, bit1=1, bit2=1
                STATUS_POLL_TIMEOUT_MS,
                "Operation Enabled",
            )?;
        } else {
            // Switch On Disabled or lower — full sequence
            // Step 1: Shutdown
            self.sdo
                .write_od(node_id, od::CONTROL_WORD, 0, 0x0006)?;
            self.wait_status(
                node_id,
                |sw| (sw & 0x0001) == 0x0001, // bit0=1 (Ready to Switch On)
                STATUS_POLL_TIMEOUT_MS,
                "Ready to Switch On (after Shutdown)",
            )?;

            // Step 2: Switch On
            self.sdo
                .write_od(node_id, od::CONTROL_WORD, 0, 0x0007)?;
            self.wait_status(
                node_id,
                |sw| (sw & 0x0003) == 0x0003, // bit0=1, bit1=1
                STATUS_POLL_TIMEOUT_MS,
                "Switched On",
            )?;

            // Step 3: Enable Operation
            self.sdo
                .write_od(node_id, od::CONTROL_WORD, 0, 0x000F)?;
            self.wait_status(
                node_id,
                |sw| (sw & 0x0007) == 0x0007, // bit0=1, bit1=1, bit2=1
                STATUS_POLL_TIMEOUT_MS,
                "Operation Enabled",
            )?;
        }

        Ok(())
    }

    /// Disable the motor: write control word 0x0000.
    pub fn disable(&mut self, node_id: u8) -> Result<()> {
        self.sdo.write_od(node_id, od::CONTROL_WORD, 0, 0x0000)
    }

    // ── Velocity Control ───────────────────────────────────────────

    /// Set target velocity (0x60FF).
    ///
    /// **Unit:** 0x60FF uses 0.1 rpm — the value written to the drive is
    /// `rpm * 10`.  Positive = forward, negative = reverse.
    ///
    /// Writes the scaled `i32` value as LE u32 (bit-level identical).
    pub fn set_velocity(&mut self, node_id: u8, rpm: i32) -> Result<()> {
        // 0x60FF unit = 0.1 rpm → multiply by 10 for raw register value
        let raw = rpm * 10;
        self.sdo.write_od(node_id, od::TARGET_VELOCITY, 0, raw as u32)
    }

    /// Stop the motor: set target velocity to 0.
    pub fn stop(&mut self, node_id: u8) -> Result<()> {
        self.set_velocity(node_id, 0)
    }

    // ── Status ─────────────────────────────────────────────────────

    /// Read the status word (0x6041).
    pub fn read_status(&mut self, node_id: u8) -> Result<u16> {
        let val = self.sdo.read_od(node_id, od::STATUS_WORD, 0)?;
        // Status word is u16 in lower 2 bytes
        Ok(val as u16)
    }

    /// Read actual velocity (0x606C).
    ///
    /// **Unit:** 0x606C uses 0.1 rpm — divides by 10 so callers get rpm.
    pub fn read_velocity(&mut self, node_id: u8) -> Result<i32> {
        let raw = self.sdo.read_od(node_id, od::VELOCITY_ACTUAL, 0)?;
        Ok((raw as i32) / 10)
    }

    /// Read actual position (0x6063) as raw counts.
    ///
    /// **Unit:** 5600 counts per wheel rev (10530NA1550F20 motor, 1400-line encoder,
    /// 4× quadrature. Gear 1:1). Raw `i32` so
    /// the caller can compute wrap-safe deltas with `wrapping_sub` —
    /// do NOT convert to rad here, that would lose wrap semantics.
    pub fn read_position_raw(&mut self, node_id: u8) -> Result<i32> {
        let raw = self.sdo.read_od(node_id, od::POSITION_ACTUAL, 0)?;
        Ok(raw as i32)
    }

    // ── Drive CAN Watchdog ─────────────────────────────────────────

    /// Configure the drive-internal CAN disconnection watchdog.
    ///
    /// Writes 0x450B (timeout, ms) and 0x450C (action: 0=alarm 1=disable
    /// 2=zero-speed).  Call once during init if desired.
    pub fn set_can_watchdog(
        &mut self,
        node_id: u8,
        timeout_ms: u32,
        action: u32,
    ) -> Result<()> {
        self.sdo.write_od(node_id, 0x450B, 0, timeout_ms)?;
        self.sdo.write_od(node_id, 0x450C, 0, action)
    }

    // ── Private ─────────────────────────────────────────────────────

    /// Poll the status word until `predicate` returns true, or timeout.
    fn wait_status(
        &mut self,
        node_id: u8,
        predicate: fn(u16) -> bool,
        timeout_ms: u64,
        target_state: &str,
    ) -> Result<()> {
        let start = Instant::now();
        let timeout = Duration::from_millis(timeout_ms);
        let interval = Duration::from_millis(STATUS_POLL_INTERVAL_MS);

        loop {
            let sw = self.read_status(node_id)?;

            if predicate(sw) {
                return Ok(());
            }

            if start.elapsed() > timeout {
                bail!(
                    "Status timeout for node 0x{:02X}: expected '{}', got status=0x{:04X}",
                    node_id,
                    target_state,
                    sw
                );
            }

            std::thread::sleep(interval);
        }
    }
}

// ── CAN Bus Recovery (SocketCanBus only) ───────────────────────────

impl Motor<SocketCanBus> {
    /// Reconnect CAN bus and reinitialize both motors after a disconnection.
    ///
    /// Call this when consecutive CAN errors indicate the bus is down
    /// (e.g. USB-CAN adapter was unplugged and reconnected).
    ///
    /// Performs: socket reconnect → NMT start → PV mode → accel → enable
    /// for both left and right wheels.
    pub fn reinit_after_reconnect(&mut self, timeout_secs: u64) -> Result<()> {
        println!(
            "[motor] CAN reconnect + full reinit (timeout={}s)...",
            timeout_secs
        );

        self.sdo.can.reconnect("can0", 1_000_000, Duration::from_secs(timeout_secs))?;

        // Full init sequence for both wheels (same as cold start)
        for &id in &[LEFT_WHEEL_ID, RIGHT_WHEEL_ID] {
            self.nmt_start(id)?;
        }
        for &id in &[LEFT_WHEEL_ID, RIGHT_WHEEL_ID] {
            self.set_mode(id, PV_MODE)?;
            self.set_accel(id, 1000, 1000)?;
            self.set_velocity(id, 0)?;
        }

        self.enable(LEFT_WHEEL_ID)?;
        self.enable(RIGHT_WHEEL_ID)?;

        println!("[motor] ✓ Motors reinitialized");
        Ok(())
    }
}

// ── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::can_driver::MockCanBus;

    /// Helper: enqueue an SDO write ack
    fn push_ack(mock: &mut MockCanBus, node_id: u8, index: u16, subindex: u8, data: u32) {
        use crate::sdo_client::{SDO_RX_BASE, SDO_WRITE_SUCCESS};
        use socketcan::{CanFrame, EmbeddedFrame, Id, StandardId};

        let cob = SDO_RX_BASE + node_id as u32;
        let mut buf = [0u8; 8];
        buf[0] = SDO_WRITE_SUCCESS;
        buf[1..3].copy_from_slice(&index.to_le_bytes());
        buf[3] = subindex;
        buf[4..8].copy_from_slice(&data.to_le_bytes());

        let id = Id::Standard(StandardId::new(cob as u16).unwrap());
        mock.push_response(CanFrame::new(id, &buf).unwrap());
    }

    /// Helper: enqueue an SDO read response
    fn push_read(mock: &mut MockCanBus, node_id: u8, index: u16, subindex: u8, value: u32) {
        use crate::sdo_client::{SDO_READ_4BYTE, SDO_RX_BASE};
        use socketcan::{CanFrame, EmbeddedFrame, Id, StandardId};

        let cob = SDO_RX_BASE + node_id as u32;
        let mut buf = [0u8; 8];
        buf[0] = SDO_READ_4BYTE;
        buf[1..3].copy_from_slice(&index.to_le_bytes());
        buf[3] = subindex;
        buf[4..8].copy_from_slice(&value.to_le_bytes());

        let id = Id::Standard(StandardId::new(cob as u16).unwrap());
        mock.push_response(CanFrame::new(id, &buf).unwrap());
    }

    #[test]
    fn test_set_mode() {
        let mock = MockCanBus::new();
        let sdo = SdoClient::new(mock);
        let mut motor = Motor::new(sdo);

        // Pre-configure: write ack for 0x6060 sub 0 = 3
        push_ack(&mut motor.sdo.can, 1, od::MODES_OF_OPERATION, 0, 3);

        motor.set_mode(1, PV_MODE).unwrap();
        assert_eq!(motor.sdo.can.sent.len(), 1);
    }

    #[test]
    fn test_set_accel() {
        let mock = MockCanBus::new();
        let sdo = SdoClient::new(mock);
        let mut motor = Motor::new(sdo);

        // Pre-configure two acks: accel + decel
        push_ack(&mut motor.sdo.can, 1, od::PROFILE_ACCELERATION, 0, 1000);
        push_ack(&mut motor.sdo.can, 1, od::PROFILE_DECELERATION, 0, 1000);

        motor.set_accel(1, 1000, 1000).unwrap();
        assert_eq!(motor.sdo.can.sent.len(), 2);
    }

    #[test]
    fn test_set_velocity_positive() {
        let mock = MockCanBus::new();
        let sdo = SdoClient::new(mock);
        let mut motor = Motor::new(sdo);

        // 500 rpm → 5000 raw (0.1 rpm unit)
        push_ack(&mut motor.sdo.can, 1, od::TARGET_VELOCITY, 0, 5000);

        motor.set_velocity(1, 500).unwrap();
    }

    #[test]
    fn test_set_velocity_negative() {
        let mock = MockCanBus::new();
        let sdo = SdoClient::new(mock);
        let mut motor = Motor::new(sdo);

        // -300 rpm → -3000 raw (0.1 rpm unit)
        let expected_u32 = (-3000i32) as u32;
        push_ack(&mut motor.sdo.can, 1, od::TARGET_VELOCITY, 0, expected_u32);

        motor.set_velocity(1, -300).unwrap();
    }

    #[test]
    fn test_read_status() {
        let mock = MockCanBus::new();
        let sdo = SdoClient::new(mock);
        let mut motor = Motor::new(sdo);

        // Status word = 0x0237 (Operation Enabled)
        push_read(&mut motor.sdo.can, 1, od::STATUS_WORD, 0, 0x0237);

        let status = motor.read_status(1).unwrap();
        assert_eq!(status, 0x0237);
    }

    /// Test enable from "Ready to Switch On" (0x0231) — the common DS20270DA case.
    /// Path: read status → 0x07 → wait Switched On → 0x0F → wait Operation Enabled
    #[test]
    fn test_enable_sequence() {
        let mock = MockCanBus::new();
        let sdo = SdoClient::new(mock);
        let mut motor = Motor::new(sdo);

        // Initial status read: Ready to Switch On
        push_read(&mut motor.sdo.can, 1, od::STATUS_WORD, 0, 0x0231);

        // Step 2: Write 0x6040 ← 0x07 (Switch On)
        push_ack(&mut motor.sdo.can, 1, od::CONTROL_WORD, 0, 0x07);
        // → Read status = 0x0233 (Switched On)
        push_read(&mut motor.sdo.can, 1, od::STATUS_WORD, 0, 0x0233);

        // Step 3: Write 0x6040 ← 0x0F (Enable Operation)
        push_ack(&mut motor.sdo.can, 1, od::CONTROL_WORD, 0, 0x0F);
        // → Read status = 0x0237 (Operation Enabled)
        push_read(&mut motor.sdo.can, 1, od::STATUS_WORD, 0, 0x0237);

        motor.enable(1).unwrap();

        // 5 SDO operations: 1 status read + 2 writes + 2 status reads
        assert_eq!(motor.sdo.can.sent.len(), 5);
    }

    /// Test enable when already in Operation Enabled — should be a no-op.
    #[test]
    fn test_enable_already_enabled() {
        let mock = MockCanBus::new();
        let sdo = SdoClient::new(mock);
        let mut motor = Motor::new(sdo);

        // Initial status read: already enabled
        push_read(&mut motor.sdo.can, 1, od::STATUS_WORD, 0, 0x0237);

        motor.enable(1).unwrap();

        // Only 1 SDO operation: the initial status read
        assert_eq!(motor.sdo.can.sent.len(), 1);
    }

    /// Test full enable sequence from Switch On Disabled (status=0x0240).
    #[test]
    fn test_enable_from_disabled() {
        let mock = MockCanBus::new();
        let sdo = SdoClient::new(mock);
        let mut motor = Motor::new(sdo);

        // Initial status read: Switch On Disabled
        push_read(&mut motor.sdo.can, 1, od::STATUS_WORD, 0, 0x0240);

        // Step 1: Write 0x6040 ← 0x06 (Shutdown)
        push_ack(&mut motor.sdo.can, 1, od::CONTROL_WORD, 0, 0x06);
        // → Read status = 0x0231 (Ready to Switch On)
        push_read(&mut motor.sdo.can, 1, od::STATUS_WORD, 0, 0x0231);

        // Step 2: Write 0x6040 ← 0x07 (Switch On)
        push_ack(&mut motor.sdo.can, 1, od::CONTROL_WORD, 0, 0x07);
        // → Read status = 0x0233 (Switched On)
        push_read(&mut motor.sdo.can, 1, od::STATUS_WORD, 0, 0x0233);

        // Step 3: Write 0x6040 ← 0x0F (Enable Operation)
        push_ack(&mut motor.sdo.can, 1, od::CONTROL_WORD, 0, 0x0F);
        // → Read status = 0x0237 (Operation Enabled)
        push_read(&mut motor.sdo.can, 1, od::STATUS_WORD, 0, 0x0237);

        motor.enable(1).unwrap();

        // 7 SDO operations: 1 status read + 3 writes + 3 status reads
        assert_eq!(motor.sdo.can.sent.len(), 7);
    }

    #[test]
    fn test_disable() {
        let mock = MockCanBus::new();
        let sdo = SdoClient::new(mock);
        let mut motor = Motor::new(sdo);

        push_ack(&mut motor.sdo.can, 1, od::CONTROL_WORD, 0, 0x00);

        motor.disable(1).unwrap();
    }

    #[test]
    fn test_stop() {
        let mock = MockCanBus::new();
        let sdo = SdoClient::new(mock);
        let mut motor = Motor::new(sdo);

        push_ack(&mut motor.sdo.can, 1, od::TARGET_VELOCITY, 0, 0);

        motor.stop(1).unwrap();
    }
}

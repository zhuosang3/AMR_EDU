/// CANopen SDO (Service Data Object) client.
///
/// Handles SDO read/write to DS20270DA servo drives over CANopen.
/// Strict confirmation-frame validation — every write is acknowledged before proceeding.
///
/// Generic over `CanBus` trait: works with both real `SocketCanBus` and `MockCanBus`.
use crate::can_driver::CanBus;
use anyhow::Result;
use socketcan::{CanFrame, EmbeddedFrame, Id, StandardId};
use std::io;
use std::time::{Duration, Instant};

// ── SDO Command Codes ────────────────────────────────────────────────────

pub const SDO_WRITE_1BYTE: u8 = 0x2F;
pub const SDO_WRITE_2BYTE: u8 = 0x2B;
pub const SDO_WRITE_4BYTE: u8 = 0x23;
pub const SDO_READ: u8 = 0x40;

pub const SDO_WRITE_SUCCESS: u8 = 0x60;
pub const SDO_READ_1BYTE: u8 = 0x4F;
pub const SDO_READ_2BYTE: u8 = 0x4B;
pub const SDO_READ_4BYTE: u8 = 0x43;
pub const SDO_ERROR: u8 = 0x80;

// ── COB-ID Bases ─────────────────────────────────────────────────────────

const SDO_TX_BASE: u32 = 0x600; // Client → Server (request)
pub(crate) const SDO_RX_BASE: u32 = 0x580; // Server → Client (response)

// ── CANopen Object Dictionary ────────────────────────────────────────────

pub mod od {
    pub const CONTROL_WORD: u16 = 0x6040;
    pub const STATUS_WORD: u16 = 0x6041;
    pub const MODES_OF_OPERATION: u16 = 0x6060;
    pub const MODES_OF_OPERATION_DISPLAY: u16 = 0x6061;
    pub const TARGET_VELOCITY: u16 = 0x60FF;
    pub const VELOCITY_ACTUAL: u16 = 0x606C;
    /// Position actual value (encoder feedback), unit: 1/5600 rev per count.
    /// NOTE: DS20270DA does NOT populate 0x6064 (always 0); real feedback
    /// lives in 0x6063 (10530NA1550F20: 1400-line encoder, 4× = 5600 counts/rev).
    /// Verified by SDO read 2026-06-11: 0x6064=0, 0x6063=live counts;
    /// encoder type "A" confirmed from motor model number.
    pub const POSITION_ACTUAL: u16 = 0x6063;
    pub const PROFILE_ACCELERATION: u16 = 0x6083;
    pub const PROFILE_DECELERATION: u16 = 0x6084;
}

// ── SdoClient ────────────────────────────────────────────────────────────

/// SDO client generic over the CAN bus transport.
pub struct SdoClient<B: CanBus> {
    pub(crate) can: B,
    timeout: Duration,
}

impl<B: CanBus> SdoClient<B> {
    /// Create a new SDO client with the given CAN transport.
    pub fn new(can: B) -> Self {
        Self {
            can,
            timeout: Duration::from_millis(100),
        }
    }

    /// Write a 4-byte value to the object dictionary.
    ///
    /// Sends SDO write request, waits for confirmation frame from server.
    /// Validates the response: cmd=0x60 → success, cmd=0x80 → error.
    pub fn write_od(&mut self, node_id: u8, index: u16, subindex: u8, data: u32) -> Result<()> {
        let tx_cob = SDO_TX_BASE + node_id as u32;
        let rx_cob = SDO_RX_BASE + node_id as u32;

        // Build SDO write 4-byte request frame
        let mut buf = [0u8; 8];
        buf[0] = SDO_WRITE_4BYTE;
        buf[1..3].copy_from_slice(&index.to_le_bytes());
        buf[3] = subindex;
        buf[4..8].copy_from_slice(&data.to_le_bytes());

        let frame = make_can_frame(tx_cob, &buf);
        self.can.write_frame(&frame)?;

        // Wait for confirmation
        let response = self.wait_sdo_response(rx_cob)?;
        let data = response.data();
        let cmd = data[0];

        if cmd == SDO_ERROR {
            let err_code = u32::from_le_bytes(data[4..8].try_into().unwrap());
            anyhow::bail!(
                "SDO error: node=0x{:02X}, index=0x{:04X}, sub={}, abort=0x{:08X}",
                node_id,
                index,
                subindex,
                err_code
            );
        }

        Ok(())
    }

    /// Read a value from the object dictionary.
    ///
    /// Sends SDO read request, waits for response. Returns 4-byte value.
    pub fn read_od(&mut self, node_id: u8, index: u16, subindex: u8) -> Result<u32> {
        let tx_cob = SDO_TX_BASE + node_id as u32;
        let rx_cob = SDO_RX_BASE + node_id as u32;

        // Build SDO read request
        let mut buf = [0u8; 8];
        buf[0] = SDO_READ;
        buf[1..3].copy_from_slice(&index.to_le_bytes());
        buf[3] = subindex;
        // bytes 4-7: reserved (0)

        let frame = make_can_frame(tx_cob, &buf);
        self.can.write_frame(&frame)?;

        // Wait for response
        let response = self.wait_sdo_response(rx_cob)?;
        let data = response.data();
        let cmd = data[0];

        if cmd == SDO_ERROR {
            let err_code = u32::from_le_bytes(data[4..8].try_into().unwrap());
            anyhow::bail!(
                "SDO read error: node=0x{:02X}, index=0x{:04X}, sub={}, abort=0x{:08X}",
                node_id,
                index,
                subindex,
                err_code
            );
        }

        // Verify the response matches what we asked for
        let resp_index = u16::from_le_bytes([data[1], data[2]]);
        let resp_sub = data[3];
        if resp_index != index || resp_sub != subindex {
            anyhow::bail!(
                "SDO response mismatch: expected (0x{:04X},{}) got (0x{:04X},{})",
                index,
                subindex,
                resp_index,
                resp_sub
            );
        }

        // Parse data based on response command
        match cmd {
            SDO_READ_1BYTE => Ok(data[4] as u32),
            SDO_READ_2BYTE => Ok(u16::from_le_bytes([data[4], data[5]]) as u32),
            SDO_READ_4BYTE => Ok(u32::from_le_bytes(data[4..8].try_into().unwrap())),
            _ => anyhow::bail!("Unexpected SDO read response cmd=0x{:02X}", cmd),
        }
    }

    // ── Private ───────────────────────────────────────────────────────

    /// Wait for an SDO response frame with the expected COB-ID.
    fn wait_sdo_response(&mut self, expected_cob: u32) -> Result<CanFrame> {
        let start = Instant::now();

        loop {
            if start.elapsed() > self.timeout {
                anyhow::bail!(
                    "SDO timeout: no response for COB-ID 0x{:03X} within {:?}",
                    expected_cob,
                    self.timeout
                );
            }

            match self.can.read_frame() {
                Ok(Some(frame)) => {
                    if can_frame_cob(&frame) == expected_cob {
                        return Ok(frame);
                    }
                    // Not our frame — keep listening
                }
                Ok(None) => {
                    // No frame available (mock bus empty)
                    std::thread::sleep(Duration::from_millis(1));
                }
                Err(e) => {
                    // Check if this is an IO error that's transient
                    let is_transient = e
                        .downcast_ref::<io::Error>()
                        .map(|ioe| {
                            ioe.kind() == io::ErrorKind::WouldBlock
                                || ioe.kind() == io::ErrorKind::TimedOut
                        })
                        .unwrap_or(false);

                    if is_transient {
                        std::thread::sleep(Duration::from_millis(1));
                    } else {
                        // Fatal (ENXIO, ENODEV, etc.) — can't recover, propagate
                        return Err(e);
                    }
                }
            }
        }
    }
}

// ── CAN Frame Utilities ──────────────────────────────────────────────────

/// Build a standard CAN data frame with given COB-ID and 8-byte payload.
fn make_can_frame(cob_id: u32, data: &[u8; 8]) -> CanFrame {
    let id = if cob_id > 0x7FF {
        Id::Extended(socketcan::ExtendedId::new(cob_id).expect("valid extended CAN-ID"))
    } else {
        Id::Standard(StandardId::new(cob_id as u16).expect("valid standard CAN-ID"))
    };
    CanFrame::new(id, data.as_slice()).expect("valid CAN frame")
}

/// Extract the raw COB-ID (u32) from a CAN frame.
fn can_frame_cob(frame: &CanFrame) -> u32 {
    match frame.id() {
        Id::Standard(id) => id.as_raw() as u32,
        Id::Extended(id) => id.as_raw(),
    }
}

// ── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::can_driver::MockCanBus;

    /// Enqueue a mock SDO write-success response.
    fn push_sdo_ack(mock: &mut MockCanBus, node_id: u8, index: u16, subindex: u8, data: u32) {
        let cob = SDO_RX_BASE + node_id as u32;
        let mut buf = [0u8; 8];
        buf[0] = SDO_WRITE_SUCCESS;
        buf[1..3].copy_from_slice(&index.to_le_bytes());
        buf[3] = subindex;
        buf[4..8].copy_from_slice(&data.to_le_bytes());
        mock.push_response(make_can_frame(cob, &buf));
    }

    /// Enqueue a mock SDO read response with 4-byte data.
    fn push_sdo_read(mock: &mut MockCanBus, node_id: u8, index: u16, subindex: u8, data: u32) {
        let cob = SDO_RX_BASE + node_id as u32;
        let mut buf = [0u8; 8];
        buf[0] = SDO_READ_4BYTE;
        buf[1..3].copy_from_slice(&index.to_le_bytes());
        buf[3] = subindex;
        buf[4..8].copy_from_slice(&data.to_le_bytes());
        mock.push_response(make_can_frame(cob, &buf));
    }

    /// Enqueue a mock SDO error response.
    fn push_sdo_error(mock: &mut MockCanBus, node_id: u8, index: u16, subindex: u8, abort: u32) {
        let cob = SDO_RX_BASE + node_id as u32;
        let mut buf = [0u8; 8];
        buf[0] = SDO_ERROR;
        buf[1..3].copy_from_slice(&index.to_le_bytes());
        buf[3] = subindex;
        buf[4..8].copy_from_slice(&abort.to_le_bytes());
        mock.push_response(make_can_frame(cob, &buf));
    }

    #[test]
    fn test_write_od_success() {
        let mut mock = MockCanBus::new();
        push_sdo_ack(&mut mock, 1, od::CONTROL_WORD, 0, 0x06);

        let mut client = SdoClient::new(mock);
        client.write_od(1, od::CONTROL_WORD, 0, 0x06).unwrap();

        assert_eq!(client.can.sent.len(), 1);
        assert_eq!(can_frame_cob(&client.can.sent[0]), SDO_TX_BASE + 1);
    }

    #[test]
    fn test_write_od_multi_node() {
        let mut mock = MockCanBus::new();
        push_sdo_ack(&mut mock, 1, od::CONTROL_WORD, 0, 0x06);
        push_sdo_ack(&mut mock, 2, od::CONTROL_WORD, 0, 0x06);

        let mut client = SdoClient::new(mock);
        client.write_od(1, od::CONTROL_WORD, 0, 0x06).unwrap();
        client.write_od(2, od::CONTROL_WORD, 0, 0x06).unwrap();

        assert_eq!(client.can.sent.len(), 2);
        assert_eq!(can_frame_cob(&client.can.sent[0]), SDO_TX_BASE + 1);
        assert_eq!(can_frame_cob(&client.can.sent[1]), SDO_TX_BASE + 2);
    }

    #[test]
    fn test_write_od_error() {
        let mut mock = MockCanBus::new();
        push_sdo_error(&mut mock, 1, 0x6060, 0, 0x0602_0000);

        let mut client = SdoClient::new(mock);
        let result = client.write_od(1, 0x6060, 0, 0x03);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("SDO error"));
        assert!(err.contains("06020000"));
    }

    #[test]
    fn test_read_od_success() {
        let mut mock = MockCanBus::new();
        push_sdo_read(&mut mock, 1, od::STATUS_WORD, 0, 0x0237);

        let mut client = SdoClient::new(mock);
        let value = client.read_od(1, od::STATUS_WORD, 0).unwrap();
        assert_eq!(value, 0x0237);
    }

    #[test]
    fn test_read_od_timeout() {
        let mock = MockCanBus::new(); // No responses queued
        let mut client = SdoClient::new(mock);

        let result = client.read_od(1, od::STATUS_WORD, 0);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("timeout"));
    }

    #[test]
    fn test_write_od_ignores_wrong_cob_id() {
        let mut mock = MockCanBus::new();
        // Pre-load a frame from a different node (should be ignored)
        let mut buf = [0u8; 8];
        buf[0] = SDO_WRITE_SUCCESS;
        mock.push_response(make_can_frame(SDO_RX_BASE + 99, &buf));
        // Then the actual response we want
        push_sdo_ack(&mut mock, 1, od::CONTROL_WORD, 0, 0x06);

        let mut client = SdoClient::new(mock);
        client.write_od(1, od::CONTROL_WORD, 0, 0x06).unwrap();
    }
}

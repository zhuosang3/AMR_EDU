// Frame header
pub const FRAME_HEADER: [u8; 4] = [0xFA, 0x5A, 0xA5, 0xAA];
pub const FRAME_LENGTH: usize = 1622;
pub const ETX: u8 = 0x66;

// Command bytes (8-byte binary packets)
pub const CMD_START_STREAM: [u8; 8] = [0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x01, 0x01];
pub const CMD_STOP_STREAM: [u8; 8] = [0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x01, 0x01];
pub const CMD_REBOOT: [u8; 8] = [0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x03, 0x03];
pub const CMD_READ_SERIAL: [u8; 8] = [0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x05, 0x05];
pub const CMD_READ_DEVICE_STATE: [u8; 8] = [0xFA, 0x5A, 0xA5, 0xAA, 0x00, 0x02, 0x04, 0x04];

pub const CMD_SIZE: usize = 8;

// Sub-packet header offsets (within a received UDP datagram)
pub const HDR_LENGTH_H: usize = 4;
pub const HDR_LENGTH_L: usize = 5;
pub const HDR_CHECKSUM: usize = 6;
pub const HDR_TYPE: usize = 7;
pub const HDR_TOTAL_IDX_H: usize = 8;
pub const HDR_TOTAL_IDX_L: usize = 9;
pub const HDR_SUB_PKG_NUM: usize = 10;
pub const HDR_SUB_INDEX: usize = 11;
pub const HDR_DATA_START: usize = 12;

// UDP / reassembly limits
pub const RECV_BUFFER_SIZE: usize = 65536;
pub const CMD_FRAME_MAX_LEN: usize = 1500;
pub const CMD_FRAME_MAX_SUB_PKG_NUM: usize = 4;
pub const CMD_FRAME_MIN_SUB_PKG_NUM: usize = 2;

// Distance value semantics
//   1    = system fault
//   2..50000 = normal range (mm)
//   >50000 = no target / low reflectivity
pub const RANGE_FAULT: u16 = 1;
pub const RANGE_MAX_VALID: u16 = 50000;

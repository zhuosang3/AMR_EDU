#pragma once
/**
 * @file protocol.hpp
 * @brief Hipnuc HI12 binary protocol — frame parser, data parser, CRC16.
 *
 * CONFIRMED frame structure (from live HI12M0-MI1-000 captures):
 *
 *   Sync:  0x5A 0xA5
 *   Len:   uint16 LE (payload length, typically 76 = 0x4C)
 *   CRC:   uint16 LE (CCITT 0x1021, over Sync+Len+Payload)
 *   Data:  76-byte fixed struct:
 *     [ 0]  uint8   product_id   (0x91 = HI12 data frame)
 *     [ 1]  uint8[3] header
 *     [ 4]  uint8[4] reserved
 *     [ 8]  uint32  timestamp_ms (LE)
 *     [12]  float32 accel_x (G)
 *     [16]  float32 accel_y (G)
 *     [20]  float32 accel_z (G)
 *     [24]  float32 gyro_x  (°/s)
 *     [28]  float32 gyro_y  (°/s)
 *     [32]  float32 gyro_z  (°/s)
 *     [36]  uint8[12] reserved (zeros)
 *     [48]  float32 pitch (°)
 *     [52]  float32 roll  (°)
 *     [56]  float32 yaw   (°)
 *     [60]  float32 quat_w
 *     [64]  float32 quat_x
 *     [68]  float32 quat_y
 *     [72]  float32 quat_z
 *
 * Total payload: 76 bytes.
 */

#include <cstdint>
#include <cstddef>

namespace hi12_imu {
namespace protocol {

// ============================================================================
// Frame constants
// ============================================================================

constexpr uint8_t  FRAME_PRE        = 0x5A;
constexpr uint8_t  FRAME_TYPE       = 0xA5;
constexpr uint8_t  PRODUCT_ID_HI12  = 0x91;
constexpr uint16_t CRC16_POLY       = 0x1021;
constexpr size_t   FRAME_HEADER_SIZE = 6;   // PRE + TYPE + LEN(2) + CRC(2)
constexpr size_t   HI12_PAYLOAD_SIZE = 76;   // Fixed payload for HI12 data
constexpr size_t   MAX_PAYLOAD_SIZE  = 256;

// ============================================================================
// Payload field offsets within the 76-byte data block
// ============================================================================

constexpr size_t OFF_PRODUCT_ID   = 0;
constexpr size_t OFF_TIMESTAMP    = 8;
constexpr size_t OFF_ACCEL_X      = 12;
constexpr size_t OFF_ACCEL_Y      = 16;
constexpr size_t OFF_ACCEL_Z      = 20;
constexpr size_t OFF_GYRO_X       = 24;
constexpr size_t OFF_GYRO_Y       = 28;
constexpr size_t OFF_GYRO_Z       = 32;
constexpr size_t OFF_EULER_PITCH  = 48;
constexpr size_t OFF_EULER_ROLL   = 52;
constexpr size_t OFF_EULER_YAW    = 56;
constexpr size_t OFF_QUAT_W       = 60;
constexpr size_t OFF_QUAT_X       = 64;
constexpr size_t OFF_QUAT_Y       = 68;
constexpr size_t OFF_QUAT_Z       = 72;

// ============================================================================
// Configuration register addresses (for sending config commands to sensor)
// ============================================================================

enum class CfgReg : uint8_t {
    BAUD_RATE     = 0x01,
    OUTPUT_RATE   = 0x02,
    OUTPUT_SELECT = 0x03,
    MOUNTING_DIR  = 0x04,
    SAVE_CONFIG   = 0x05,
    RESET_FACTORY = 0x06,
};

// Baud rate codes
constexpr uint8_t baud_to_code(int baud) noexcept {
    switch (baud) {
    case 4800:   return 0;
    case 9600:   return 1;
    case 115200: return 2;
    case 460800: return 3;
    case 921600: return 4;
    default:     return 0xFF;
    }
}

constexpr int code_to_baud(uint8_t code) noexcept {
    switch (code) {
    case 0: return 4800;
    case 1: return 9600;
    case 2: return 115200;
    case 3: return 460800;
    case 4: return 921600;
    default: return -1;
    }
}

// ============================================================================
// IMU data struct — all value types, no heap allocation
// ============================================================================

struct ImuData {
    double timestamp   = 0.0;     // ROS time (seconds)
    uint64_t frame_id  = 0;
    uint32_t timestamp_ms = 0;    // raw sensor timestamp (ms)

    // Acceleration (m/s²) — converted from G
    float accel_x = 0.0f;
    float accel_y = 0.0f;
    float accel_z = 0.0f;

    // Angular velocity (rad/s) — converted from °/s
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;

    // Quaternion (W, X, Y, Z) — normalized
    float quat_w = 1.0f;
    float quat_x = 0.0f;
    float quat_y = 0.0f;
    float quat_z = 0.0f;

    // Euler angles (degrees) — raw from sensor
    float pitch = 0.0f;
    float roll  = 0.0f;
    float yaw   = 0.0f;
};

// ============================================================================
// CRC16
// ============================================================================

inline uint16_t crc16_update(uint16_t crc, const uint8_t* data, size_t len) noexcept {
    for (size_t j = 0; j < len; ++j) {
        crc ^= static_cast<uint16_t>(data[j]) << 8;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC16_POLY;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

// ============================================================================
// FrameParser — zero-allocation state machine for sync + CRC validation
// ============================================================================

class FrameParser {
public:
    FrameParser() { reset(); }

    /**
     * @brief Feed one byte. Returns true when a complete, CRC-verified payload is ready.
     */
    bool feed(uint8_t byte, const uint8_t** payload, size_t* len) noexcept;

    void reset() noexcept;
    size_t bytes_consumed() const noexcept { return _bytes_in_frame; }

private:
    enum State { SYNC, LEN_LO, LEN_HI, CRC_LO, CRC_HI, DATA };

    State    _state;
    uint8_t  _buf[FRAME_HEADER_SIZE + MAX_PAYLOAD_SIZE];
    size_t   _buf_pos;
    uint16_t _data_len;
    uint16_t _crc_received;
    size_t   _bytes_in_frame;

    void append(uint8_t byte) noexcept;
};

// ============================================================================
// DataParser — parse 76-byte payload into ImuData struct
// ============================================================================

/**
 * @brief Parse a fixed-struct HI12 data payload into ImuData.
 *
 * @returns true if the payload is a valid HI12 data frame (product_id == 0x91).
 */
bool parse_payload(const uint8_t* payload, size_t len, ImuData& data) noexcept;

// ============================================================================
// Configuration frame builder
// ============================================================================

/**
 * @brief Build a configuration command frame.
 *
 * Configuration commands use a register-address format:
 *   PRE(1) + TYPE(1) + LEN(2) + CRC16(2) + REG_ADDR(1) + REG_DATA(N)
 *
 * @param out_buffer must be at least FRAME_HEADER_SIZE + 1 + reg_len bytes.
 * @returns total frame length in bytes.
 */
size_t build_config_frame(CfgReg reg_addr,
                          const uint8_t* reg_data, size_t reg_len,
                          uint8_t* out_buffer) noexcept;

} // namespace protocol
} // namespace hi12_imu

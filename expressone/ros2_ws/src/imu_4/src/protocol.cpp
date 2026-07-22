#include "hi12_imu/protocol.hpp"

#include <cstring>
#include <algorithm>

namespace hi12_imu {
namespace protocol {

// ============================================================================
// FrameParser implementation
// ============================================================================

void FrameParser::reset() noexcept {
    _state      = SYNC;
    _buf_pos    = 0;
    _data_len   = 0;
    _bytes_in_frame = 0;
}

void FrameParser::append(uint8_t byte) noexcept {
    if (_buf_pos < sizeof(_buf)) {
        _buf[_buf_pos++] = byte;
    }
}

bool FrameParser::feed(uint8_t byte, const uint8_t** payload, size_t* len) noexcept {
    switch (_state) {

    case SYNC:
        append(byte);
        if (_buf_pos >= 2) {
            if (_buf[_buf_pos - 2] == FRAME_PRE &&
                _buf[_buf_pos - 1] == FRAME_TYPE) {
                _buf[0] = FRAME_PRE;
                _buf[1] = FRAME_TYPE;
                _buf_pos = 2;
                _state = LEN_LO;
            } else {
                _buf[0] = _buf[_buf_pos - 1];
                _buf_pos = 1;
            }
        }
        return false;

    case LEN_LO:
        append(byte);
        _state = LEN_HI;
        return false;

    case LEN_HI: {
        append(byte);
        uint16_t len_lo = _buf[2];
        uint16_t len_hi = _buf[3];
        _data_len = len_lo | (len_hi << 8);
        if (_data_len > MAX_PAYLOAD_SIZE) {
            reset();
            return false;
        }
        _state = CRC_LO;
        return false;
    }

    case CRC_LO:
        append(byte);
        _state = CRC_HI;
        return false;

    case CRC_HI: {
        append(byte);
        uint16_t crc_lo = _buf[4];
        uint16_t crc_hi = _buf[5];
        _crc_received = crc_lo | (crc_hi << 8);
        _state = DATA;
        return false;
    }

    case DATA:
        append(byte);
        if (_buf_pos >= FRAME_HEADER_SIZE + _data_len) {
            // Verify CRC: covers PRE+TYPE+LEN(4 bytes) + DATA
            uint16_t computed = 0;
            computed = crc16_update(computed, _buf, 4);              // PRE + TYPE + LEN
            computed = crc16_update(computed, _buf + 6, _data_len);  // DATA

            _bytes_in_frame = _buf_pos;

            if (computed == _crc_received) {
                *payload = _buf + FRAME_HEADER_SIZE;
                *len = _data_len;
                reset();
                return true;
            }

            // CRC mismatch — hunt for next sync
            reset();
            return false;
        }
        return false;
    }

    return false;
}

// ============================================================================
// DataParser — fixed-struct HI12 payload
// ============================================================================

bool parse_payload(const uint8_t* payload, size_t len, ImuData& data) noexcept {
    if (payload == nullptr || len < HI12_PAYLOAD_SIZE) {
        return false;
    }

    // Verify product ID
    if (payload[OFF_PRODUCT_ID] != PRODUCT_ID_HI12) {
        return false;
    }

    // Timestamp (uint32 LE, milliseconds)
    data.timestamp_ms = static_cast<uint32_t>(payload[OFF_TIMESTAMP])
                      | (static_cast<uint32_t>(payload[OFF_TIMESTAMP + 1]) << 8)
                      | (static_cast<uint32_t>(payload[OFF_TIMESTAMP + 2]) << 16)
                      | (static_cast<uint32_t>(payload[OFF_TIMESTAMP + 3]) << 24);

    // Accelerometer — raw values in G, convert to m/s²
    constexpr float G_TO_MS2 = 9.80665f;
    float ax, ay, az;
    std::memcpy(&ax, payload + OFF_ACCEL_X, 4);
    std::memcpy(&ay, payload + OFF_ACCEL_Y, 4);
    std::memcpy(&az, payload + OFF_ACCEL_Z, 4);
    data.accel_x = ax * G_TO_MS2;
    data.accel_y = ay * G_TO_MS2;
    data.accel_z = az * G_TO_MS2;

    // Gyroscope — raw values in °/s, convert to rad/s
    constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;
    float gx, gy, gz;
    std::memcpy(&gx, payload + OFF_GYRO_X, 4);
    std::memcpy(&gy, payload + OFF_GYRO_Y, 4);
    std::memcpy(&gz, payload + OFF_GYRO_Z, 4);
    data.gyro_x = gx * DEG_TO_RAD;
    data.gyro_y = gy * DEG_TO_RAD;
    data.gyro_z = gz * DEG_TO_RAD;

    // Euler angles — raw values in degrees
    std::memcpy(&data.pitch, payload + OFF_EULER_PITCH, 4);
    std::memcpy(&data.roll,  payload + OFF_EULER_ROLL,  4);
    std::memcpy(&data.yaw,   payload + OFF_EULER_YAW,   4);

    // Quaternion — raw float values (normalized)
    std::memcpy(&data.quat_w, payload + OFF_QUAT_W, 4);
    std::memcpy(&data.quat_x, payload + OFF_QUAT_X, 4);
    std::memcpy(&data.quat_y, payload + OFF_QUAT_Y, 4);
    std::memcpy(&data.quat_z, payload + OFF_QUAT_Z, 4);

    return true;
}

// ============================================================================
// Configuration frame builder (register-address format for commands)
// ============================================================================

size_t build_config_frame(CfgReg reg_addr,
                          const uint8_t* reg_data, size_t reg_len,
                          uint8_t* out_buffer) noexcept {
    // Frame: PRE(1) + TYPE(1) + LEN(2) + CRC16(2) + REG_ADDR(1) + REG_DATA(reg_len)
    size_t payload_len = 1 + reg_len;
    size_t frame_len = FRAME_HEADER_SIZE + payload_len;

    out_buffer[0] = FRAME_PRE;
    out_buffer[1] = FRAME_TYPE;

    uint16_t len = static_cast<uint16_t>(payload_len);
    out_buffer[2] = static_cast<uint8_t>(len & 0xFF);
    out_buffer[3] = static_cast<uint8_t>((len >> 8) & 0xFF);

    out_buffer[6] = static_cast<uint8_t>(reg_addr);
    if (reg_len > 0 && reg_data != nullptr) {
        std::memcpy(out_buffer + 7, reg_data, reg_len);
    }

    uint16_t crc = 0;
    crc = crc16_update(crc, out_buffer, 4);               // PRE + TYPE + LEN
    crc = crc16_update(crc, out_buffer + 6, payload_len); // REG_ADDR + REG_DATA

    out_buffer[4] = static_cast<uint8_t>(crc & 0xFF);
    out_buffer[5] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    return frame_len;
}

} // namespace protocol
} // namespace hi12_imu

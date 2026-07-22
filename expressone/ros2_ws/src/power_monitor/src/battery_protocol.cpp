#include "power_monitor/battery_protocol.hpp"

namespace {

inline uint16_t be16u(const uint8_t* d) { return (d[0] << 8) | d[1]; }
inline int16_t  be16s(const uint8_t* d) { return static_cast<int16_t>((d[0] << 8) | d[1]); }

void parse_0x200(const uint8_t* d, BatteryData& out) {
  out.total_voltage    = be16u(&d[0]);  // BYTE0~1, 10mV
  out.current          = be16s(&d[2]);  // BYTE2~3, 10mA signed
  out.min_cell_voltage = be16u(&d[4]);  // BYTE4~5, mV
  out.max_cell_voltage = be16u(&d[6]);  // BYTE6~7, mV
}

void parse_0x201(const uint8_t* d, BatteryData& out) {
  out.remaining_capacity = be16u(&d[0]);  // BYTE0~1, 10mAh
  out.full_capacity      = be16u(&d[2]);  // BYTE2~3, 10mAh
  out.cycle_count        = be16u(&d[4]);  // BYTE4~5, cycle count
  out.rsoc               = be16u(&d[6]);  // BYTE6~7, RSOC %
}

void parse_0x202(const uint8_t* d, BatteryData& out) {
  out.protection_flags = be16u(&d[0]);  // BYTE0~1, protection flags
  out.charge_flag      = d[2];           // BYTE2, charge flag
  out.mosfet_status    = d[3];           // BYTE3, MOSFET status
  out.max_temperature  = be16s(&d[4]);  // BYTE4~5, max temp 0.1℃
  out.min_temperature  = be16s(&d[6]);  // BYTE6~7, min temp 0.1℃
}

}  // namespace

void parse_can_frame(uint32_t can_id, const uint8_t* data, uint8_t len, BatteryData& out) {
  if (len < 8) return;
  switch (can_id) {
    case 0x200: parse_0x200(data, out); break;
    case 0x201: parse_0x201(data, out); break;
    case 0x202: parse_0x202(data, out); break;
    default:    break;
  }
}

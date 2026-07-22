#pragma once

#include <cstdint>

struct BatteryData {
  // 0x200: Voltage/Current
  uint16_t total_voltage    = 0;  ///< unit: 10mV
  int16_t  current          = 0;  ///< unit: 10mA (signed: charge+/discharge-)
  uint16_t min_cell_voltage = 0;  ///< unit: mV
  uint16_t max_cell_voltage = 0;  ///< unit: mV

  // 0x201: Capacity
  uint16_t remaining_capacity = 0;  ///< unit: 10mAh
  uint16_t full_capacity      = 0;  ///< unit: 10mAh
  uint16_t cycle_count        = 0;  ///< cycle count
  uint16_t rsoc               = 0;  ///< RSOC, unit: %

  // 0x202: Temperature/Status
  uint16_t protection_flags = 0;  ///< protection flags (see enum)
  uint8_t  charge_flag      = 0;  ///< 0=not charging, 1=charging
  uint8_t  mosfet_status    = 0;  ///< bit0=charge MOS, bit1=discharge MOS
  int16_t  max_temperature  = 0;  ///< unit: 0.1℃ (signed)
  int16_t  min_temperature  = 0;  ///< unit: 0.1℃ (signed)
};

enum ProtectionBit : uint16_t {
  PROT_CELL_OVERVOLTAGE      = 1 << 0,   ///< bit0  单体过压保护
  PROT_CELL_UNDERVOLTAGE     = 1 << 1,   ///< bit1  单体欠压保护
  PROT_PACK_OVERVOLTAGE      = 1 << 2,   ///< bit2  整组过压保护
  PROT_PACK_UNDERVOLTAGE     = 1 << 3,   ///< bit3  整组欠压保护
  PROT_CHARGE_OVERTEMP       = 1 << 4,   ///< bit4  充电过温保护
  PROT_CHARGE_LOWTEMP        = 1 << 5,   ///< bit5  充电低温保护
  PROT_DISCHARGE_OVERTEMP    = 1 << 6,   ///< bit6  放电过温保护
  PROT_DISCHARGE_LOWTEMP     = 1 << 7,   ///< bit7  放电低温保护
  PROT_CHARGE_OVERCURRENT    = 1 << 8,   ///< bit8  充电过流保护
  PROT_DISCHARGE_OVERCURRENT = 1 << 9,   ///< bit9  放电过流保护
  PROT_SHORT_CIRCUIT         = 1 << 10,  ///< bit10 短路保护
  // bit11~13: reserved
  ALARM_LOW_BATTERY_30       = 1 << 14,  ///< bit14 低电量报警 (SOC<30%)
  ALARM_LOW_BATTERY_10       = 1 << 15,  ///< bit15 低电量保护 (SOC<10%)
};

void parse_can_frame(uint32_t can_id, const uint8_t* data, uint8_t len, BatteryData& out);

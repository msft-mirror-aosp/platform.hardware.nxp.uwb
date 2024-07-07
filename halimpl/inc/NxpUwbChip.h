#ifndef _NXPUWB_CHIP_H_
#define _NXPUWB_CHIP_H_

#include <cstddef>
#include <cstdint>

#include <memory>

#include "phUwbTypes.h"

// Chip type
typedef enum {
  DEVICE_TYPE_UNKNOWN,
  DEVICE_TYPE_SR1xxT,
  DEVICE_TYPE_SR1xxS,
  DEVICE_TYPE_SR200,
} device_type_t;

// SW defined data structures
typedef enum {
  // 6 bytes
  // [1:0] cap1 [3:2] cap2 [5:4] gm current control
  EXTCAL_PARAM_CLK_ACCURACY   = 0x1,    // xtal

  // 3n + 1 bytes
  // [0] n, number of entries +  n * { [0] antenna-id [1:0] RX delay(Q14.2) }
  EXTCAL_PARAM_RX_ANT_DELAY   = 0x2,    // ant_delay

  // 5N + 1 bytes
  // [0]: n, number of entries + n * { [0] antenna-id [2:1] delta-peak [4:3] id-rms }
  EXTCAL_PARAM_TX_POWER       = 0x3,    // tx_power

  // channel independent
  // 1 byte
  //  b0: enable/disable DDFS tone generation (default off)
  //  b1: enable/disable DC suppression (default off)
  EXTCAL_PARAM_TX_BASE_BAND_CONTROL   = 0x101,  // ddfs_enable, dc_suppress

  // channel independent (raw data contains channel info)
  // bytes array
  EXTCAL_PARAM_DDFS_TONE_CONFIG       = 0x102,  // ddfs_tone_config

  // channel independent
  // byte array
  EXTCAL_PARAM_TX_PULSE_SHAPE         = 0x103,  // tx_pulse_shape
} extcal_param_id_t;

class NxpUwbChip {
public:
  virtual ~NxpUwbChip() = default;

  // Bring-up the chip into UCI operational modes
  // FW donwloading and enter UCI mode
  virtual tHAL_UWB_STATUS chip_init() = 0;

  // Per-chip device configurations
  // Binding check, life cycle check.
  virtual tHAL_UWB_STATUS core_init() = 0;

  // Determine device_type_t from DEVICE_INFO_RSP::UWB_CHIP_ID
  virtual device_type_t get_device_type(const uint8_t* param, size_t param_len) = 0;

  // Read Calibration parameters storead at OTP
  virtual tHAL_UWB_STATUS read_otp(extcal_param_id_t id,
                                   uint8_t *data,
                                   size_t data_len,
                                   size_t *retlen);

  // Apply device calibration
  virtual tHAL_UWB_STATUS apply_calibration(extcal_param_id_t id,
                                           const uint8_t ch,
                                           const uint8_t *data,
                                           size_t data_len) = 0;
};

std::unique_ptr<NxpUwbChip> GetUwbChip();

#endif
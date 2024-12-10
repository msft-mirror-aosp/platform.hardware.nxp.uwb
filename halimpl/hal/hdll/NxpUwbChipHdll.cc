#include "NxpUwbChip.h"
#include "phNxpConfig.h"
#include "phNxpUciHal.h"
#include "phNxpUciHal_ext.h"
#include "phUwbStatus.h"
#include "phUwbTypes.h"
#include "phNxpUwbCalib.h"
#include "uci_defs.h"

#define UCI_MSG_UWB_ESE_BINDING_LEN                   11
#define UCI_MSG_UWB_ESE_BINDING_OFFSET_COUNT          5
#define UCI_MSG_UWB_ESE_BINDING_OFFSET_BINDING_STATE  6

extern phNxpUciHal_Control_t nxpucihal_ctrl;
extern int hdll_fw_download();

class NxpUwbChipSr200 final : public NxpUwbChip {
public:
  NxpUwbChipSr200();
  virtual ~NxpUwbChipSr200();

  tHAL_UWB_STATUS chip_init();
  tHAL_UWB_STATUS core_init();
  device_type_t get_device_type(const uint8_t *param, size_t param_len);
  tHAL_UWB_STATUS read_otp(extcal_param_id_t id, uint8_t *data, size_t data_len, size_t *retlen);
  tHAL_UWB_STATUS apply_calibration(extcal_param_id_t id, const uint8_t ch, const uint8_t *data, size_t data_len);
  tHAL_UWB_STATUS get_supported_channels(const uint8_t **cal_channels, uint8_t *nr);
private:
  bool on_binding_status_ntf(size_t packet_len, const uint8_t* packet);

  tHAL_UWB_STATUS check_binding_done();

  UciHalRxHandler bindingStatusNtfHandler_;
  UciHalSemaphore bindingStatusNtfWait_;
  uint8_t bindingStatus_;
};

NxpUwbChipSr200::NxpUwbChipSr200() :
  bindingStatus_(UWB_DEVICE_UNKNOWN)
{
}

NxpUwbChipSr200::~NxpUwbChipSr200()
{
}

bool NxpUwbChipSr200::on_binding_status_ntf(size_t packet_len, const uint8_t* packet)
{
  if (packet_len >= UCI_RESPONSE_STATUS_OFFSET) {
    bindingStatus_ = packet[UCI_RESPONSE_STATUS_OFFSET];
    NXPLOG_UCIHAL_D("BINDING_STATUS_NTF: 0x%x", bindingStatus_);
    bindingStatusNtfWait_.post(UWBSTATUS_SUCCESS);
  }
  return true;
}

tHAL_UWB_STATUS NxpUwbChipSr200::check_binding_done()
{
  // Wait for Binding status notification
  if (bindingStatusNtfWait_.getStatus() != UWBSTATUS_SUCCESS) {
    bindingStatusNtfWait_.wait();
  }
  if (bindingStatusNtfWait_.getStatus() != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("Binding status notification timeout");

    // Stop HAL init when it didn't receive the binding notification
    if (nxpucihal_ctrl.fw_boot_mode == USER_FW_BOOT_MODE)
      return UWBSTATUS_FAILED;
    else
      return UWBSTATUS_SUCCESS;
  }

  switch (bindingStatus_) {
  case UWB_DEVICE_NOT_BOUND:
    NXPLOG_UCIHAL_E("Binding status: Unbound.");
    break;
  case UWB_DEVICE_BOUND_UNLOCKED:
    NXPLOG_UCIHAL_E("Binding status: bound & unlocked.");
    break;
  case UWB_DEVICE_BOUND_LOCKED:
    NXPLOG_UCIHAL_D("Binding status: bound & locked.");
    break;
  case UWB_DEVICE_UNKNOWN:
    NXPLOG_UCIHAL_D("Binding status: Unknown.");
    break;
  default:
    NXPLOG_UCIHAL_E("Unknown binding status: 0x%x", bindingStatus_);
    return UWBSTATUS_FAILED;
  }

  return UWBSTATUS_SUCCESS;
}

tHAL_UWB_STATUS NxpUwbChipSr200::chip_init()
{
  tHAL_UWB_STATUS status;

  // system in FW download mode
  // This will be cleared on first Device Status NTF
  nxpucihal_ctrl.fw_dwnld_mode = true;

  NXPLOG_UCIHAL_D("Start SR200 FW download");

  for (int i = 0; i < 5; i++) {
    phTmlUwb_Chip_Reset();

    status = hdll_fw_download();

    if (status == UWBSTATUS_SUCCESS) {
      NXPLOG_UCIHAL_D("Complete SR200 FW download");
      break;
    } else if(status == UWBSTATUS_FILE_NOT_FOUND) {
      NXPLOG_UCIHAL_E("FW file Not found.");
      break;
    } else {
      NXPLOG_UCIHAL_E("FW download failed, status= 0x%x, retry.", status);
    }
  }

  // register binding status ntf handler
  bindingStatusNtfHandler_ = UciHalRxHandler(
      UCI_MT_NTF, UCI_GID_PROPRIETARY, UCI_MSG_BINDING_STATUS_NTF,
      std::bind(&NxpUwbChipSr200::on_binding_status_ntf, this, std::placeholders::_1, std::placeholders::_2));

  return status;
}

tHAL_UWB_STATUS NxpUwbChipSr200::core_init()
{
  return check_binding_done();
}

device_type_t NxpUwbChipSr200::get_device_type(const uint8_t *param, size_t param_len)
{
  // should be 'SR200..'
  const char marker[] = { 'S', 'R', '2', '0', '0' };
  if (param_len >= sizeof(marker)) {
    if (!memcmp(param, marker, sizeof(marker)))
      return DEVICE_TYPE_SR200;
  }
  return DEVICE_TYPE_UNKNOWN;
}

tHAL_UWB_STATUS
NxpUwbChipSr200::read_otp(extcal_param_id_t id,
                          uint8_t *data, size_t data_len, size_t *retlen)
{
  return UWBSTATUS_NOT_ALLOWED;
}

tHAL_UWB_STATUS
NxpUwbChipSr200::apply_calibration(extcal_param_id_t id, const uint8_t ch,
                                   const uint8_t *data, size_t data_len)
{
  switch (id) {
  case EXTCAL_PARAM_TX_POWER:
  case EXTCAL_PARAM_TX_BASE_BAND_CONTROL:
  case EXTCAL_PARAM_DDFS_TONE_CONFIG:
  case EXTCAL_PARAM_TX_PULSE_SHAPE:
  case EXTCAL_PARAM_RX_ANT_DELAY:
    return sr1xx_apply_calibration(id, ch, data, data_len);
  case EXTCAL_PARAM_CLK_ACCURACY:
    /* break through */
  default:
    NXPLOG_UCIHAL_E("Unsupported parameter: 0x%x", id);
    return UWBSTATUS_FAILED;
  }
}

tHAL_UWB_STATUS
NxpUwbChipSr200::get_supported_channels(const uint8_t **cal_channels, uint8_t *nr)
{
  static const uint8_t sr200_cal_channels[] = {5, 9, 10};
  *cal_channels = sr200_cal_channels;
  *nr = std::size(sr200_cal_channels);
  return UWBSTATUS_SUCCESS;
}

std::unique_ptr<NxpUwbChip> GetUwbChip()
{
  return std::make_unique<NxpUwbChipSr200>();
}

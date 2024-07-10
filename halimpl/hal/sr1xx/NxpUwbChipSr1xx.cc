#include "NxpUwbChip.h"
#include "phNxpConfig.h"
#include "phNxpUciHal.h"
#include "phNxpUciHal_ext.h"
#include "phNxpUciHal_utils.h"
#include "phUwbStatus.h"
#include "phUwbTypes.h"
#include "phNxpUwbCalib.h"
#include "uci_defs.h"

#define UCI_MSG_UWB_ESE_BINDING_LEN                   11
#define UCI_MSG_UWB_ESE_BINDING_OFFSET_COUNT          5
#define UCI_MSG_UWB_ESE_BINDING_OFFSET_BINDING_STATE  6

extern phNxpUciHal_Control_t nxpucihal_ctrl;

static void report_binding_status(uint8_t binding_status)
{
  // BINDING_STATUS_NTF
  uint8_t data_len = 5;
  uint8_t buffer[5];
  buffer[0] = 0x6E;
  buffer[1] = 0x06;
  buffer[2] = 0x00;
  buffer[3] = 0x01;
  buffer[4] = binding_status;
  nxpucihal_ctrl.rx_data_len = 5;
  if (nxpucihal_ctrl.p_uwb_stack_data_cback != NULL) {
    (*nxpucihal_ctrl.p_uwb_stack_data_cback)(data_len, buffer);
  }
}

/******************************************************************************
 * Function         otp_read_data
 *
 * Description      Read OTP calibration data
 *
 * Returns          true on success
 *
 ******************************************************************************/
static bool otp_read_data(const uint8_t channel, const uint8_t param_id, uint8_t *buffer, size_t len)
{
  phNxpUciHal_Sem_t calib_data_ntf_wait;
  phNxpUciHal_init_cb_data(&calib_data_ntf_wait, NULL);

  // NXP_READ_CALIB_DATA_NTF
  bool received = false;
  auto read_calib_ntf_cb =
  [&] (size_t packet_len, const uint8_t *packet) mutable
  {
    // READ_CALIB_DATA_NTF: status(1), length-of-payload(1), payload(N)
    const uint8_t plen = packet[3]; // payload-length
    const uint8_t *p = &packet[4];  // payload

    if (plen < 2) {
      NXPLOG_UCIHAL_E("Otp read: bad payload length %u", plen);
    } else if (p[0] != UCI_STATUS_OK) {
      NXPLOG_UCIHAL_E("Otp read: bad status=0x%x", nxpucihal_ctrl.p_rx_data[4]);
    } else if (p[1] != len) {
      NXPLOG_UCIHAL_E("Otp read: size mismatch %u (expected %zu for param 0x%x)",
        p[1], len, param_id);
    } else {
      memcpy(buffer, &p[2], len);
      received = true;
      SEM_POST(&calib_data_ntf_wait);
    }
  };
  auto handler = phNxpUciHal_rx_handler_add(
      UCI_MT_NTF, UCI_GID_PROPRIETARY_0X0A, UCI_MSG_READ_CALIB_DATA,
      true, true, read_calib_ntf_cb);


  // READ_CALIB_DATA_CMD
  std::vector<uint8_t> packet{(UCI_MT_CMD << UCI_MT_SHIFT) | UCI_GID_PROPRIETARY_0X0A, UCI_MSG_READ_CALIB_DATA, 0x00, 0x03};
  packet.push_back(channel);
  packet.push_back(0x01);      // OTP read option
  packet.push_back(param_id);

  tHAL_UWB_STATUS status = phNxpUciHal_send_ext_cmd(packet.size(), packet.data());
  if (status != UWBSTATUS_SUCCESS) {
    goto fail_otp_read_data;
  }

  phNxpUciHal_sem_timed_wait_sec(&calib_data_ntf_wait, 3);
  if (!received) {
    goto fail_otp_read_data;
  }

  phNxpUciHal_cleanup_cb_data(&calib_data_ntf_wait);
  return true;

fail_otp_read_data:
  phNxpUciHal_cleanup_cb_data(&calib_data_ntf_wait);
  NXPLOG_UCIHAL_E("Failed to read OTP data id=%u", param_id);
  return false;
}

static tHAL_UWB_STATUS sr1xx_read_otp(extcal_param_id_t id, uint8_t *data, size_t data_len, size_t *retlen)
{
  switch(id) {
  case EXTCAL_PARAM_CLK_ACCURACY:
    {
      const size_t param_len = 6;
      uint8_t otp_xtal_data[3];

      if (data_len < param_len) {
        NXPLOG_UCIHAL_E("Requested RF_CLK_ACCURACY_CALIB with %zu bytes (expected >= %zu)", data_len, param_len);
        return UWBSTATUS_FAILED;
      }
      if (!otp_read_data(0x09, OTP_ID_XTAL_CAP_GM_CTRL, otp_xtal_data, sizeof(otp_xtal_data))) {
        NXPLOG_UCIHAL_E("Failed to read OTP XTAL_CAP_GM_CTRL");
        return UWBSTATUS_FAILED;
      }
      memset(data, 0, param_len);
      // convert OTP_ID_XTAL_CAP_GM_CTRL to EXTCAL_PARAM_RX_ANT_DELAY
      data[0] = otp_xtal_data[0]; // cap1
      data[2] = otp_xtal_data[1]; // cap2
      data[4] = otp_xtal_data[2]; // gm_current_control (default: 0x30)
      *retlen = param_len;
      return UWBSTATUS_SUCCESS;
    }
    break;
  default:
    NXPLOG_UCIHAL_E("Unsupported otp parameter %d", id);
    return UWBSTATUS_FAILED;
  }
}

//
// SR1XX Error handlers (Thermal Runaway, LOW VBATT)
//

static void sr1xx_handle_device_error()
{
 /* Send FW crash NTF to upper layer for triggering MW recovery */
  phNxpUciHal_send_dev_error_status_ntf();
}

static void sr1xx_clear_device_error()
{
}

//
// SE binding
//

// Temporarily disable DPD for binding, vendor config should re-enable it
static tHAL_UWB_STATUS sr1xx_disable_dpd()
{
  uint8_t buffer[] = {0x20, 0x04, 0x00, 0x04, 0x01, 0x01, 0x01, 0x00};
  return phNxpUciHal_send_ext_cmd(sizeof(buffer), buffer);
}

/******************************************************************************
 * Function         sr1xx_do_bind
 *
 * Description      Sends UWB_ESE_BINDING_CMD and returns
 *                  updated binding status and remaining UWBS binding count
 *
 * Returns          status
 *
 ******************************************************************************/
static tHAL_UWB_STATUS sr1xx_do_bind(uint8_t *binding_status, uint8_t *remain_count)
{
  tHAL_UWB_STATUS status;

  // register rx handler for UWB_ESE_BINDING_NTF
  phNxpUciHal_Sem_t binding_ntf_wait;
  phNxpUciHal_init_cb_data(&binding_ntf_wait, NULL);

  auto binding_ntf_cb =
    [&](size_t packet_len, const uint8_t *packet) mutable
  {
      if (packet_len == UCI_MSG_UWB_ESE_BINDING_LEN) {
        uint8_t status = packet[UCI_RESPONSE_STATUS_OFFSET];
        if (status != UWBSTATUS_SUCCESS) {
          NXPLOG_UCIHAL_E("UWB_ESE_BINDING_NTF: Binding failed, status=0x%x", status);
        }
        *binding_status = packet[UCI_MSG_UWB_ESE_BINDING_OFFSET_BINDING_STATE];
        *remain_count = packet[UCI_MSG_UWB_ESE_BINDING_OFFSET_COUNT];
        NXPLOG_UCIHAL_D("Received UWB_ESE_BINDING_NTF, status=0x%x, binding_state=0x%x, count=%u",
          status, *binding_status, *remain_count);
        SEM_POST(&binding_ntf_wait);
      } else {
        NXPLOG_UCIHAL_E("UWB_ESE_BINDING_NTF: packet length mismatched %zu", packet_len);
      }
  };
  auto handler = phNxpUciHal_rx_handler_add(
      UCI_MT_NTF, UCI_GID_PROPRIETARY_0X0F, UCI_MSG_UWB_ESE_BINDING,
      true, true, binding_ntf_cb);

  // UWB_ESE_BINDING_CMD
  uint8_t buffer[] = {0x2F, 0x31, 0x00, 0x00};
  status = phNxpUciHal_send_ext_cmd(sizeof(buffer), buffer);
  if (status != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("Failed to send UWB_ESE_BINDING_CMD");
    goto exit_do_bind;
  }

  if (phNxpUciHal_sem_timed_wait(&binding_ntf_wait) ||
      binding_ntf_wait.status != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("Failed to retrieve UWB_ESE_BINDING_NTF");
    goto exit_do_bind;
  }

  status = UWBSTATUS_SUCCESS;

exit_do_bind:
  phNxpUciHal_rx_handler_del(handler);
  phNxpUciHal_cleanup_cb_data(&binding_ntf_wait);
  return status;
}

/******************************************************************************
 * Function         sr1xx_check_binding_status
 *
 * Description      Send UWB_ESE_BINDING_CHECK_CMD and returns updated binding status
 *
 * Returns          status
 *
 ******************************************************************************/
static tHAL_UWB_STATUS sr1xx_check_binding_status(uint8_t *binding_status)
{
  tHAL_UWB_STATUS status;
  *binding_status = UWB_DEVICE_UNKNOWN;

  // register rx handler for UWB_ESE_BINDING_CHECK_NTF
  uint8_t binding_status_got = UWB_DEVICE_UNKNOWN;
  phNxpUciHal_Sem_t binding_check_ntf_wait;
  phNxpUciHal_init_cb_data(&binding_check_ntf_wait, NULL);
  auto binding_check_ntf_cb = [&](size_t packet_len, const uint8_t *packet) mutable {
    if (packet_len >= UCI_RESPONSE_STATUS_OFFSET) {
      binding_status_got = packet[UCI_RESPONSE_STATUS_OFFSET];
      NXPLOG_UCIHAL_D("Received UWB_ESE_BINDING_CHECK_NTF, binding_status=0x%x", binding_status_got);
      SEM_POST(&binding_check_ntf_wait);
    }
  };
  auto handler = phNxpUciHal_rx_handler_add(
      UCI_MT_NTF, UCI_GID_PROPRIETARY_0X0F, UCI_MSG_UWB_ESE_BINDING_CHECK,
      true, true, binding_check_ntf_cb);

  // UWB_ESE_BINDING_CHECK_CMD
  uint8_t lock_cmd[] = {0x2F, 0x32, 0x00, 0x00};
  status = phNxpUciHal_send_ext_cmd(sizeof(lock_cmd), lock_cmd);
  if (status != UWBSTATUS_SUCCESS) {
    goto exit_check_binding_status;
  }

  if (phNxpUciHal_sem_timed_wait(&binding_check_ntf_wait) ||
      binding_check_ntf_wait.status != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("Failed to retrieve UWB_ESE_BINDING_CHECK_NTF");
    goto exit_check_binding_status;
  }

  *binding_status = binding_status_got;
  status = UWBSTATUS_SUCCESS;

exit_check_binding_status:
  phNxpUciHal_rx_handler_del(handler);
  phNxpUciHal_cleanup_cb_data(&binding_check_ntf_wait);
  return status;
}

class NxpUwbChipSr1xx final : public NxpUwbChip {
public:
  NxpUwbChipSr1xx();
  virtual ~NxpUwbChipSr1xx();

  tHAL_UWB_STATUS chip_init();
  tHAL_UWB_STATUS core_init();
  device_type_t get_device_type(const uint8_t *param, size_t param_len);
  tHAL_UWB_STATUS read_otp(extcal_param_id_t id, uint8_t *data, size_t data_len, size_t *retlen);
  tHAL_UWB_STATUS apply_calibration(extcal_param_id_t id, const uint8_t ch, const uint8_t *data, size_t data_len);
  int16_t extra_group_delay(void);

private:
  tHAL_UWB_STATUS check_binding();
  void onDeviceStatusNtf(size_t packet_len, const uint8_t* packet);
  void onGenericErrorNtf(size_t packet_len, const uint8_t* packet);
  void onBindingStatusNtf(size_t packet_len, const uint8_t* packet);

private:
  UciHalRxHandler deviceStatusNtfHandler_;
  UciHalRxHandler genericErrorNtfHandler_;
  UciHalRxHandler bindingStatusNtfHandler_;
  UciHalSemaphore bindingStatusNtfWait_;
  uint8_t bindingStatus_;
};

NxpUwbChipSr1xx::NxpUwbChipSr1xx() :
  bindingStatus_(UWB_DEVICE_UNKNOWN)
{
}

NxpUwbChipSr1xx::~NxpUwbChipSr1xx()
{
}

void NxpUwbChipSr1xx::onDeviceStatusNtf(size_t packet_len, const uint8_t* packet)
{
  if(packet_len > UCI_RESPONSE_STATUS_OFFSET) {
    uint8_t status = packet[UCI_RESPONSE_STATUS_OFFSET];
    if (status == UCI_STATUS_HW_RESET) {
      sr1xx_clear_device_error();
    }
  }
}

void NxpUwbChipSr1xx::onGenericErrorNtf(size_t packet_len, const uint8_t* packet)
{
  if(packet_len > UCI_RESPONSE_STATUS_OFFSET) {
    uint8_t status = packet[UCI_RESPONSE_STATUS_OFFSET];
    if ( status == UCI_STATUS_THERMAL_RUNAWAY || status == UCI_STATUS_LOW_VBAT) {
      nxpucihal_ctrl.isSkipPacket = 1;
      sr1xx_handle_device_error();
    }
  }
}

void NxpUwbChipSr1xx::onBindingStatusNtf(size_t packet_len, const uint8_t* packet)
{
  if (packet_len > UCI_RESPONSE_STATUS_OFFSET) {
    bindingStatus_ = packet[UCI_RESPONSE_STATUS_OFFSET];
    NXPLOG_UCIHAL_D("BINDING_STATUS_NTF: 0x%x", bindingStatus_);
    bindingStatusNtfWait_.post(UWBSTATUS_SUCCESS);
  }
}

tHAL_UWB_STATUS NxpUwbChipSr1xx::check_binding()
{
  // Wait for Binding status notification
  if (bindingStatusNtfWait_.getStatus() != UWBSTATUS_SUCCESS) {
    bindingStatusNtfWait_.wait_timeout_msec(3000);
  }
  if (bindingStatusNtfWait_.getStatus() != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("Binding status notification timeout");

    // Stop HAL init when it didn't receive the binding notification.
    // or if it's not user mode fw, just continue
    if (nxpucihal_ctrl.fw_boot_mode == USER_FW_BOOT_MODE)
      return UWBSTATUS_FAILED;
    else
      return UWBSTATUS_SUCCESS;
  }

  uint32_t val = 0;
  NxpConfig_GetNum(NAME_UWB_BINDING_LOCKING_ALLOWED, &val, sizeof(val));
  bool isBindingLockingAllowed = !!val;
  if (!isBindingLockingAllowed) {
    return UWBSTATUS_SUCCESS;
  }

  NXPLOG_UCIHAL_E("Current binding status: 0x%x", bindingStatus_);

  switch (bindingStatus_) {
    case UWB_DEVICE_UNKNOWN:
      // Treat 'UNKNOWN' state as 'NOT_BOUND'
      NXPLOG_UCIHAL_E("Unknown binding status, proceed binding.");
      [[fallthrough]];
    case UWB_DEVICE_NOT_BOUND:
    {
      sr1xx_disable_dpd();

      // perform bind
      uint8_t remaining_count = 0;
      tHAL_UWB_STATUS status = sr1xx_do_bind(&bindingStatus_, &remaining_count);
      if (status != UWBSTATUS_SUCCESS) {
        return status;
      }

      // perform lock
      if (bindingStatus_ == UWB_DEVICE_BOUND_UNLOCKED && remaining_count < 3) {
        status = sr1xx_check_binding_status(&bindingStatus_);
        if (status != UWBSTATUS_SUCCESS) {
          return status;
        }
      }
    }
    break;
  case UWB_DEVICE_BOUND_UNLOCKED:
    {
      sr1xx_disable_dpd();

      // perform lock
      tHAL_UWB_STATUS status = sr1xx_check_binding_status(&bindingStatus_);
      if (status != UWBSTATUS_SUCCESS) {
        // Sending originial binding status notification to upper layer
        // XXX: Why?
        report_binding_status(bindingStatus_);
      }
    }
    break;

  case UWB_DEVICE_BOUND_LOCKED:
    // do nothing
    break;

  default:
    NXPLOG_UCIHAL_E("Unknown binding status: 0x%x", bindingStatus_);
    return UWBSTATUS_FAILED;
  }

  return UWBSTATUS_SUCCESS;
}

extern int phNxpUciHal_fw_download();

tHAL_UWB_STATUS NxpUwbChipSr1xx::chip_init()
{
  tHAL_UWB_STATUS status;

  // system in FW download mode
  // This will be cleared on first Device Status NTF
  nxpucihal_ctrl.fw_dwnld_mode = true;

  NXPLOG_UCIHAL_D("Start SR1XX FW download");

  for (int i = 0; i < 5; i++) {
    phTmlUwb_Chip_Reset();

    status = phNxpUciHal_fw_download();

    if (status == UWBSTATUS_SUCCESS) {
      NXPLOG_UCIHAL_D("Complete SR1XX FW download");
      break;
    } else if(status == UWBSTATUS_FILE_NOT_FOUND) {
      NXPLOG_UCIHAL_E("FW file Not found.");
      break;
    } else {
      NXPLOG_UCIHAL_E("FW download failed, status= 0x%x, retry.", status);
    }
  }

  // register device status ntf handler
  deviceStatusNtfHandler_ = UciHalRxHandler(
      UCI_MT_NTF, UCI_GID_CORE, UCI_MSG_CORE_DEVICE_STATUS_NTF, false,
      std::bind(&NxpUwbChipSr1xx::onDeviceStatusNtf, this, std::placeholders::_1, std::placeholders::_2)
  );

  // register device error ntf handler
  genericErrorNtfHandler_ = UciHalRxHandler(
    UCI_MT_NTF, UCI_GID_CORE, UCI_MSG_CORE_GENERIC_ERROR_NTF, false,
    std::bind(&NxpUwbChipSr1xx::onGenericErrorNtf, this, std::placeholders::_1, std::placeholders::_2)
  );

  // register binding status ntf handler
  bindingStatusNtfHandler_ = UciHalRxHandler(
      UCI_MT_NTF, UCI_GID_PROPRIETARY, UCI_MSG_BINDING_STATUS_NTF, true,
      std::bind(&NxpUwbChipSr1xx::onBindingStatusNtf, this, std::placeholders::_1, std::placeholders::_2)
  );

  return status;
}

tHAL_UWB_STATUS NxpUwbChipSr1xx::core_init()
{
  return check_binding();
}

device_type_t NxpUwbChipSr1xx::get_device_type(const uint8_t *param, size_t param_len)
{
  // 'SR100S' or 'SR1..T'
  if (param_len >= 6) {
    const uint8_t marker = param[5];
    if (marker == 'S')
      return DEVICE_TYPE_SR1xxS;
    else if (marker == 'T')
      return DEVICE_TYPE_SR1xxT;
  }
  return DEVICE_TYPE_UNKNOWN;
}

tHAL_UWB_STATUS NxpUwbChipSr1xx::read_otp(extcal_param_id_t id, uint8_t *data, size_t data_len, size_t *retlen)
{
  return sr1xx_read_otp(id, data, data_len, retlen);
}

tHAL_UWB_STATUS NxpUwbChipSr1xx::apply_calibration(extcal_param_id_t id, const uint8_t ch, const uint8_t *data, size_t data_len)
{
  return sr1xx_apply_calibration(id, ch, data, data_len);
}

int16_t NxpUwbChipSr1xx::extra_group_delay(void) {
  bool need_7cm_offset = FALSE;
  // + Compensation for D48/D49 calibration
  // If calibration was done with D48 / D49
  char calibrated_with_fw[15] = {0};

  int has_calibrated_with_fw_config = NxpConfig_GetStr(
    "cal.fw_version", calibrated_with_fw, sizeof(calibrated_with_fw) - 1);

  if ( has_calibrated_with_fw_config ) {
    // Conf file has entry of `cal.fw_version`
    if (
      ( 0 == memcmp("48.", calibrated_with_fw, 3)) ||
      ( 0 == memcmp("49.", calibrated_with_fw, 3))) {
      // Calibrated with D48 / D49.
      if (nxpucihal_ctrl.fw_version.major_version == 0xFF) {
        // Current FW seems to be Test FW
        NXPLOG_UCIHAL_W("For Test FW, D49 -> D50+ 7cm Compensation is applied");
        need_7cm_offset = TRUE;
      }
      else if (nxpucihal_ctrl.fw_version.major_version >= 0x50) {
        // D50 and later fix is needed.
        need_7cm_offset = TRUE;
      }
    }
    else
    {
      // Not calibrated with D48/D49
    }
  }
  else
  {
    // Missing Entry cal.fw_version
    NXPLOG_UCIHAL_W("Could not get cal.fw_version. Assuming D48 used for calibration.");
    need_7cm_offset = TRUE;
  }
  if (need_7cm_offset) {
    /* Its Q14.2 format, hence << 2 */
    return (7 << 2);
  }
  else
  {
    return 0;
  }
}

std::unique_ptr<NxpUwbChip> GetUwbChip()
{
  return std::make_unique<NxpUwbChipSr1xx>();
}

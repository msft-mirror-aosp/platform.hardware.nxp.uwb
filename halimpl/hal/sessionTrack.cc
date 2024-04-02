#include <bit>
#include <mutex>
#include <unordered_map>
#include <thread>
#include <vector>

#include "phNxpConfig.h"
#include "phNxpUciHal.h"
#include "phNxpUciHal_ext.h"
#include "phNxpUciHal_utils.h"

extern phNxpUciHal_Control_t nxpucihal_ctrl;

//
// SessionTrack
//
// Keeps track of device/session state.
//
// 1. Per-country calibrations
//
// When the country code is switching from A to B,
// a. Active Sessions restricted by B country should be stopped.
// b. Per-country device calibrations should be delayed to where
//    device stays in IDLE.
//
// 2. Issue URSK_DELETE_CMD on SESSION_DEINIT_RSP (optional/experimental)
//
// Calls URSK_DELETE_CMD for every CCC session closing,
// for the cases where CCC session ID was created but not started.
// (This is only activated when DELETE_URSK_FOR_CCC_SESSION=1 is set
// from config)
//
// 3. Call suspend to kernel driver on idle (optional/experimental)
//
// (This is only activated when AUTO_SUSPEND_ENABLED=1 is set from config)
// Tracks the each session's status and automatically requests suspend
// to kernel driver when it's idle for a given duration,
// and resumes the device before sending any commands.
// SessionTracks detects UWBS is in idle when there's no session created.
//

class SessionTrack {
private:
  // Session
  struct SessionInfo {
    uint32_t  session_id_;
    uint8_t   session_type_;
    uint8_t   session_state_;
    uint8_t   channel_;
    SessionInfo(uint32_t session_id, uint8_t session_type) :
      session_id_(session_id),
      session_type_(session_type),
      session_state_(UCI_MSG_SESSION_STATE_UNDEFINED),
      channel_(0) {
    }
  };
  enum class SessionTrackWorkType {
    IDLE = 0,
    REFRESH_IDLE,
    ACTIVATE,
    IDLE_TIMER_FIRED,
    DELETE_URSK,
    STOP,
  };
  enum class PowerState {
    SUSPEND = 0,
    IDLE,
    ACTIVE,
  };
  struct SessionTrackMsg {
    SessionTrackWorkType type_;
    std::shared_ptr<SessionInfo> session_info_;
    bool sync_;
    std::condition_variable cond_;

    SessionTrackMsg(SessionTrackWorkType type, bool sync) : type_(type), sync_(sync) { }

    // Per-session work item
    SessionTrackMsg(SessionTrackWorkType type, std::shared_ptr<SessionInfo> session_info, bool sync) :
      type_(type), session_info_(session_info), sync_(sync) { }
  };
  static constexpr unsigned long kAutoSuspendTimeoutDefaultMs_ = (30 * 1000);
  static constexpr long kQueueTimeoutMs = 500;

private:
  std::shared_ptr<phNxpUciHal_RxHandler> rx_handler_session_status_ntf_;
  std::unordered_map<uint32_t, std::shared_ptr<SessionInfo>> sessions_;
  std::mutex sessions_lock_;

  bool auto_suspend_enabled_;
  bool delete_ursk_ccc_enabled_;
  bool calibration_delayed_;
  std::atomic<PowerState> power_state_;
  bool idle_timer_started_;
  unsigned long idle_timeout_ms_;

  std::thread worker_thread_;
  std::mutex sync_mutex_;
  uint32_t idle_timer_;
  std::unique_ptr<MessageQueue<SessionTrackMsg>> msgq_;

public:
  SessionTrack() :
    auto_suspend_enabled_(false),
    delete_ursk_ccc_enabled_(false),
    calibration_delayed_(false),
    power_state_(PowerState::IDLE),
    idle_timer_started_(false),
    idle_timeout_ms_(kAutoSuspendTimeoutDefaultMs_)
  {
    sessions_.clear();

    msgq_ = std::make_unique<MessageQueue<SessionTrackMsg>>("SessionTrack");
    worker_thread_ = std::thread(&SessionTrack::PowerManagerWorker, this);

    unsigned long numval = 0;

    if (NxpConfig_GetNum(NAME_DELETE_URSK_FOR_CCC_SESSION, &numval, sizeof(numval)) && numval) {
      delete_ursk_ccc_enabled_ = true;
    }

    if (NxpConfig_GetNum(NAME_AUTO_SUSPEND_ENABLE, &numval, sizeof(numval)) && numval) {
      auto_suspend_enabled_ = true;

      NxpConfig_GetNum(NAME_AUTO_SUSPEND_TIMEOUT_MS, &idle_timeout_ms_, sizeof(idle_timeout_ms_));

      // Idle timer is only activated when AUTO_SUSPEND_ENABLED=1
      // device suspend won't be triggered when it's not activated.
      idle_timer_ = phOsalUwb_Timer_Create();
      RefreshIdle();
    }

    // register SESSION_STATUS_NTF rx handler
    rx_handler_session_status_ntf_ = phNxpUciHal_rx_handler_add(
      UCI_MT_NTF, UCI_GID_SESSION_MANAGE, UCI_MSG_SESSION_STATUS_NTF,
      false, false,
      std::bind(&SessionTrack::OnSessionStatusNtf, this, std::placeholders::_1, std::placeholders::_2));
  }

  virtual ~SessionTrack() {
    phNxpUciHal_rx_handler_del(rx_handler_session_status_ntf_);

    if (auto_suspend_enabled_) {
      phOsalUwb_Timer_Delete(idle_timer_);
    }
    auto msg = std::make_shared<SessionTrackMsg>(SessionTrackWorkType::STOP, true);
    QueueSessionTrackWork(msg);
    worker_thread_.join();
  }

  // Called upon upper-layer's SESSION_INIT_CMD
  void OnSessionInit(size_t packet_len, const uint8_t *packet)
  {
    if (packet_len != UCI_MSG_SESSION_STATE_INIT_CMD_LEN)
      return;

    uint32_t session_id = le_bytes_to_cpu<uint32_t>(&packet[UCI_MSG_SESSION_STATE_INIT_CMD_ID_OFFSET]);
    uint8_t session_type = packet[UCI_MSG_SESSION_STATE_INIT_CMD_TYPE_OFFSET];

    // Check SESSION_INIT_RSP for SessionID - Handle matching
    auto session_init_rsp_cb =
      [this, session_id, session_type](size_t packet_len, const uint8_t *packet)
    {
      if (packet_len != UCI_MSG_SESSION_STATE_INIT_RSP_LEN )
        return;

      uint8_t status = packet[UCI_MSG_SESSION_STATE_INIT_RSP_STATUS_OFFSET];
      uint32_t handle = le_bytes_to_cpu<uint32_t>(&packet[UCI_MSG_SESSION_STATE_INIT_RSP_HANDLE_OFFSET]);
      if (status != UWBSTATUS_SUCCESS)
        return;

      bool was_idle;
      {
        std::lock_guard<std::mutex> lock(sessions_lock_);

        was_idle = IsDeviceIdle();

        sessions_.emplace(std::make_pair(handle,
                                         std::make_shared<SessionInfo>(session_id, session_type)));
      }
      if (was_idle) {
        NXPLOG_UCIHAL_D("Queue Active");
        auto msg = std::make_shared<SessionTrackMsg>(SessionTrackWorkType::ACTIVATE, false);
        QueueSessionTrackWork(msg);
      }
    };

    // XXX: This rx handler can be called multiple times on
    // UCI_STATUS_COMMAND_RETRY(0xA) from SESSION_INIT_CMD
    phNxpUciHal_rx_handler_add(UCI_MT_RSP, UCI_GID_SESSION_MANAGE,
      UCI_MSG_SESSION_STATE_INIT, false, true, session_init_rsp_cb);
  }

  // Called by upper-layer's SetAppConfig command handler
  void OnChannelConfig(uint32_t session_handle, uint8_t channel) {
    // Update channel info
    std::lock_guard<std::mutex> lock(sessions_lock_);
    auto pSessionInfo = GetSessionInfo(session_handle);
    if (!pSessionInfo)
      return;
    pSessionInfo->channel_ = channel;
  }

  // Called by upper-layer's SetCountryCode command handler,
  // Check whether per-country calibration can be executed.
  // phNxpUciHal_Runtime_Settings_t should've been updated.
  void OnCountryCodeChanged() {
    phNxpUciHal_Runtime_Settings_t *rt_set = &nxpucihal_ctrl.rt_settings;
    NXPLOG_UCIHAL_D("SessionTrack: OnCountryCodeChanged");

    calibration_delayed_ = false;
    std::vector<uint32_t> blocked_session_handles;
    {
      std::lock_guard<std::mutex> lock(sessions_lock_);
      for (const auto elem : sessions_) {
        auto session_handle = elem.first;
        auto pSessionInfo = elem.second;

        if(pSessionInfo->session_state_ != UCI_MSG_SESSION_STATE_ACTIVE)
          continue;
        // there's active sessions existed, delay per-country calibrations
        calibration_delayed_ = true;
        if (!rt_set->uwb_enable || rt_set->restricted_channel_mask & (1 << pSessionInfo->channel_)) {
          blocked_session_handles.push_back(session_handle);
        }
      }
    }

    if (rt_set->uwb_enable && !calibration_delayed_) {
      NXPLOG_UCIHAL_D("SessionTrack: no active sessions, execute per-country cal now.")
      apply_per_country_calibrations();
    } else {
      NXPLOG_UCIHAL_D("SessionTrack: device is in active state, delay per-country cal.")
      // stop all sessions affected by new country code's restrictions
      for (auto session_handle : blocked_session_handles) {
        NXPLOG_UCIHAL_D("SessionTrack: stop session (handle=0x%08x) due to country code restrictions", session_handle);
        // Can issue an UCI command. This function is only called from upper-layer thread.
        StopRanging(session_handle);
      }
    }
  }

  void RefreshIdle() {
    auto msg = std::make_shared<SessionTrackMsg>(SessionTrackWorkType::REFRESH_IDLE, true);
    QueueSessionTrackWork(msg);
  }

private:
  // Send SESSION_STOP_CMD
  void StopRanging(uint32_t session_handle) {
    uint8_t session_handle_bytes[4];
    cpu_to_le_bytes(session_handle_bytes, session_handle);

    std::vector<uint8_t> packet{(UCI_MT_CMD << UCI_MT_SHIFT) | UCI_GID_SESSION_CONTROL, UCI_MSG_SESSION_STOP, 0, 0};
    packet.insert(packet.end(), std::begin(session_handle_bytes), std::end(session_handle_bytes));
    packet[UCI_PAYLOAD_LENGTH_OFFSET] = packet.size() - UCI_MSG_HDR_SIZE;

    auto ret = phNxpUciHal_send_ext_cmd(packet.size(), packet.data());
    if (ret != UWBSTATUS_SUCCESS) {
      NXPLOG_UCIHAL_E("SessionTrack: Failed to stop session handle 0x%08x", session_handle);
    }
  }

  // Send URSK_DELETE_CMD
  void DeleteUrsk(std::shared_ptr<SessionInfo> session_info) {
    if (!session_info)
      return;

    phNxpUciHal_rx_handler_add(UCI_MT_RSP, UCI_GID_PROPRIETARY_0X0F,
      UCI_MSG_URSK_DELETE, true, true,
      [](size_t packet_len, const uint8_t *packet) {
        if (packet_len < 5)
          return;
        if (packet[4] != UWBSTATUS_SUCCESS) {
          NXPLOG_UCIHAL_E("SessionTrack: URSR_DELETE failed, rsp status=0x%x", packet[4]);
        }
      }
    );
    phNxpUciHal_rx_handler_add(UCI_MT_NTF, UCI_GID_PROPRIETARY_0X0F,
      UCI_MSG_URSK_DELETE, true, true,
      [](size_t packet_len, const uint8_t *packet) {
        if (packet_len < 6)
          return;
        uint8_t status = packet[4];
        uint8_t nr = packet[5];
        if (packet_len != (6 + 5 * nr)) {
          NXPLOG_UCIHAL_E("SessionTrack: unrecognized packet type of URSK_DELETE_NTF");
        } else {
          for (auto i = 6; i < packet_len; i += 5) {
            uint32_t session_id = le_bytes_to_cpu<uint32_t>(&packet[i]);
            uint8_t del_status = packet[i + 4];
            if (status != UWBSTATUS_SUCCESS) {
              NXPLOG_UCIHAL_E("SessionTrack: URSK_DELETE failed, ntf status=0x%x", status);
            } else {
              NXPLOG_UCIHAL_D("SessionTrack: URSK_DELETE done");
            }
          }
        }
      }
    );

    NXPLOG_UCIHAL_D("SessionTrack: URSK_DELETE for session ID 0x%x", session_info->session_id_);
    uint8_t session_id_bytes[4];
    cpu_to_le_bytes(session_id_bytes, session_info->session_id_);

    std::vector<uint8_t> packet{(UCI_MT_CMD << UCI_MT_SHIFT) | UCI_GID_PROPRIETARY_0X0F,
      UCI_MSG_URSK_DELETE, 0, 0};

    packet.push_back(1);  // Num of Session IDs = 1
    packet.insert(packet.end(), std::begin(session_id_bytes), std::end(session_id_bytes));
    packet[UCI_PAYLOAD_LENGTH_OFFSET] = packet.size() - UCI_MSG_HDR_SIZE;

    auto ret = phNxpUciHal_send_ext_cmd(packet.size(), packet.data());
    if (ret != UWBSTATUS_SUCCESS) {
      NXPLOG_UCIHAL_E("SessionTrack: Failed to delete URSK for session id 0x%08x",
        session_info->session_id_);
    }
  }

  // UCI_MSG_SESSION_STATUS_NTF rx handler
  void OnSessionStatusNtf(size_t packet_len, const uint8_t* packet) {
    if (packet_len != UCI_MSG_SESSION_STATUS_NTF_LENGTH) {
      NXPLOG_UCIHAL_E("SessionTrack: SESSION_STATUS_NTF packet parse error");
      return;
    }

    uint32_t session_handle = le_bytes_to_cpu<uint32_t>(&packet[UCI_MSG_SESSION_STATUS_NTF_HANDLE_OFFSET]);
    uint8_t session_state = packet[UCI_MSG_SESSION_STATUS_NTF_STATE_OFFSET];

    bool is_idle = false;
    {
      std::lock_guard<std::mutex> lock(sessions_lock_);

      if (session_state == UCI_MSG_SESSION_STATE_DEINIT) {
        NXPLOG_UCIHAL_D("SessionTrack: remove session handle 0x%08x", session_handle);
        auto pSessionInfo = GetSessionInfo(session_handle);

        if (delete_ursk_ccc_enabled_ && pSessionInfo &&
            pSessionInfo->session_type_ == kSessionType_CCCRanging) {
          // If this CCC ranging session, issue DELETE_URSK_CMD for this session.
          auto msg = std::make_shared<SessionTrackMsg>(SessionTrackWorkType::DELETE_URSK, pSessionInfo, false);
          QueueSessionTrackWork(msg);
        }
        sessions_.erase(session_handle);
        is_idle = IsDeviceIdle();
      } else {
        NXPLOG_UCIHAL_D("SessionTrack: update session handle 0x%08x state %u", session_handle, session_state);
        auto pSessionInfo = GetSessionInfo(session_handle);
        if (pSessionInfo) {
          pSessionInfo->session_state_ = session_state;
        }
      }
    }

    if (is_idle) { // transition to IDLE
      NXPLOG_UCIHAL_D("Queue Idle");
      auto msg = std::make_shared<SessionTrackMsg>(SessionTrackWorkType::IDLE, false);
      QueueSessionTrackWork(msg);
    }
  }

  static void IdleTimerCallback(uint32_t TimerId, void* pContext) {
    SessionTrack *mgr = static_cast<SessionTrack*>(pContext);
    auto msg = std::make_shared<SessionTrackMsg>(SessionTrackWorkType::IDLE_TIMER_FIRED, false);
    mgr->QueueSessionTrackWork(msg);
  }

  void PowerIdleTimerStop() {
    if (!auto_suspend_enabled_)
      return;

    NXPLOG_UCIHAL_D("SessionTrack: stop idle timer");
    if (idle_timer_started_) {
      if (phOsalUwb_Timer_Stop(idle_timer_) != UWBSTATUS_SUCCESS) {
        NXPLOG_UCIHAL_E("SessionTrack: idle timer stop failed");
      }
      idle_timer_started_ = false;
    }
  }
  void PowerIdleTimerRefresh() {
    if (!auto_suspend_enabled_)
      return;

    NXPLOG_UCIHAL_D("SessionTrack: refresh idle timer, %lums", idle_timeout_ms_);
    if (idle_timer_started_) {
      if (phOsalUwb_Timer_Stop(idle_timer_) != UWBSTATUS_SUCCESS) {
        NXPLOG_UCIHAL_E("SessionTrack: idle timer stop failed");
      }
    }
    if (phOsalUwb_Timer_Start(idle_timer_, idle_timeout_ms_, IdleTimerCallback, this) != UWBSTATUS_SUCCESS) {
      NXPLOG_UCIHAL_E("SessionTrack: idle timer start failed");
    }
    idle_timer_started_ = true;
  }

  // Worker thread for auto suspend
  void PowerManagerWorker() {
    NXPLOG_UCIHAL_D("SessionTrack: worker thread started.")

    bool stop_thread = false;
    while (!stop_thread) {
      auto msg = msgq_->recv();
      if (!msg) {
        NXPLOG_UCIHAL_E("Power State: CRITICAL: worker thread received a bad message!, stop the queue");
        break;
      }
      NXPLOG_UCIHAL_D("SessionTrack: work %d state %d",
        static_cast<int>(msg->type_), static_cast<int>(power_state_.load()));

      switch (msg->type_) {
      case SessionTrackWorkType::IDLE:
        if (calibration_delayed_) {
          NXPLOG_UCIHAL_D("SessionTrack: No active session, execute per-country calibrations");
          CONCURRENCY_LOCK();
          apply_per_country_calibrations();
          CONCURRENCY_UNLOCK();
          calibration_delayed_ = false;
        }
        power_state_ = PowerState::IDLE;
        PowerIdleTimerRefresh();
        break;
      case SessionTrackWorkType::REFRESH_IDLE:
        if (power_state_ == PowerState::SUSPEND) {
          NXPLOG_UCIHAL_D("SessionTrack: resume");
          phTmlUwb_Resume();
          power_state_ = PowerState::IDLE;
        }
        if (power_state_ == PowerState::IDLE) {
          PowerIdleTimerRefresh();
        }
        break;
      case SessionTrackWorkType::ACTIVATE:
        if (power_state_ == PowerState::SUSPEND) {
          NXPLOG_UCIHAL_E("SessionTrack: activated while in suspend!");
          phTmlUwb_Resume();
        }
        PowerIdleTimerStop();
        power_state_ = PowerState::ACTIVE;
        break;
      case SessionTrackWorkType::IDLE_TIMER_FIRED:
        if (power_state_ == PowerState::IDLE) {
          NXPLOG_UCIHAL_D("SessionTrack: idle timer expired, go suspend");
          power_state_ = PowerState::SUSPEND;
          phTmlUwb_Suspend();
        } else {
          NXPLOG_UCIHAL_E("SessionTrack: idle timer expired while in %d",
            static_cast<int>(power_state_.load()));
        }
        break;
      case SessionTrackWorkType::DELETE_URSK:
        DeleteUrsk(msg->session_info_);
        break;
      case SessionTrackWorkType::STOP:
        stop_thread = true;
        break;
      default:
        NXPLOG_UCIHAL_E("SessionTrack: worker thread received a bad message!");
        break;
      }
      if (msg->sync_)
        msg->cond_.notify_one();
    }
    if (idle_timer_started_) {
      PowerIdleTimerStop();
    }

    NXPLOG_UCIHAL_D("SessionTrack: worker thread exit.");
  }

  void QueueSessionTrackWork(std::shared_ptr<SessionTrackMsg> msg) {
    msgq_->send(msg);

    if (msg->sync_) {
      std::unique_lock<std::mutex> lock(sync_mutex_);
      if (msg->cond_.wait_for(lock, std::chrono::milliseconds(kQueueTimeoutMs)) == std::cv_status::timeout) {
        NXPLOG_UCIHAL_E("SessionTrack: timeout to process %d", static_cast<int>(msg->type_));
      }
    }
  }

  std::shared_ptr<SessionInfo> GetSessionInfo(uint32_t session_handle) {
    auto it = sessions_.find(session_handle);
    if (it == sessions_.end()) {
      NXPLOG_UCIHAL_E("SessionTrack: Session 0x%08x not registered", session_handle);
      return NULL;
    }
    return it->second;
  }

  bool IsDeviceIdle() {
    return sessions_.size() == 0;
  }
};

static std::unique_ptr<SessionTrack> gSessionTrack;

void SessionTrack_init()
{
  gSessionTrack = std::make_unique<SessionTrack>();
}

void SessionTrack_deinit()
{
  gSessionTrack.reset();
}

void SessionTrack_onCountryCodeChanged()
{
  if (gSessionTrack)
    gSessionTrack->OnCountryCodeChanged();
}

void SessionTrack_onAppConfig(uint32_t session_handle, uint8_t channel)
{
  if (gSessionTrack)
    gSessionTrack->OnChannelConfig(session_handle, channel);
}

void SessionTrack_keepAlive()
{
  if (gSessionTrack)
    gSessionTrack->RefreshIdle();
}

void SessionTrack_onSessionInit(size_t packet_len, const uint8_t *packet)
{
  if (gSessionTrack)
    gSessionTrack->OnSessionInit(packet_len, packet);
}
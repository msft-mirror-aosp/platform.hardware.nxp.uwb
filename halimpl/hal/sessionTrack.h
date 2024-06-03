#ifndef _SESSIONTRACK_H_
#define _SESSIONTRACK_H_

#include <cstdint>

void SessionTrack_init();
void SessionTrack_deinit();
void SessionTrack_onCountryCodeChanged();
void SessionTrack_onAppConfig(uint32_t session_handle, uint8_t channel);
void SessionTrack_keepAlive();
void SessionTrack_onSessionInit(size_t packet_len, const uint8_t *packet);
void SessionTrack_onSessionStart(size_t packet_len, const uint8_t *packet);
#endif
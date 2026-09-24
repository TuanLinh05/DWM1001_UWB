/**
 ******************************************************************************
 * @file    telemetry.h
 * @brief   TAG telemetry — binary packets (default) or ASCII CSV (bring-up)
 *
 * Every packet goes through the non-blocking UART ring buffer.
 *
 * Frame (unchanged since v1, so the ESP32 gateway and the GUI keep parsing):
 *   SOF[2]=0xAA,0x55 | VER[1]=1 | TYPE[1] | LEN[2] | SEQ[4] | TIME[4] |
 *   PAYLOAD[LEN] | CRC16[2]
 *   CRC16-CCITT (0x1021, init 0xFFFF) over VER..end of payload, little-endian.
 *   LEN <= 256 (gateway limit).
 *
 * Types:
 *   0x00 INFO        (1 Hz)  schema 2 — profile, calibration mask/offsets
 *   0x01 RANGE       (cycle) snapshot of every anchor, 16-byte records
 *   0x02 STATS       (1 Hz)  global counters and rates
 *   0x10 RANGE_MEAS  (each measurement, optional) — see telemetry.c
 *   0x11 DIAG_ANCHOR (rotating, 2 anchors/s) — per-anchor health
 *   0x12 ANCHOR_INFO (when received) — anchor position/build TLVs
 *   0x13 CMD_ACK     reply to a host command (uwb_cmd.h)
 *   0x14 SNIFFER_FRAME (sniffer role only)
 *   0x15 DIAG_SYSTEM (1 Hz)  health, recovery, RX error causes, UART, env
 *   0x16 DEVICE_INFO (boot + every 10 s) — build, OTP, radio config
 *   0x20 CMD         host → TAG command (uwb_cmd.h)
 *
 * The payload layouts are documented next to each encoder in telemetry.c
 * and mirrored by Software/UWB_UART_GUI/telemetry_protocol.py.
 ******************************************************************************
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tag_ranging.h"
#include "telemetry_frame.h"
#include "uwb_app_config.h"

/** 1 = ASCII CSV (Serial Monitor bring-up); 0 = binary for gateway/GUI. */
#ifndef TELEM_ASCII
#define TELEM_ASCII   0
#endif

#define TELEM_INFO_SCHEMA    2U
#define TELEM_INFO_HEADER_LEN 10U

/* INFO flags stay within the existing one-byte payload. */
#define TELEM_INFO_FLAG_HW_ANTENNA_DELAY       0x01U
#define TELEM_INFO_FLAG_LEGACY_OFFSET          0x02U
#define TELEM_INFO_FLAG_DS_BUILD               0x04U
#define TELEM_INFO_FLAG_LEGACY_ADAPTIVE_SHIFT  3U
#define TELEM_INFO_FLAG_LEGACY_ADAPTIVE_MASK   0x18U
/** FPP/RX power use the corrected RXPACC (≈ +12 dB vs older firmware). */
#define TELEM_INFO_FLAG_FPP_CORRECTED          0x20U

/* Runtime telemetry features (SET_TELEMETRY command). */
#define TELEM_FEATURE_RANGE_SNAPSHOT  0x01U
#define TELEM_FEATURE_RANGE_MEAS      0x02U
#define TELEM_FEATURE_DIAG            0x04U

/** Features at boot unless settings override them. RANGE_MEAS needs a
 *  UART of at least TELEM_RANGE_MEAS_MIN_BAUD. */
#ifndef UWB_TELEM_DEFAULT_FEATURES
#define UWB_TELEM_DEFAULT_FEATURES (TELEM_FEATURE_RANGE_SNAPSHOT | TELEM_FEATURE_DIAG)
#endif
#define TELEM_RANGE_MEAS_MIN_BAUD  460800U

/** Configured UART baud (from the devicetree in a Zephyr build). */
uint32_t Telem_UartBaud(void);

/** Enable features; RANGE_MEAS is refused below TELEM_RANGE_MEAS_MIN_BAUD.
 *  @retval 0 on success, -1 when a requested feature is not supported. */
int Telem_SetFeatures(uint8_t features);
uint8_t Telem_GetFeatures(void);

/** Last die temperature / supply readings (raw SAR) for DIAG_SYSTEM. */
void Telem_SetEnvironment(uint8_t temp_raw, uint8_t vbat_raw);

/** Send the active ranging/calibration profile (periodic: clients attach late). */
void Telem_SendInfo(void);

/** One range snapshot (CSV line or binary packet). */
void Telem_SendRangeCycle(const TagCycleSnapshot_t *snap);

/** Global counters and measured rates (~1 Hz). */
void Telem_SendStats(uint16_t cyc_hz, uint16_t ops_hz);

/** Telemetry v2 messages (binary only; no-ops in ASCII mode). */
void Telem_SendRangeMeas(const TagMeasurement_t *m);
void Telem_SendDiagAnchor(uint8_t anchor_index);
void Telem_SendDiagSystem(void);
void Telem_SendDeviceInfo(void);
void Telem_SendAnchorInfo(const TagAnchorInfo_t *info);
void Telem_SendCmdAck(uint32_t cmd_seq, uint8_t cmd_id, uint8_t result,
                      const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */

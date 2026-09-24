/* Persistent settings: atomic snapshot, CRC, identity and reset failures. */
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "dw1000_hw.h"
#include "tag_ranging.h"
#include "telemetry.h"
#include "uwb_settings.h"
#include <zephyr/settings/settings.h>

#define CHECK(cond) do { if (!(cond)) { \
    fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
    return 1; } } while (0)

DW1000_RadioConfig_t dw1000_radio_config = {
    .tx_ant_dly = UWB_TX_ANT_DLY, .rx_ant_dly = UWB_RX_ANT_DLY,
    .tx_power_mode = UWB_TX_POWER_MODE,
    .reference_tuning = UWB_DW_REFERENCE_TUNING,
    .tx_power_custom = UWB_TX_POWER_CUSTOM_VALUE,
};
DW1000_OtpInfo_t dw1000_otp;

static int32_t s_bias[TAG_NUM_ANCHORS];
static uint8_t s_cal[TAG_NUM_ANCHORS];
static uint8_t s_active_mask = 0xFFU;
static uint8_t s_features = TELEM_FEATURE_RANGE_SNAPSHOT;
static uint8_t s_measurement_queue;
static uint32_t s_profile_id = 0x13572468UL;

uint32_t DW1000_CalibrationProfileId(void) { return s_profile_id; }

int Tag_SetDsCalibration(uint16_t anchor_id, int32_t bias_um, uint8_t calibrated)
{
    if (anchor_id == 0U || anchor_id > TAG_NUM_ANCHORS
        || bias_um > 10000000 || bias_um < -10000000)
        return -1;
    const uint8_t i = (uint8_t)(anchor_id - 1U);
    s_bias[i] = bias_um;
    s_cal[i] = calibrated ? 1U : 0U;
    return 0;
}

int Tag_GetDsCalibration(uint16_t anchor_id, int32_t *bias_um, uint8_t *calibrated)
{
    if (anchor_id == 0U || anchor_id > TAG_NUM_ANCHORS)
        return -1;
    const uint8_t i = (uint8_t)(anchor_id - 1U);
    if (bias_um != NULL) *bias_um = s_bias[i];
    if (calibrated != NULL) *calibrated = s_cal[i];
    return 0;
}

void Tag_InvalidateDsCalibration(void)
{
    memset(s_cal, 0, sizeof(s_cal));
}

void Tag_RestoreDefaultConfiguration(void)
{
    memset(s_bias, 0, sizeof(s_bias));
    memset(s_cal, 0, sizeof(s_cal));
    s_active_mask = 0xFFU;
}

void Tag_SetActiveAnchorMask(uint8_t mask) { s_active_mask = mask; }
uint8_t Tag_GetActiveAnchorMask(void) { return s_active_mask; }
void Tag_EnableMeasurementQueue(uint8_t enable) { s_measurement_queue = enable; }
int Telem_SetFeatures(uint8_t features) { s_features = features; return 0; }
uint8_t Telem_GetFeatures(void) { return s_features; }

static uint8_t s_stored[128];
static size_t s_stored_len;
static int s_init_result;
static int s_load_result;
static int s_save_result;
static const char *s_delete_fail_key;
static unsigned s_save_calls;
static unsigned s_delete_calls;

static ssize_t backend_read(void *cb_arg, void *data, size_t len)
{
    (void)cb_arg;
    if (len > s_stored_len) len = s_stored_len;
    memcpy(data, s_stored, len);
    return (ssize_t)len;
}

int settings_subsys_init(void) { return s_init_result; }

int settings_load_subtree(const char *subtree)
{
    CHECK(strcmp(subtree, "uwb") == 0);
    if (s_load_result != 0) return s_load_result;
    if (s_stored_len != 0U)
        return settings_test_handler("config", s_stored_len, backend_read, NULL);
    return 0;
}

int settings_save_one(const char *name, const void *value, size_t value_len)
{
    s_save_calls++;
    CHECK(strcmp(name, "uwb/config") == 0);
    CHECK(value_len <= sizeof(s_stored));
    if (s_save_result != 0) return s_save_result;
    memcpy(s_stored, value, value_len);
    s_stored_len = value_len;
    return 0;
}

int settings_delete(const char *name)
{
    s_delete_calls++;
    if (s_delete_fail_key != NULL && strcmp(name, s_delete_fail_key) == 0)
        return -EIO;
    if (strcmp(name, "uwb/config") == 0)
        s_stored_len = 0U;
    return 0;
}

static void backend_reset(void)
{
    s_stored_len = 0U;
    s_init_result = 0;
    s_load_result = 0;
    s_save_result = 0;
    s_delete_fail_key = NULL;
    s_save_calls = 0U;
    s_delete_calls = 0U;
    memset(&dw1000_otp, 0, sizeof(dw1000_otp));
    Tag_RestoreDefaultConfiguration();
    s_features = TELEM_FEATURE_RANGE_SNAPSHOT;
    s_measurement_queue = 0U;
}

static int test_round_trip_and_identity(void)
{
    backend_reset();
    CHECK(uwb_settings_load() == 0);
    CHECK(Tag_SetDsCalibration(1U, 123456, 1U) == 0);
    CHECK(uwb_settings_save() == -EACCES && s_save_calls == 0U);
    dw1000_otp.valid = 1U;
    dw1000_otp.part_id = 0xA1B2C3D4UL;
    dw1000_radio_config.tx_ant_dly = 16500U;
    Tag_SetActiveAnchorMask(0x0FU);
    CHECK(uwb_settings_save() == 0);
    CHECK(s_save_calls == 1U && s_stored_len == 80U); /* one atomic blob */

    dw1000_radio_config.tx_ant_dly = UWB_TX_ANT_DLY;
    Tag_RestoreDefaultConfiguration();
    CHECK(uwb_settings_load() == 0);
    CHECK(dw1000_radio_config.tx_ant_dly == 16500U);
    CHECK(Tag_GetActiveAnchorMask() == 0x0FU);
    uint8_t calibrated = 0U;
    CHECK(Tag_GetDsCalibration(1U, NULL, &calibrated) == 0 && calibrated == 0U);
    CHECK(uwb_settings_apply_calibration() == 0);
    CHECK(Tag_GetDsCalibration(1U, NULL, &calibrated) == 0 && calibrated == 1U);
    CHECK((uwb_settings_status & (UWB_SETTINGS_LOADED_RADIO
                                  | UWB_SETTINGS_LOADED_CAL
                                  | UWB_SETTINGS_LOADED_TAG)) == 0x07U);

    /* Same snapshot on another physical TAG must never enable calibration. */
    Tag_RestoreDefaultConfiguration();
    CHECK(uwb_settings_load() == 0);
    dw1000_otp.part_id ^= 1U;
    CHECK(uwb_settings_apply_calibration() == -EACCES);
    CHECK(Tag_GetDsCalibration(1U, NULL, &calibrated) == 0 && calibrated == 0U);
    CHECK((uwb_settings_status & UWB_SETTINGS_CAL_REJECTED) != 0U);
    return 0;
}

static int test_profile_and_crc_rejection(void)
{
    /* Recreate a valid snapshot. */
    backend_reset();
    CHECK(uwb_settings_load() == 0);
    dw1000_otp.valid = 1U;
    dw1000_otp.part_id = 0x11223344UL;
    CHECK(Tag_SetDsCalibration(2U, -7000, 1U) == 0);
    CHECK(uwb_settings_save() == 0);

    Tag_RestoreDefaultConfiguration();
    CHECK(uwb_settings_load() == 0);
    s_profile_id ^= 0x100U;
    CHECK(uwb_settings_apply_calibration() == -EACCES);
    CHECK(s_cal[1] == 0U);
    s_profile_id ^= 0x100U;

    s_stored[20] ^= 0x80U;             /* corrupt payload, leave CRC stale */
    CHECK(uwb_settings_load() == 0);
    CHECK((uwb_settings_status & UWB_SETTINGS_INVALID_BLOB) != 0U);
    CHECK((uwb_settings_status & UWB_SETTINGS_LOADED_RADIO) == 0U);
    return 0;
}

static int test_factory_reset_error_aggregation(void)
{
    backend_reset();
    CHECK(uwb_settings_load() == 0);
    dw1000_radio_config.tx_ant_dly = 16600U;
    s_delete_fail_key = "uwb/cal";
    CHECK(uwb_settings_factory_reset() == -EIO);
    CHECK(s_delete_calls == 3U);        /* all legacy deletes attempted */
    CHECK(dw1000_radio_config.tx_ant_dly == 16600U); /* RAM untouched */

    s_delete_fail_key = "uwb/config";
    s_delete_calls = 0U;
    CHECK(uwb_settings_factory_reset() == -EIO);
    CHECK(s_delete_calls == 4U);
    CHECK(dw1000_radio_config.tx_ant_dly == 16600U);

    s_delete_fail_key = NULL;
    s_delete_calls = 0U;
    CHECK(uwb_settings_factory_reset() == 0);
    CHECK(s_delete_calls == 4U);
    CHECK(dw1000_radio_config.tx_ant_dly == UWB_TX_ANT_DLY);
    CHECK(Tag_GetActiveAnchorMask() == 0xFFU);
    return 0;
}

int main(void)
{
    if (test_round_trip_and_identity()
        || test_profile_and_crc_rejection()
        || test_factory_reset_error_aggregation())
        return 1;
    puts("settings tests passed (atomic snapshot, CRC, PARTID/profile binding, reset errors)");
    return 0;
}

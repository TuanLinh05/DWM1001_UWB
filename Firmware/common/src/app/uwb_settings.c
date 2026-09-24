/**
 ******************************************************************************
 * @file    uwb_settings.c
 * @brief   Atomic, fail-closed persistent TAG configuration.
 *
 * Schema v2 stores radio, calibration and TAG settings in one CRC-protected
 * snapshot. Calibration is applied only after DW1000_Init() has read OTP and
 * only when both PARTID and the complete RF/PHY profile identity match.
 ******************************************************************************
 */

#include "uwb_settings.h"
#include "dw1000_hw.h"
#include "tag_ranging.h"
#include "telemetry.h"

#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/settings/settings.h>

#define UWB_SETTINGS_MAGIC            0x32425755UL  /* "UWB2", little-endian */
#define UWB_SETTINGS_SCHEMA           2U
#define UWB_SETTINGS_KEY              "uwb/config"
#define UWB_SETTINGS_MAX_GENERATION   0xFFFFFFFFUL

typedef struct __attribute__((packed)) {
    int32_t bias_um;
    uint8_t calibrated;
} CalEntryV2_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t schema;
    uint8_t reserved0;
    uint16_t size;
    uint32_t generation;

    uint8_t tx_power_mode;
    uint8_t reference_tuning;
    uint16_t tx_ant_dly;
    uint16_t rx_ant_dly;
    uint16_t reserved1;
    uint32_t tx_power_custom;

    uint32_t calibration_profile_id;
    uint32_t tag_part_id;
    CalEntryV2_t anchor[TAG_NUM_ANCHORS];

    uint8_t active_mask;
    uint8_t telem_features;
    uint16_t reserved2;
    uint32_t crc32;
} SettingsBlobV2_t;

_Static_assert(sizeof(SettingsBlobV2_t) == 80U,
               "settings v2 layout changed: bump schema and migration policy");

static const DW1000_RadioConfig_t k_radio_default = {
    .tx_ant_dly = UWB_TX_ANT_DLY,
    .rx_ant_dly = UWB_RX_ANT_DLY,
    .tx_power_mode = UWB_TX_POWER_MODE,
    .reference_tuning = UWB_DW_REFERENCE_TUNING,
    .tx_power_custom = UWB_TX_POWER_CUSTOM_VALUE,
};

uint8_t uwb_settings_status;

static SettingsBlobV2_t s_pending;
static uint8_t s_pending_valid;
static uint8_t s_calibration_pending;
static uint32_t s_generation;

static uint32_t crc32_ieee(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;

    for (size_t i = 0U; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
            crc = (crc & 1U) ? (crc >> 1) ^ 0xEDB88320UL : crc >> 1;
    }
    return ~crc;
}

static int read_blob(size_t len, settings_read_cb read_cb, void *cb_arg,
                     SettingsBlobV2_t *blob)
{
    if (len != sizeof(*blob))
        return -EINVAL;
    const ssize_t got = read_cb(cb_arg, blob, sizeof(*blob));
    return (got == (ssize_t)sizeof(*blob)) ? 0 : -EIO;
}

static int validate_blob(const SettingsBlobV2_t *blob)
{
    if (blob->magic != UWB_SETTINGS_MAGIC
        || blob->schema != UWB_SETTINGS_SCHEMA
        || blob->size != sizeof(*blob)
        || blob->reserved0 != 0U || blob->reserved1 != 0U || blob->reserved2 != 0U)
        return -EINVAL;
    if (blob->tx_power_mode > UWB_TX_POWER_CUSTOM
        || blob->reference_tuning > 1U
        || blob->tx_ant_dly == 0U || blob->rx_ant_dly == 0U
        || (blob->tx_power_mode == UWB_TX_POWER_CUSTOM && blob->tx_power_custom == 0U))
        return -EINVAL;

    const uint32_t expected = crc32_ieee((const uint8_t *)blob,
                                         offsetof(SettingsBlobV2_t, crc32));
    return blob->crc32 == expected ? 0 : -EBADMSG;
}

static void restore_runtime_defaults(void)
{
    dw1000_radio_config = k_radio_default;
    Tag_RestoreDefaultConfiguration();
    (void)Telem_SetFeatures((uint8_t)UWB_TELEM_DEFAULT_FEATURES);
    Tag_EnableMeasurementQueue((Telem_GetFeatures() & TELEM_FEATURE_RANGE_MEAS) != 0U);
}

static void apply_radio(const SettingsBlobV2_t *blob)
{
    dw1000_radio_config.tx_power_mode = blob->tx_power_mode;
    dw1000_radio_config.reference_tuning = blob->reference_tuning;
    dw1000_radio_config.tx_ant_dly = blob->tx_ant_dly;
    dw1000_radio_config.rx_ant_dly = blob->rx_ant_dly;
    dw1000_radio_config.tx_power_custom = blob->tx_power_custom;
    uwb_settings_status |= UWB_SETTINGS_LOADED_RADIO;
}

static void apply_tag(const SettingsBlobV2_t *blob)
{
    Tag_SetActiveAnchorMask(blob->active_mask);
    /* A stored feature the compiled UART cannot carry is dropped safely. */
    if (Telem_SetFeatures(blob->telem_features) != 0)
        (void)Telem_SetFeatures((uint8_t)(blob->telem_features
                                         & ~TELEM_FEATURE_RANGE_MEAS));
    Tag_EnableMeasurementQueue((Telem_GetFeatures() & TELEM_FEATURE_RANGE_MEAS) != 0U);
    uwb_settings_status |= UWB_SETTINGS_LOADED_TAG;
}

static int apply_calibration(const SettingsBlobV2_t *blob)
{
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        if (Tag_SetDsCalibration((uint16_t)(i + 1U), blob->anchor[i].bias_um,
                                 blob->anchor[i].calibrated) != 0)
        {
            Tag_InvalidateDsCalibration();
            return -EINVAL;
        }
    }
    uwb_settings_status |= UWB_SETTINGS_LOADED_CAL;
    return 0;
}

static int uwb_settings_set(const char *key, size_t len,
                            settings_read_cb read_cb, void *cb_arg)
{
    /* Legacy schema-1 keys are deliberately ignored: they did not bind
     * calibration to a device or RF profile and are therefore unsafe. */
    if (strcmp(key, "radio") == 0 || strcmp(key, "cal") == 0
        || strcmp(key, "tag") == 0)
    {
        uwb_settings_status |= UWB_SETTINGS_LEGACY_IGNORED;
        return 0;
    }
    if (strcmp(key, "config") != 0)
        return 0;                       /* forward-compatible unknown key */

    SettingsBlobV2_t blob;
    if (read_blob(len, read_cb, cb_arg, &blob) != 0 || validate_blob(&blob) != 0)
    {
        uwb_settings_status |= UWB_SETTINGS_INVALID_BLOB;
        return 0;                       /* fail closed, keep defaults */
    }

    s_pending = blob;
    s_pending_valid = 1U;
    s_calibration_pending = 1U;
    s_generation = blob.generation;
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(uwb, "uwb", NULL, uwb_settings_set, NULL, NULL);

int uwb_settings_load(void)
{
    uwb_settings_status = 0U;
    s_pending_valid = 0U;
    s_calibration_pending = 0U;
    s_generation = 0U;
    memset(&s_pending, 0, sizeof(s_pending));
    restore_runtime_defaults();

    int rc = settings_subsys_init();
    if (rc != 0)
    {
        uwb_settings_status = UWB_SETTINGS_INIT_FAILED;
        return rc;
    }

    rc = settings_load_subtree("uwb");
    if (rc != 0)
    {
        uwb_settings_status |= UWB_SETTINGS_LOAD_FAILED;
        return rc;
    }
    if (s_pending_valid != 0U)
    {
        apply_radio(&s_pending);         /* needed before DW1000_Configure() */
        apply_tag(&s_pending);
    }
    return 0;
}

int uwb_settings_apply_calibration(void)
{
    if (s_pending_valid == 0U || s_calibration_pending == 0U)
        return 0;
    s_calibration_pending = 0U;          /* exactly once per boot/load */

    uint8_t any_calibrated = 0U;
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
        any_calibrated |= s_pending.anchor[i].calibrated ? 1U : 0U;

    if (any_calibrated != 0U
        && (dw1000_otp.valid == 0U || dw1000_otp.part_id == 0U
            || s_pending.tag_part_id != dw1000_otp.part_id
            || s_pending.calibration_profile_id != DW1000_CalibrationProfileId()))
    {
        Tag_InvalidateDsCalibration();
        uwb_settings_status |= UWB_SETTINGS_CAL_REJECTED;
        return -EACCES;
    }

    const int rc = apply_calibration(&s_pending);
    if (rc != 0)
        uwb_settings_status |= UWB_SETTINGS_CAL_REJECTED;
    return rc;
}

int uwb_settings_save(void)
{
    if ((uwb_settings_status & UWB_SETTINGS_INIT_FAILED) != 0U)
        return -ENODEV;

    uint8_t any_calibrated = 0U;
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        uint8_t calibrated = 0U;
        (void)Tag_GetDsCalibration((uint16_t)(i + 1U), NULL, &calibrated);
        any_calibrated |= calibrated ? 1U : 0U;
    }
    if (any_calibrated != 0U
        && (dw1000_otp.valid == 0U || dw1000_otp.part_id == 0U))
        return -EACCES;                 /* never persist unbound calibration */

    SettingsBlobV2_t blob;
    memset(&blob, 0, sizeof(blob));
    blob.magic = UWB_SETTINGS_MAGIC;
    blob.schema = UWB_SETTINGS_SCHEMA;
    blob.size = sizeof(blob);
    blob.generation = (s_generation == UWB_SETTINGS_MAX_GENERATION)
        ? 1U : s_generation + 1U;
    blob.tx_power_mode = dw1000_radio_config.tx_power_mode;
    blob.reference_tuning = dw1000_radio_config.reference_tuning;
    blob.tx_ant_dly = dw1000_radio_config.tx_ant_dly;
    blob.rx_ant_dly = dw1000_radio_config.rx_ant_dly;
    blob.tx_power_custom = dw1000_radio_config.tx_power_custom;
    blob.calibration_profile_id = DW1000_CalibrationProfileId();
    blob.tag_part_id = (dw1000_otp.valid != 0U) ? dw1000_otp.part_id : 0U;

    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        int32_t bias_um = 0;
        uint8_t calibrated = 0U;
        (void)Tag_GetDsCalibration((uint16_t)(i + 1U), &bias_um, &calibrated);
        blob.anchor[i].bias_um = bias_um;
        blob.anchor[i].calibrated = calibrated;
    }
    blob.active_mask = Tag_GetActiveAnchorMask();
    blob.telem_features = Telem_GetFeatures();
    blob.crc32 = crc32_ieee((const uint8_t *)&blob,
                            offsetof(SettingsBlobV2_t, crc32));

    const int rc = settings_save_one(UWB_SETTINGS_KEY, &blob, sizeof(blob));
    if (rc == 0)
    {
        s_generation = blob.generation;
        s_pending = blob;
        s_pending_valid = 1U;
        s_calibration_pending = 0U;       /* runtime table is already applied */
    }
    return rc;
}

static void collect_delete_error(int *first_error, const char *key)
{
    const int rc = settings_delete(key);
    if (rc != 0 && rc != -ENOENT && *first_error == 0)
        *first_error = rc;
}

int uwb_settings_factory_reset(void)
{
    if ((uwb_settings_status & UWB_SETTINGS_INIT_FAILED) != 0U)
        return -ENODEV;

    int rc = 0;
    /* Legacy keys are non-authoritative in v2. Remove them first; keep the
     * current v2 snapshot intact if storage fails during that cleanup. */
    collect_delete_error(&rc, "uwb/radio");
    collect_delete_error(&rc, "uwb/cal");
    collect_delete_error(&rc, "uwb/tag");
    if (rc != 0)
        return rc;                       /* leave RAM untouched on failure */
    collect_delete_error(&rc, UWB_SETTINGS_KEY);
    if (rc != 0)
        return rc;

    restore_runtime_defaults();
    memset(&s_pending, 0, sizeof(s_pending));
    s_pending_valid = 0U;
    s_calibration_pending = 0U;
    s_generation = 0U;
    uwb_settings_status = 0U;
    return 0;
}

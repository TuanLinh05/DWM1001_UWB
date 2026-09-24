/**
 ******************************************************************************
 * @file    dw1000.c
 * @brief   DW1000 register-level driver shared by every DWM1001 node.
 *
 * PHY (verified against the registers written below): Channel 5, PRF 16 MHz,
 * preamble 256, PAC 16, preamble code 4 (TX/RX), 6.8 Mbps, standard SFD.
 *
 * Every register access is a single SPI transaction (header + data). The
 * radio parameters that may change at runtime (antenna delays, TX power,
 * reference tuning) live in dw1000_radio_config; their defaults reproduce
 * the compile-time configuration exactly.
 ******************************************************************************
 */

#include "dw1000_hw.h"
#include "uwb_platform.h"
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define CS_LOW()  uwb_platform_cs_set(true)
#define CS_HIGH() uwb_platform_cs_set(false)

#define LED_OFF() uwb_platform_led_set(false)

volatile uint8_t dw1000_spi_mhz = DW_SPI_INIT_MHZ;
volatile uint32_t dw1000_spi_error_count;

DW1000_RadioConfig_t dw1000_radio_config = {
    .tx_ant_dly = UWB_TX_ANT_DLY,
    .rx_ant_dly = UWB_RX_ANT_DLY,
    .tx_power_mode = UWB_TX_POWER_MODE,
    .reference_tuning = UWB_DW_REFERENCE_TUNING,
    .tx_power_custom = UWB_TX_POWER_CUSTOM_VALUE,
};

DW1000_OtpInfo_t dw1000_otp = {
    .applied_xtal_trim = 0xFFU,
};

/* SYS_CFG bits written by DW1000_Configure(). */
#define DW_SYS_CFG_HIRQ_POL  (1UL << 9)
#define DW_SYS_CFG_DIS_DRXB  (1UL << 12)
#define DW_SYS_CFG_DIS_STXP  (1UL << 18)

/* Reference values from the DW1000 User Manual (TX_POWER tables, Ch5 PRF16)
 * and the baseline value this project shipped with. */
#define DW_TX_POWER_LEGACY     0x1E1E1E1EUL
#define DW_TX_POWER_REFERENCE  0x48484848UL
#define DW_TX_POWER_SMART      0x0E082848UL

#define DW_LDE_CFG1_NTM13      0x6DU   /* NTM = 13, PMULT = 3 (Decawave LDE_PARAM1) */
#define DW_FS_XTALT_MIDRANGE   0x10U
#define DW_FS_XTALT_RESERVED   0x60U   /* bits 7:5 must be written as 011 */

#define DW_SYS_MASK_VALUE      0x2437D080UL   /* bytes 80 D0 37 24, see DW1000_Configure() */

/* ========================================================================== */
/*                     SPI ACCESS (one transaction per register access)        */
/* ========================================================================== */

static void spi_access(const uint8_t *header, uint8_t header_length,
                       const uint8_t *tx, uint8_t *rx, uint16_t length)
{
    CS_LOW();
    if (uwb_platform_spi_xfer(header, header_length, tx, rx, length) != 0) {
        dw1000_spi_error_count++;
        if (rx != NULL)
            memset(rx, 0, length);
    }
    CS_HIGH();
}

/* Header: bit7 = write, bit6 = sub-index present, bits5:0 = register file. */
static uint8_t build_header(uint8_t header[3], uint8_t is_write, uint8_t reg_id,
                            uint16_t sub_addr, uint8_t has_sub)
{
    header[0] = (uint8_t)((is_write ? 0x80U : 0x00U) | (reg_id & 0x3FU));
    if (!has_sub)
        return 1U;

    header[0] |= 0x40U;
    if (sub_addr > 0x7FU)
    {
        header[1] = (uint8_t)(0x80U | (sub_addr & 0x7FU)); /* extended address */
        header[2] = (uint8_t)((sub_addr >> 7) & 0xFFU);
        return 3U;
    }
    header[1] = (uint8_t)(sub_addr & 0x7FU);
    return 2U;
}

static void DW1000_ReadReg(uint8_t reg_id, uint8_t *data, uint16_t len)
{
    uint8_t header[3];
    spi_access(header, build_header(header, 0U, reg_id, 0U, 0U), NULL, data, len);
}

static void DW1000_WriteReg(uint8_t reg_id, const uint8_t *data, uint16_t len)
{
    uint8_t header[3];
    spi_access(header, build_header(header, 1U, reg_id, 0U, 0U), data, NULL, len);
}

static void DW1000_ReadSubReg(uint8_t reg_id, uint16_t sub_addr, uint8_t *data, uint16_t len)
{
    uint8_t header[3];
    spi_access(header, build_header(header, 0U, reg_id, sub_addr, 1U), NULL, data, len);
}

static void DW1000_WriteSubReg(uint8_t reg_id, uint16_t sub_addr, const uint8_t *data, uint16_t len)
{
    uint8_t header[3];
    spi_access(header, build_header(header, 1U, reg_id, sub_addr, 1U), data, NULL, len);
}

static uint32_t read_le32(const uint8_t *buf)
{
    return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
         | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}

static void write_le32(uint8_t *buf, uint32_t value)
{
    buf[0] = (uint8_t)value;
    buf[1] = (uint8_t)(value >> 8);
    buf[2] = (uint8_t)(value >> 16);
    buf[3] = (uint8_t)(value >> 24);
}

static uint32_t read_reg32(uint8_t reg_id)
{
    uint8_t buf[4];
    DW1000_ReadReg(reg_id, buf, 4);
    return read_le32(buf);
}

static uint32_t read_sub32(uint8_t reg_id, uint16_t sub_addr, uint16_t len)
{
    uint8_t buf[4] = {0};
    DW1000_ReadSubReg(reg_id, sub_addr, buf, len);
    return read_le32(buf);
}

static void write_sub8(uint8_t reg_id, uint16_t sub_addr, uint8_t value)
{
    DW1000_WriteSubReg(reg_id, sub_addr, &value, 1);
}

/* ========================================================================== */
/*                     SPI RATE                                                */
/* ========================================================================== */

static uint8_t SPI_ApplyRuntimeRate(uint8_t rate_mhz)
{
    CS_HIGH();
    if (uwb_platform_spi_set_frequency((uint32_t)rate_mhz * 1000000U) != 0)
        return 0U;

    dw1000_spi_mhz = rate_mhz;
    return (DW1000_ReadDeviceID() == DW1000_DEVICE_ID) ? 1U : 0U;
}

uint8_t DW1000_EnableFastSPI(void)
{
    /* The nRF52832 SPI instance is limited to 8 MHz. Initialization stays at
     * 2 MHz; runtime switches to 8 MHz after LDE loading. */
    if (SPI_ApplyRuntimeRate(DW_SPI_FAST_MHZ))
        return DW_SPI_FAST_MHZ;
    if (SPI_ApplyRuntimeRate(DW_SPI_INIT_MHZ))
        return DW_SPI_INIT_MHZ;

    dw1000_spi_mhz = 0U;
    return 0U;
}

/* ========================================================================== */
/*                     DEVICE ID / RESET                                       */
/* ========================================================================== */

uint32_t DW1000_ReadDeviceID(void)
{
    return read_reg32(DW_REG_DEV_ID);
}

static int DW1000_GPIO_Init(void)
{
    int error = uwb_platform_init();

    if (error == 0) {
        CS_HIGH();
        LED_OFF();
        error = uwb_platform_spi_set_frequency(DW_SPI_INIT_MHZ * 1000000U);
        if (error == 0)
            dw1000_spi_mhz = DW_SPI_INIT_MHZ;
    }
    return error;
}

/* ========================================================================== */
/*                     OTP                                                     */
/* ========================================================================== */

/* DW1000 UM 6.3.3: address, OTPRDEN|OTPREAD, clear, then read OTP_RDAT. */
static uint32_t dw_otp_read(uint16_t address)
{
    uint8_t addr[2] = { (uint8_t)(address & 0xFFU), (uint8_t)((address >> 8) & 0x07U) };

    DW1000_WriteSubReg(DW_REG_OTP_IF, DW_SUB_OTP_ADDR, addr, 2);
    write_sub8(DW_REG_OTP_IF, DW_SUB_OTP_CTRL, 0x03U);
    write_sub8(DW_REG_OTP_IF, DW_SUB_OTP_CTRL, 0x00U);
    return read_sub32(DW_REG_OTP_IF, DW_SUB_OTP_RDAT, 4);
}

static void dw_read_otp_info(void)
{
    const uint32_t xtrim_word = dw_otp_read(DW_OTP_XTRIM_ADDR);

    dw1000_otp.xtal_trim = (uint8_t)(xtrim_word & 0x1FU);
    dw1000_otp.otp_rev = (uint8_t)((xtrim_word >> 8) & 0xFFU);
    dw1000_otp.ldotune = dw_otp_read(DW_OTP_LDOTUNE_ADDR);
    dw1000_otp.part_id = dw_otp_read(DW_OTP_PARTID_ADDR);
    dw1000_otp.lot_id = dw_otp_read(DW_OTP_LOTID_ADDR);
    dw1000_otp.vbat_cal = (uint8_t)(dw_otp_read(DW_OTP_VBAT_ADDR) & 0xFFU);
    dw1000_otp.vtemp_cal = (uint8_t)(dw_otp_read(DW_OTP_VTEMP_ADDR) & 0xFFU);
    dw1000_otp.valid = 1U;
}

/* ========================================================================== */
/*                     INITIALIZATION                                          */
/* ========================================================================== */

/**
 * @brief Full DW1000 initialization sequence.
 *
 *   1. Platform GPIO/SPI at the safe rate
 *   2. Hardware reset
 *   3. Verify Device ID
 *   4. Force the system clock to XTI and read OTP (Decawave dwt_initialise
 *      does the same so OTP values are reliable)
 *   5. Optional reference tuning: LDOTUNE kick and crystal trim
 *   6. Load LDE microcode (UM 2.5.5.10) and return clocks to auto
 */
int DW1000_Init(void)
{
    if (DW1000_GPIO_Init() != 0)
        return -2;

    uwb_platform_reset_radio();

    if (DW1000_ReadDeviceID() != DW1000_DEVICE_ID)
        return -1;

    /* Force SYSCLKS = XTI while reading OTP. */
    uint8_t pmsc0 = 0U;
    DW1000_ReadSubReg(DW_REG_PMSC, DW_SUB_PMSC_CTRL0, &pmsc0, 1);
    write_sub8(DW_REG_PMSC, DW_SUB_PMSC_CTRL0, (uint8_t)((pmsc0 & 0xFCU) | 0x01U));

    memset(&dw1000_otp, 0, sizeof(dw1000_otp));
    dw1000_otp.applied_xtal_trim = 0xFFU;
    dw_read_otp_info();

    if (dw1000_radio_config.reference_tuning)
    {
        if ((dw1000_otp.ldotune & 0xFFU) != 0U)
        {
            write_sub8(DW_REG_OTP_IF, DW_SUB_OTP_SF, 0x02U);   /* LDO_KICK */
            dw1000_otp.ldo_kicked = 1U;
        }
        const uint8_t trim = dw1000_otp.xtal_trim != 0U
            ? dw1000_otp.xtal_trim : DW_FS_XTALT_MIDRANGE;
        write_sub8(DW_REG_FS_CTRL, DW_SUB_FS_XTALT,
                   (uint8_t)(DW_FS_XTALT_RESERVED | (trim & 0x1FU)));
        dw1000_otp.applied_xtal_trim = trim;
    }

    /* LDE microcode load: FORCE_LDE clocks (0x0301), LDELOAD, wait >= 150 us,
     * then ENABLE_ALL_SEQ (0x0200) which also returns SYSCLKS to auto. */
    const uint8_t pmsc_lde[2] = { 0x01, 0x03 };
    DW1000_WriteSubReg(DW_REG_PMSC, DW_SUB_PMSC_CTRL0, pmsc_lde, 2);

    const uint8_t otp_ctrl_load[2] = { 0x00, 0x80 };
    DW1000_WriteSubReg(DW_REG_OTP_IF, DW_SUB_OTP_CTRL, otp_ctrl_load, 2);

    uwb_platform_delay_ms(5);

    const uint8_t pmsc_restore[2] = { 0x00, 0x02 };
    DW1000_WriteSubReg(DW_REG_PMSC, DW_SUB_PMSC_CTRL0, pmsc_restore, 2);

    /* Enable the clock-PLL lock detector exactly as the Decawave reference
     * initialisation does. CPLOCK is a latched event, not a level bit. */
    write_sub8(DW_REG_EXT_SYNC, DW_SUB_EC_CTRL, DW_EC_CTRL_PLLLCK);

    uwb_platform_delay_ms(2);
    return 0;
}

/* ========================================================================== */
/*                     CONFIGURATION                                           */
/* ========================================================================== */

uint32_t DW1000_TxPowerRegisterValue(void)
{
    switch (dw1000_radio_config.tx_power_mode)
    {
        case UWB_TX_POWER_REFERENCE: return DW_TX_POWER_REFERENCE;
        case UWB_TX_POWER_SMART:     return DW_TX_POWER_SMART;
        case UWB_TX_POWER_CUSTOM:    return dw1000_radio_config.tx_power_custom;
        case UWB_TX_POWER_LEGACY:
        default:                     return DW_TX_POWER_LEGACY;
    }
}

uint32_t DW1000_ExpectedSysCfg(void)
{
    uint32_t sys_cfg = DW_SYS_CFG_HIRQ_POL | DW_SYS_CFG_DIS_DRXB;

    if (dw1000_radio_config.tx_power_mode != UWB_TX_POWER_SMART)
        sys_cfg |= DW_SYS_CFG_DIS_STXP;
    return sys_cfg;
}

static uint16_t active_rx_antenna_delay(void);

static void profile_add_u32(uint32_t *hash, uint32_t value)
{
    for (uint8_t i = 0U; i < 4U; i++)
    {
        *hash ^= (uint8_t)(value >> (8U * i));
        *hash *= 16777619UL;       /* FNV-1a, canonical little-endian input */
    }
}

uint32_t DW1000_CalibrationProfileId(void)
{
    uint32_t hash = 2166136261UL;

    /* Bump the first field whenever the ranging/calibration interpretation
     * changes even if the register profile remains identical. */
    profile_add_u32(&hash, 1U);
    profile_add_u32(&hash, DW_PHY_PROFILE_ID);
    profile_add_u32(&hash, UWB_USE_DS_TWR);
    profile_add_u32(&hash, UWB_USE_HW_ANTENNA_DELAY);
    profile_add_u32(&hash, UWB_USE_LEGACY_OFFSET);
    profile_add_u32(&hash, dw1000_radio_config.tx_power_mode);
    profile_add_u32(&hash, DW1000_TxPowerRegisterValue());
    profile_add_u32(&hash, DW1000_ActiveTxAntennaDelay());
    profile_add_u32(&hash, active_rx_antenna_delay());
    profile_add_u32(&hash, dw1000_radio_config.reference_tuning);
    profile_add_u32(&hash, dw1000_radio_config.reference_tuning
                             ? dw1000_otp.applied_xtal_trim : 0xFFU);
    profile_add_u32(&hash, dw1000_radio_config.reference_tuning
                             ? dw1000_otp.ldo_kicked : 0U);
    return hash;
}

uint16_t DW1000_ActiveTxAntennaDelay(void)
{
#if UWB_USE_HW_ANTENNA_DELAY
    return dw1000_radio_config.tx_ant_dly;
#else
    return 0U;
#endif
}

static uint16_t active_rx_antenna_delay(void)
{
#if UWB_USE_HW_ANTENNA_DELAY
    return dw1000_radio_config.rx_ant_dly;
#else
    return 0U;
#endif
}

void DW1000_ApplyAntennaDelay(void)
{
    const uint16_t tx = DW1000_ActiveTxAntennaDelay();
    const uint16_t rx = active_rx_antenna_delay();
    uint8_t buf[2];

    buf[0] = (uint8_t)(tx & 0xFFU);
    buf[1] = (uint8_t)(tx >> 8);
    DW1000_WriteReg(DW_REG_TX_ANTD, buf, 2);

    buf[0] = (uint8_t)(rx & 0xFFU);
    buf[1] = (uint8_t)(rx >> 8);
    DW1000_WriteSubReg(DW_REG_LDE_IF, DW_SUB_LDE_RXANTD, buf, 2);
}

/**
 * @brief Configure the DW1000 for ranging (values from the DW1000 UM).
 */
int DW1000_Configure(void)
{
    uint8_t buf[5];

    /* SYS_CFG: HIRQ_POL, DIS_DRXB, and DIS_STXP unless smart TX power. */
    write_le32(buf, DW1000_ExpectedSysCfg());
    DW1000_WriteReg(DW_REG_SYS_CFG, buf, 4);

    /* AGC_TUNE1: 0x8870 for PRF 16 MHz */
    buf[0] = 0x70; buf[1] = 0x88;
    DW1000_WriteSubReg(DW_REG_AGC_CTRL, DW_SUB_AGC_TUNE1, buf, 2);

    /* AGC_TUNE2: 0x2502A907 */
    buf[0] = 0x07; buf[1] = 0xA9; buf[2] = 0x02; buf[3] = 0x25;
    DW1000_WriteSubReg(DW_REG_AGC_CTRL, DW_SUB_AGC_TUNE2, buf, 4);

    /* AGC_TUNE3: 0x0035 */
    buf[0] = 0x35; buf[1] = 0x00;
    DW1000_WriteSubReg(DW_REG_AGC_CTRL, DW_SUB_AGC_TUNE3, buf, 2);

    /* DRX_TUNE0b: 0x0001 for 6.8 Mbps, standard SFD */
    buf[0] = 0x01; buf[1] = 0x00;
    DW1000_WriteSubReg(DW_REG_DRX_CONF, DW_SUB_DRX_TUNE0b, buf, 2);

    /* DRX_TUNE1a: 0x0087 for PRF 16 MHz */
    buf[0] = 0x87; buf[1] = 0x00;
    DW1000_WriteSubReg(DW_REG_DRX_CONF, DW_SUB_DRX_TUNE1a, buf, 2);

    /* DRX_TUNE1b: 0x0020 for preamble > 64 at 6.8 Mbps */
    buf[0] = 0x20; buf[1] = 0x00;
    DW1000_WriteSubReg(DW_REG_DRX_CONF, DW_SUB_DRX_TUNE1b, buf, 2);

    /* DRX_TUNE2: 0x331A0052 for PRF16, PAC16 */
    buf[0] = 0x52; buf[1] = 0x00; buf[2] = 0x1A; buf[3] = 0x33;
    DW1000_WriteSubReg(DW_REG_DRX_CONF, DW_SUB_DRX_TUNE2, buf, 4);

    /* DRX_SFDTOC: preamble 256 + 1 + SFD 8 - PAC 16 */
    buf[0] = (uint8_t)(DW_PHY_SFD_TIMEOUT & 0xFFU);
    buf[1] = (uint8_t)((DW_PHY_SFD_TIMEOUT >> 8) & 0xFFU);
    DW1000_WriteSubReg(DW_REG_DRX_CONF, DW_SUB_DRX_SFDTOC, buf, 2);

    /* DRX_TUNE4H: 0x0028 for preamble >= 128 */
    buf[0] = 0x28; buf[1] = 0x00;
    DW1000_WriteSubReg(DW_REG_DRX_CONF, DW_SUB_DRX_TUNE4H, buf, 2);

    /* RF_RXCTRLH: 0xD8 for channel 5 */
    write_sub8(DW_REG_RF_CONF, DW_SUB_RF_RXCTRLH, 0xD8U);

    /* RF_TXCTRL: 0x001E3FE0 for channel 5 */
    buf[0] = 0xE0; buf[1] = 0x3F; buf[2] = 0x1E; buf[3] = 0x00;
    DW1000_WriteSubReg(DW_REG_RF_CONF, DW_SUB_RF_TXCTRL, buf, 4);

    /* TC_PGDELAY: 0xC0 for channel 5 */
    write_sub8(DW_REG_TX_CAL, DW_SUB_TC_PGDELAY, 0xC0U);

    /* FS_PLLCFG: 0x0800041D for channel 5 */
    buf[0] = 0x1D; buf[1] = 0x04; buf[2] = 0x00; buf[3] = 0x08;
    DW1000_WriteSubReg(DW_REG_FS_CTRL, DW_SUB_FS_PLLCFG, buf, 4);

    /* FS_PLLTUNE: 0xBE for channel 5 */
    write_sub8(DW_REG_FS_CTRL, DW_SUB_FS_PLLTUNE, 0xBEU);

    /* LDE_CFG1 NTM = 13 is the Decawave recommendation; the baseline kept the
     * reset value, so it is applied only with reference tuning. */
    if (dw1000_radio_config.reference_tuning)
        write_sub8(DW_REG_LDE_IF, DW_SUB_LDE_CFG1, DW_LDE_CFG1_NTM13);

    /* LDE_CFG2: 0x1607 for PRF 16 MHz */
    buf[0] = 0x07; buf[1] = 0x16;
    DW1000_WriteSubReg(DW_REG_LDE_IF, DW_SUB_LDE_CFG2, buf, 2);

    /* CHAN_CTRL: 0x21040055 — TX=5, RX=5, RXPRF=16 MHz, PCODE=4 */
    buf[0] = 0x55; buf[1] = 0x00; buf[2] = 0x04; buf[3] = 0x21;
    DW1000_WriteReg(DW_REG_CHAN_CTRL, buf, 4);

    /* Antenna delays (F-01): runtime values, 0 in the legacy-offset profile. */
    DW1000_ApplyAntennaDelay();

    /* LDE_REPC: 0x428E for preamble code 4 */
    buf[0] = 0x8E; buf[1] = 0x42;
    DW1000_WriteSubReg(DW_REG_LDE_IF, DW_SUB_LDE_REPC, buf, 2);

    /* TX_POWER for the selected profile (see uwb_calibration.h). */
    write_le32(buf, DW1000_TxPowerRegisterValue());
    DW1000_WriteReg(DW_REG_TX_POWER, buf, 4);

    /* W4R_TIM = 0: with WAIT4RESP the receiver turns on right after TX. */
    memset(buf, 0, 4);
    DW1000_WriteReg(DW_REG_ACK_RESP_T, buf, 4);

    /* CPLOCK is W1C and edge-latched. Do not clear an already valid event and
     * then require the same PLL values to generate a second edge: real DW1000
     * parts are allowed to stay locked, so that sequence can false-fail every
     * radio initialisation. Accept the lock event latched since reset, or wait
     * for one if the synthesiser is still settling. */
    uwb_platform_delay_ms(5);
    const uint32_t t0 = uwb_platform_time_ms();
    uint8_t pll_locked = 0U;
    while ((uwb_platform_time_ms() - t0) < 100U)
    {
        if ((read_reg32(DW_REG_SYS_STATUS) & DW_CPLOCK_BIT) != 0U)
        {
            pll_locked = 1U;
            break;
        }
        uwb_platform_delay_ms(1);
    }
    if (pll_locked == 0U)
        return -1;

    /* SYS_MASK: TX done, RX good and every recoverable RX fault, so faults
     * reach the state machines through the IRQ instead of a timeout.
     *   byte0 0x80: MTXFRS
     *   byte1 0xD0: MRXPHE | MRXFCG | MRXFCE
     *   byte2 0x37: MRXRFSL | MRXRFTO | MLDEERR | MRXOVRR | MRXPTO
     *   byte3 0x24: MRXSFDTO | MAFFREJ */
    buf[0] = 0x80;
    buf[1] = 0xD0;
    buf[2] = 0x37;
    buf[3] = 0x24;
    DW1000_WriteReg(DW_REG_SYS_MASK, buf, 4);
    return 0;
}

/* ========================================================================== */
/*                     ADDRESS CONFIGURATION                                   */
/* ========================================================================== */

void DW1000_SetAddress(uint16_t pan_id, uint16_t short_addr)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)(short_addr & 0xFFU);
    buf[1] = (uint8_t)((short_addr >> 8) & 0xFFU);
    buf[2] = (uint8_t)(pan_id & 0xFFU);
    buf[3] = (uint8_t)((pan_id >> 8) & 0xFFU);
    DW1000_WriteReg(DW_REG_PANADR, buf, 4);
}

/* ========================================================================== */
/*                     TX OPERATIONS                                           */
/* ========================================================================== */

void DW1000_WriteTxData(const uint8_t *data, uint16_t len)
{
    DW1000_WriteReg(DW_REG_TX_BUFFER, data, len);
}

/**
 * TX_FCTRL (0x08, 5 bytes): length incl. FCS in bits [9:0]; TXBR = 6.8 Mbps,
 * TR = ranging, TXPRF = 16 MHz, TXPSR/PE = preamble 256 (DW_TX_FCTRL_UPPER).
 */
void DW1000_SetTxFrameCtrl(uint16_t len)
{
    const uint32_t fctrl = (uint32_t)len | DW_TX_FCTRL_UPPER;
    uint8_t buf[5];

    write_le32(buf, fctrl);
    buf[4] = 0x00;  /* IFSDELAY */
    DW1000_WriteReg(DW_REG_TX_FCTRL, buf, 5);
}

static void write_sys_ctrl(uint8_t value)
{
    DW1000_WriteReg(DW_REG_SYS_CTRL, &value, 1);
}

void DW1000_StartTx(void)
{
    write_sys_ctrl((uint8_t)DW_TXSTRT_BIT);
}

void DW1000_StartTxWait4Resp(void)
{
    write_sys_ctrl((uint8_t)(DW_TXSTRT_BIT | DW_WAIT4RESP_BIT));
}

void DW1000_SetDelayedTxTime(const uint8_t tx_time[5])
{
    DW1000_WriteReg(DW_REG_DX_TIME, tx_time, 5);
}

int DW1000_StartTxDelayedEx(uint8_t wait4resp)
{
    uint8_t ctrl = (uint8_t)(DW_TXSTRT_BIT | DW_TXDLYS_BIT);

    if (wait4resp)
        ctrl |= (uint8_t)DW_WAIT4RESP_BIT;
    write_sys_ctrl(ctrl);

    /* HPDWARN: the scheduled time has already passed. The DW1000 would send
     * the frame only after the 40-bit counter wraps (~17 s), so the caller
     * must force the transceiver off at once (F-09). */
    const uint64_t status = DW1000_ReadStatus();
    if ((status & (DW_HPDWARN_BIT | DW_TXPUTE_BIT)) != 0U)
        return -1;
    return 0;
}

int DW1000_StartTxDelayed(void)
{
    return DW1000_StartTxDelayedEx(0U);
}

int DW1000_WaitTxDone(uint32_t timeout_ms)
{
    const uint32_t start = uwb_platform_time_ms();

    while ((uwb_platform_time_ms() - start) < timeout_ms)
    {
        if ((DW1000_ReadStatus() & DW_TXFRS_BIT) != 0U)
        {
            DW1000_ClearTxStatus();
            return 1;
        }
    }
    return 0;
}

/* ========================================================================== */
/*                     RX OPERATIONS                                           */
/* ========================================================================== */

void DW1000_StartRx(void)
{
    const uint8_t buf[2] = { 0x00, 0x01 };  /* bit 8 = RXENAB */
    DW1000_WriteReg(DW_REG_SYS_CTRL, buf, 2);
}

int DW1000_WaitRxDone(uint32_t timeout_ms)
{
    const uint32_t start = uwb_platform_time_ms();

    while ((uwb_platform_time_ms() - start) < timeout_ms)
    {
        const DW1000_RxEvent_t event = DW1000_ClassifyRx(DW1000_ReadStatus());
        if (event == DW_RX_EVENT_GOOD)
            return 1;
        if (event == DW_RX_EVENT_ERROR)
            return 2;
    }
    return 0;
}

uint16_t DW1000_ReadRxData(uint8_t *data, uint16_t max_len)
{
    uint8_t finfo[4];
    DW1000_ReadReg(DW_REG_RX_FINFO, finfo, 4);

    /* RXFLEN, bits [6:0]; the length includes the 2-byte FCS. */
    const uint16_t frame_len = finfo[0] & 0x7FU;

    /* A truncated frame must never be accepted as a complete ranging frame. */
    if (data == NULL || frame_len > max_len)
        return 0U;

    if (frame_len > 0U)
        DW1000_ReadReg(DW_REG_RX_BUFFER, data, frame_len);

    return frame_len;
}

void DW1000_ReadRxTimestamp(uint8_t ts[5])
{
    DW1000_ReadReg(DW_REG_RX_TIME, ts, 5);
}

void DW1000_ReadTxTimestamp(uint8_t ts[5])
{
    DW1000_ReadReg(DW_REG_TX_TIME, ts, 5);
}

void DW1000_ReadSysTime(uint8_t ts[5])
{
    DW1000_ReadReg(DW_REG_SYS_TIME, ts, 5);
}

/* ========================================================================== */
/*                     DIAGNOSTICS                                             */
/* ========================================================================== */

void DW1000_ReadSignalDiag(DW1000_SignalDiag_t *diag)
{
    uint8_t buf[8];

    /* RX_FINFO: RXPACC is bits [31:20] (12 bits), as in Decawave's
     * RX_FINFO_RXPACC_MASK. The baseline extracted bits [27:18], i.e.
     * about 4 x RXPACC plus RXPSR, which reported FPP ~12 dB too low. */
    DW1000_ReadReg(DW_REG_RX_FINFO, buf, 4);
    diag->rxpacc = (uint16_t)((read_le32(buf) >> 20) & 0x0FFFU);

    /* RX_FQUAL: STD_NOISE [1:0], FP_AMPL2 [3:2], FP_AMPL3 [5:4], CIR_PWR [7:6]. */
    DW1000_ReadReg(DW_REG_RX_FQUAL, buf, 8);
    diag->std_noise = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    diag->fp_ampl2  = (uint16_t)(buf[2] | ((uint16_t)buf[3] << 8));
    diag->fp_ampl3  = (uint16_t)(buf[4] | ((uint16_t)buf[5] << 8));
    diag->cir_pwr   = (uint16_t)(buf[6] | ((uint16_t)buf[7] << 8));

    /* RX_TIME: FP_INDEX at offset 5, FP_AMPL1 at offset 7. */
    DW1000_ReadSubReg(DW_REG_RX_TIME, 5, buf, 4);
    diag->fp_index = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    diag->fp_ampl1 = (uint16_t)(buf[2] | ((uint16_t)buf[3] << 8));
}

float DW1000_GetFirstPathPower(const DW1000_SignalDiag_t *diag)
{
    if (diag->rxpacc == 0U)
        return -120.0f;

    const float f1 = (float)diag->fp_ampl1;
    const float f2 = (float)diag->fp_ampl2;
    const float f3 = (float)diag->fp_ampl3;
    const float n = (float)diag->rxpacc;
    const float num = f1 * f1 + f2 * f2 + f3 * f3;

    if (num <= 0.0f)
        return -120.0f;
    /* UM 4.7.1: FP power = 10 log10((F1²+F2²+F3²)/N²) − A, A = 113.77 (PRF16). */
    return 10.0f * log10f(num / (n * n)) - UWB_FPP_A_CONST;
}

float DW1000_GetRxPower(const DW1000_SignalDiag_t *diag)
{
    if (diag->rxpacc == 0U || diag->cir_pwr == 0U)
        return -120.0f;

    const float n = (float)diag->rxpacc;
    /* UM 4.7.2: RX level = 10 log10(C × 2^17 / N²) − A. */
    return 10.0f * log10f(((float)diag->cir_pwr * 131072.0f) / (n * n))
         - UWB_FPP_A_CONST;
}

int32_t DW1000_ReadCarrierIntegrator(void)
{
    uint8_t buf[3] = {0};
    DW1000_ReadSubReg(DW_REG_DRX_CONF, DW_SUB_DRX_CAR_INT, buf, 3);

    uint32_t raw = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
                 | ((uint32_t)buf[2] << 16);

    /* 21-bit signed value: sign-extend bit 20. */
    raw &= 0x001FFFFFUL;
    if ((raw & (1UL << 20)) != 0U)
        raw |= 0xFFE00000UL;
    return (int32_t)raw;
}

int32_t DW1000_CarrierIntegratorToPpmX100(int32_t carrier_integrator)
{
    /* UWB_CLOCK_OFFSET_MULT is the ratio per CI unit; ×1e6 → ppm, ×100. */
    const double ppm_x100 = (double)carrier_integrator * UWB_CLOCK_OFFSET_MULT * 1.0e8;

    if (ppm_x100 > (double)INT32_MAX)
        return INT32_MAX;
    if (ppm_x100 < (double)INT32_MIN)
        return INT32_MIN;
    return (int32_t)lround(ppm_x100);
}

/* ========================================================================== */
/*                     STATUS / CONTROL                                        */
/* ========================================================================== */

void DW1000_ClearAllStatus(void)
{
    const uint8_t buf[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    DW1000_WriteReg(DW_REG_SYS_STATUS, buf, 5);
}

void DW1000_ClearTxStatus(void)
{
    /* TXFRB|TXPRS|TXPHS|TXFRS, HPDWARN|TXBERR, TXPUTE — nothing else. */
    const uint8_t buf[5] = { 0xF0, 0x00, 0x00, 0x18, 0x04 };
    DW1000_WriteReg(DW_REG_SYS_STATUS, buf, 5);
}

void DW1000_ForceRxOff(void)
{
    write_sys_ctrl((uint8_t)DW_TRXOFF_BIT);
    uwb_platform_delay_us(DW_TRXOFF_SETTLE_US);
}

void DW1000_RxSoftReset(void)
{
    /* dwt_rxreset(): SOFTRESET RX (0xE0) then clear (0xF0). */
    write_sub8(DW_REG_PMSC, DW_SUB_PMSC_SOFTRESET, 0xE0U);
    write_sub8(DW_REG_PMSC, DW_SUB_PMSC_SOFTRESET, 0xF0U);
}

uint64_t DW1000_ReadStatus(void)
{
    uint8_t buf[5];
    DW1000_ReadReg(DW_REG_SYS_STATUS, buf, 5);

    return (uint64_t)read_le32(buf) | ((uint64_t)buf[4] << 32);
}

/* ========================================================================== */
/*                     INTERRUPT SUPPORT                                       */
/* ========================================================================== */

/** IRQ flag — set by the GPIO callback, cleared by the state machines. */
volatile uint8_t dw1000_irq_flag = 0;

void DW1000_EnableIRQ(void)
{
    uwb_platform_enable_irq();
}

/* ========================================================================== */
/*                     µs TIMER, IRQ LEVEL                                     */
/* ========================================================================== */

void MCU_TimerInit(void)
{
    /* Zephyr's timing counter is started by uwb_platform_init(). */
}

uint32_t MCU_Micros(void)
{
    return uwb_platform_elapsed_us(0U);
}

McuCycleStamp_t MCU_CycleNow(void)
{
    return uwb_platform_cycle_now();
}

uint32_t MCU_ElapsedUs(McuCycleStamp_t start)
{
    /* Subtract raw cycles before converting (FIX-02, wrap-safe). */
    return uwb_platform_elapsed_us(start);
}

uint8_t DW1000_IrqLineActive(void)
{
    return uwb_platform_irq_active() ? 1U : 0U;
}

/* ========================================================================== */
/*                     CONFIG VERIFY                                           */
/* ========================================================================== */

#define DW_EXP_CHAN_CTRL    0x21040055UL   /* Ch5, RXPRF16, PCODE4 */
#define DW_EXP_DRX_TUNE2    0x331A0052UL

uint32_t DW1000_VerifyConfig(void)
{
    uint32_t mismatch = 0U;

    if (DW1000_ReadDeviceID() != DW1000_DEVICE_ID)
        mismatch |= DW_VERIFY_DEVID;
    if (read_reg32(DW_REG_SYS_CFG) != DW1000_ExpectedSysCfg())
        mismatch |= DW_VERIFY_SYS_CFG;
    if (read_reg32(DW_REG_CHAN_CTRL) != DW_EXP_CHAN_CTRL)
        mismatch |= DW_VERIFY_CHAN_CTRL;
    if (read_sub32(DW_REG_DRX_CONF, DW_SUB_DRX_SFDTOC, 2) != (uint32_t)DW_PHY_SFD_TIMEOUT)
        mismatch |= DW_VERIFY_SFDTOC;
    if (read_sub32(DW_REG_DRX_CONF, DW_SUB_DRX_TUNE2, 4) != DW_EXP_DRX_TUNE2)
        mismatch |= DW_VERIFY_DRX_TUNE2;
    if (read_reg32(DW_REG_TX_POWER) != DW1000_TxPowerRegisterValue())
        mismatch |= DW_VERIFY_TX_POWER;

    uint8_t buf[2];
    DW1000_ReadReg(DW_REG_TX_ANTD, buf, 2);
    if ((uint16_t)(buf[0] | ((uint16_t)buf[1] << 8)) != DW1000_ActiveTxAntennaDelay())
        mismatch |= DW_VERIFY_TX_ANTD;
    DW1000_ReadSubReg(DW_REG_LDE_IF, DW_SUB_LDE_RXANTD, buf, 2);
    if ((uint16_t)(buf[0] | ((uint16_t)buf[1] << 8)) != active_rx_antenna_delay())
        mismatch |= DW_VERIFY_RX_ANTD;

    if (dw1000_radio_config.reference_tuning)
    {
        if (read_sub32(DW_REG_LDE_IF, DW_SUB_LDE_CFG1, 1) != DW_LDE_CFG1_NTM13)
            mismatch |= DW_VERIFY_LDE_CFG1;
        if (dw1000_otp.applied_xtal_trim != 0xFFU
            && (read_sub32(DW_REG_FS_CTRL, DW_SUB_FS_XTALT, 1) & 0x1FU)
               != dw1000_otp.applied_xtal_trim)
            mismatch |= DW_VERIFY_XTALT;
    }

    if (read_reg32(DW_REG_SYS_MASK) != DW_SYS_MASK_VALUE)
        mismatch |= DW_VERIFY_SYS_MASK;

    return mismatch;
}

/* ========================================================================== */
/*                     TEMPERATURE / VOLTAGE (SAR)                             */
/* ========================================================================== */

#define DW_SUB_RF_TLD_BIAS   0x11
#define DW_SUB_RF_ADC_BIAS   0x12
#define DW_SUB_TC_SARC       0x00   /* SAR control */
#define DW_SUB_TC_SARL       0x03   /* SAR latest readings: [0]=vbat, [1]=temp */

void DW1000_ReadTempVbatRaw(uint8_t *temp_raw, uint8_t *vbat_raw)
{
    /* Sequence of Decawave dwt_readtempvbat(). */
    write_sub8(DW_REG_RF_CONF, DW_SUB_RF_TLD_BIAS, 0x80U);
    write_sub8(DW_REG_RF_CONF, DW_SUB_RF_ADC_BIAS, 0x0AU);
    write_sub8(DW_REG_RF_CONF, DW_SUB_RF_ADC_BIAS, 0x0FU);
    write_sub8(DW_REG_TX_CAL, DW_SUB_TC_SARC, 0x00U);
    write_sub8(DW_REG_TX_CAL, DW_SUB_TC_SARC, 0x01U);
    uwb_platform_delay_ms(1);

    uint8_t sar[2] = {0};
    DW1000_ReadSubReg(DW_REG_TX_CAL, DW_SUB_TC_SARL, sar, 2);
    write_sub8(DW_REG_TX_CAL, DW_SUB_TC_SARC, 0x00U);

    if (vbat_raw != NULL)
        *vbat_raw = sar[0];
    if (temp_raw != NULL)
        *temp_raw = sar[1];
}

uint8_t DW1000_ReadTemperatureRaw(void)
{
    uint8_t temp = 0U;
    DW1000_ReadTempVbatRaw(&temp, NULL);
    return temp;
}

int16_t DW1000_TemperatureCentiDeg(uint8_t temp_raw)
{
    if (!dw1000_otp.valid || dw1000_otp.vtemp_cal == 0U)
        return INT16_MIN;
    /* Decawave: T = (raw − OTP_23C) × 1.14 + 23 °C. */
    const int32_t centi = ((int32_t)temp_raw - (int32_t)dw1000_otp.vtemp_cal) * 114 + 2300;
    return (int16_t)centi;
}

uint16_t DW1000_VoltageMilliVolt(uint8_t vbat_raw)
{
    if (!dw1000_otp.valid || dw1000_otp.vbat_cal == 0U)
        return 0U;
    /* Decawave: V = (raw − OTP_3V3) / 173 + 3.3 V. */
    const int32_t mv = (((int32_t)vbat_raw - (int32_t)dw1000_otp.vbat_cal) * 1000) / 173 + 3300;
    return (uint16_t)(mv < 0 ? 0 : mv);
}

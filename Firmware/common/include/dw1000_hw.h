/**
 ******************************************************************************
 * @file    dw1000_hw.h
 * @brief   DW1000 register-level driver shared by every DWM1001 node.
 *
 * The Zephyr port obtains SPI, CS, IRQ and reset pins from the DWM1001 board
 * devicetree. No MCU pin is hard-coded in this driver.
 ******************************************************************************
 */

#ifndef DW1000_HW_H
#define DW1000_HW_H

#include <stdint.h>
#include "uwb_app_config.h"
#include "uwb_calibration.h"

/* ========================================================================== */
/*                     DW1000 REGISTER MAP                                     */
/* ========================================================================== */

#define DW_REG_DEV_ID       0x00    /* Device Identifier (4 bytes, RO) */
#define DW_REG_EUI          0x01    /* Extended Unique Identifier (8 bytes) */
#define DW_REG_PANADR       0x03    /* PAN ID and Short Address (4 bytes) */
#define DW_REG_SYS_CFG      0x04    /* System Configuration (4 bytes) */
#define DW_REG_SYS_TIME     0x06    /* System Time Counter (5 bytes, RO) */
#define DW_REG_TX_FCTRL     0x08    /* TX Frame Control (5 bytes) */
#define DW_REG_TX_BUFFER    0x09    /* TX Data Buffer (up to 1024 bytes) */
#define DW_REG_DX_TIME      0x0A    /* Delayed Send or Receive Time (5 bytes) */
#define DW_REG_SYS_CTRL     0x0D    /* System Control Register (4 bytes) */
#define DW_REG_SYS_MASK     0x0E    /* System Event Mask Register (4 bytes) */
#define DW_REG_SYS_STATUS   0x0F    /* System Event Status Register (5 bytes) */
#define DW_REG_RX_FINFO     0x10    /* RX Frame Information (4 bytes, RO) */
#define DW_REG_RX_BUFFER    0x11    /* RX Data Buffer (up to 1024 bytes, RO) */
#define DW_REG_RX_FQUAL     0x12    /* Receive Quality Data (8 bytes, RO) */
#define DW_REG_RX_TIME      0x15    /* RX Message Time of Arrival (14 bytes, RO) */
#define DW_REG_TX_TIME      0x17    /* TX Message Time of Sending (10 bytes, RO) */
#define DW_REG_TX_ANTD      0x18    /* TX Antenna Delay (2 bytes) */
#define DW_REG_ACK_RESP_T   0x1A    /* Wait-for-response time / ACK time (4 bytes) */
#define DW_REG_TX_POWER     0x1E    /* TX Power Control (4 bytes) */
#define DW_REG_CHAN_CTRL    0x1F    /* Channel Control (4 bytes) */
#define DW_REG_AGC_CTRL     0x23    /* AGC configuration and control block */
#define DW_REG_EXT_SYNC     0x24    /* External synchronisation control */
#define DW_REG_DRX_CONF     0x27    /* Digital Receiver configuration block */
#define DW_REG_RF_CONF      0x28    /* Analog RF configuration block */
#define DW_REG_TX_CAL       0x2A    /* Transmitter Calibration block */
#define DW_REG_FS_CTRL      0x2B    /* Frequency Synthesiser control block */
#define DW_REG_OTP_IF       0x2D    /* One-Time Programmable Memory IF */
#define DW_REG_LDE_IF       0x2E    /* Leading Edge Detection IF */
#define DW_REG_PMSC         0x36    /* Power Management and System Control */

/* ========================================================================== */
/*                     SUB-REGISTER OFFSETS                                    */
/* ========================================================================== */

/* AGC_CTRL sub-registers */
#define DW_SUB_AGC_TUNE1    0x04    /* AGC Tuning register 1 (2 bytes) */
#define DW_SUB_AGC_TUNE2    0x0C    /* AGC Tuning register 2 (4 bytes) */
#define DW_SUB_AGC_TUNE3    0x12    /* AGC Tuning register 3 (2 bytes) */

/* DRX_CONF sub-registers */
#define DW_SUB_DRX_TUNE0b   0x02   /* Digital Tuning 0b (2 bytes) */
#define DW_SUB_DRX_TUNE1a   0x04   /* Digital Tuning 1a (2 bytes) */
#define DW_SUB_DRX_TUNE1b   0x06   /* Digital Tuning 1b (2 bytes) */
#define DW_SUB_DRX_TUNE2    0x08   /* Digital Tuning 2 (4 bytes) */
#define DW_SUB_DRX_SFDTOC   0x20   /* SFD detection timeout (2 bytes) */
#define DW_SUB_DRX_TUNE4H   0x26   /* Digital Tuning 4H (2 bytes) */
#define DW_SUB_DRX_CAR_INT  0x28   /* Carrier Integrator (3 bytes, RO, 21-bit signed) */

/* RF_CONF sub-registers */
#define DW_SUB_RF_RXCTRLH   0x0B   /* RF RX Control (1 byte) */
#define DW_SUB_RF_TXCTRL    0x0C   /* RF TX Control (4 bytes) */

/* TX_CAL sub-registers */
#define DW_SUB_TC_PGDELAY   0x0B   /* Pulse Generator Delay (1 byte) */

/* FS_CTRL sub-registers */
#define DW_SUB_FS_PLLCFG    0x07   /* PLL Configuration (4 bytes) */
#define DW_SUB_FS_PLLTUNE   0x0B   /* PLL Tuning (1 byte) */
#define DW_SUB_FS_XTALT     0x0E   /* Crystal trim (1 byte) */

/* LDE_IF sub-registers (extended addressing) */
#define DW_SUB_LDE_CFG1     0x0806 /* LDE Configuration 1 (1 byte): NTM | PMULT */
#define DW_SUB_LDE_RXANTD   0x1804 /* RX antenna delay (2 bytes) */
#define DW_SUB_LDE_CFG2     0x1806 /* LDE Configuration 2 (2 bytes) */
#define DW_SUB_LDE_REPC     0x2804 /* LDE Replica Coefficient (2 bytes) */

/* OTP_IF sub-registers */
#define DW_SUB_OTP_ADDR     0x04   /* OTP address (2 bytes, 11 bits used) */
#define DW_SUB_OTP_CTRL     0x06   /* OTP Control (2 bytes) */
#define DW_SUB_OTP_RDAT     0x0A   /* OTP read data (4 bytes) */
#define DW_SUB_OTP_SF       0x12   /* OTP special function (1 byte) */

/* PMSC sub-registers */
#define DW_SUB_PMSC_CTRL0   0x00   /* PMSC Control 0 (4 bytes) */
#define DW_SUB_PMSC_SOFTRESET 0x03 /* PMSC_CTRL0 byte holding the SOFTRESET bits */

/* EXT_SYNC sub-registers. EC_CTRL_PLLLCK enables the documented clock-PLL
 * lock detector used by the Decawave reference driver. */
#define DW_SUB_EC_CTRL      0x00
#define DW_EC_CTRL_PLLLCK   0x04U

/* OTP memory map (DW1000 UM, OTP memory map) */
#define DW_OTP_LDOTUNE_ADDR 0x04
#define DW_OTP_PARTID_ADDR  0x06
#define DW_OTP_LOTID_ADDR   0x07
#define DW_OTP_VBAT_ADDR    0x08
#define DW_OTP_VTEMP_ADDR   0x09
#define DW_OTP_XTRIM_ADDR   0x1E

/* ========================================================================== */
/*                     SYS_STATUS BIT MASKS                                    */
/* ========================================================================== */

#define DW_CPLOCK_BIT       (1UL << 1)   /* Clock PLL locked */
#define DW_TXFRB_BIT        (1UL << 4)   /* TX Frame Begins */
#define DW_TXPRS_BIT        (1UL << 5)   /* TX Preamble Sent */
#define DW_TXPHS_BIT        (1UL << 6)   /* TX PHY Header Sent */
#define DW_TXFRS_BIT        (1UL << 7)   /* TX Frame Sent */
#define DW_RXPRD_BIT        (1UL << 8)   /* RX Preamble Detected */
#define DW_RXSFDD_BIT       (1UL << 9)   /* RX SFD Detected */
#define DW_LDEDONE_BIT      (1UL << 10)  /* LDE processing done: timestamp valid */
#define DW_RXPHD_BIT        (1UL << 11)  /* RX PHY Header Detected */
#define DW_RXPHE_BIT        (1UL << 12)  /* RX PHY Header Error */
#define DW_RXDFR_BIT        (1UL << 13)  /* RX Data Frame Ready */
#define DW_RXFCG_BIT        (1UL << 14)  /* RX FCS Good */
#define DW_RXFCE_BIT        (1UL << 15)  /* RX FCS Error */
#define DW_RXRFSL_BIT       (1UL << 16)  /* RX Reed Solomon Frame Sync Loss */
#define DW_RXRFTO_BIT       (1UL << 17)  /* RX Frame Wait Timeout */
#define DW_LDEERR_BIT       (1UL << 18)  /* Leading-edge detection error */
#define DW_RXOVRR_BIT       (1UL << 20)  /* Receiver overrun */
#define DW_RXPTO_BIT        (1UL << 21)  /* Preamble Detection Timeout */
#define DW_RXSFDTO_BIT      (1UL << 26)  /* RX SFD timeout */
#define DW_HPDWARN_BIT      (1UL << 27)  /* Half Period Delay Warning (delayed TX late) */
#define DW_TXBERR_BIT       (1UL << 28)  /* TX buffer error */
#define DW_AFFREJ_BIT       (1UL << 29)  /* Automatic frame-filter rejection */
#define DW_TXPUTE_BIT       (1ULL << 34) /* TX power-up time error (5-byte status) */

/* Combined masks for convenience */
#define DW_ALL_RX_GOOD      (DW_RXDFR_BIT | DW_RXFCG_BIT)
#define DW_ALL_RX_ERR       (DW_RXPHE_BIT | DW_RXFCE_BIT | DW_RXRFSL_BIT | \
                             DW_RXRFTO_BIT | DW_LDEERR_BIT | DW_RXOVRR_BIT | \
                             DW_RXPTO_BIT | DW_RXSFDTO_BIT | DW_AFFREJ_BIT)
#define DW_ALL_TX_DONE      (DW_TXFRS_BIT)
#define DW_ALL_TX_EVENTS    (DW_TXFRB_BIT | DW_TXPRS_BIT | DW_TXPHS_BIT | \
                             DW_TXFRS_BIT | DW_HPDWARN_BIT | DW_TXBERR_BIT | \
                             DW_TXPUTE_BIT)

/**
 * @brief Outcome of one RX-related SYS_STATUS snapshot.
 *
 * A frame is only usable when data, FCS and leading-edge detection all
 * completed and no error flag is present in the same snapshot. RX_GOOD with,
 * for example, LDEERR set carries an unreliable timestamp and must be
 * rejected (FIRMWARE_REVIEW F3).
 */
typedef enum {
    DW_RX_EVENT_NONE  = 0,  /* no RX completion in this snapshot */
    DW_RX_EVENT_GOOD  = 1,  /* frame + FCS + timestamp valid, no error */
    DW_RX_EVENT_ERROR = 2,  /* any RX error/timeout, or incomplete frame */
} DW1000_RxEvent_t;

static inline DW1000_RxEvent_t DW1000_ClassifyRx(uint64_t status)
{
    const uint64_t good = DW_ALL_RX_GOOD | DW_LDEDONE_BIT;

    if ((status & DW_ALL_RX_ERR) != 0U)
        return DW_RX_EVENT_ERROR;
    if ((status & good) == good)
        return DW_RX_EVENT_GOOD;
    if ((status & DW_RXDFR_BIT) != 0U)
        return DW_RX_EVENT_ERROR;   /* frame ended without FCS/LDE success */
    return DW_RX_EVENT_NONE;
}

/* ========================================================================== */
/*                     SYS_CTRL BIT MASKS                                      */
/* ========================================================================== */

#define DW_TXSTRT_BIT       (1UL << 1)   /* Transmit Start */
#define DW_TXDLYS_BIT       (1UL << 2)   /* Transmitter Delayed Sending */
#define DW_TRXOFF_BIT       (1UL << 6)   /* Force TRX Off */
#define DW_WAIT4RESP_BIT    (1UL << 7)   /* Turn the receiver on after this TX */
#define DW_RXENAB_BIT       (1UL << 8)   /* Enable Receiver */

/* ========================================================================== */
/*                     DEVICE ID                                               */
/* ========================================================================== */

#define DW1000_DEVICE_ID    0xDECA0130UL

/* ========================================================================== */
/*                     FRAME / PROTOCOL CONSTANTS                              */
/* ========================================================================== */

#define FRAME_POLL_FUNC     0x21    /* Function code: Poll message */
#define FRAME_RESP_FUNC     0x10    /* Function code: Response message */
#define FRAME_REPORT_FUNC   0x22    /* Function code: Report message (Rb) */
#define FRAME_FINAL_FUNC    0x23    /* Function code: Final message */

#define DW_PAN_ID           0xDECA  /* PAN Identifier */
/* ANCHOR_ADDR and TAG_ADDR come from the node's uwb_app_config.h. */

/* Fast-50 profile: Ch5, 6.8Mbps, PRF16, preamble 256, PAC16, standard SFD.
 * DWT_PLEN_256 encoding is 0x24 in TX_FCTRL[21:18]. */
#define DW_PHY_PREAMBLE_SYMBOLS  256U
#define DW_PHY_PAC_SYMBOLS       16U
#define DW_PHY_SFD_TIMEOUT       249U  /* preamble + 1 + SFD(8) - PAC(16) */
#define DW_PHY_PROFILE_ID        2U    /* 0=unspecified, 1=legacy1024, 2=Fast-256 */
#define DW_TX_FCTRL_UPPER        0x0025C000UL

/* SPI starts at 2 MHz for reset/OTP/LDE, then moves to the 8 MHz maximum of
 * the nRF52832 SPI instance. The driver falls back to 2 MHz if the device-ID
 * readback is not reliable at 8 MHz. */
#define DW_SPI_INIT_MHZ          2U
#define DW_SPI_FAST_MHZ          8U
extern volatile uint8_t dw1000_spi_mhz;

/* Default antenna delay (Decawave starting point before calibration) */
#define DW_DEFAULT_ANT_DLY  16436   /* 0x4024 */

/**
 * Settling time after TRXOFF. The Decawave driver issues the next command
 * immediately; the baseline waited 100 us on every slot change. Override
 * with -DDW_TRXOFF_SETTLE_US=100 to restore the old behaviour for an A/B.
 */
#ifndef DW_TRXOFF_SETTLE_US
#define DW_TRXOFF_SETTLE_US 10U
#endif

/* ========================================================================== */
/*                     RUNTIME RADIO CONFIGURATION                             */
/* ========================================================================== */

/**
 * @brief Radio parameters that may change at runtime (commands/settings).
 *        Defaults come from uwb_calibration.h / uwb_app_config.h, so a build
 *        without stored settings behaves exactly like the compile-time build.
 */
typedef struct {
    uint16_t tx_ant_dly;        /* DTU, written to TX_ANTD */
    uint16_t rx_ant_dly;        /* DTU, written to LDE_RXANTD */
    uint8_t  tx_power_mode;     /* UWB_TX_POWER_* */
    uint8_t  reference_tuning;  /* see UWB_DW_REFERENCE_TUNING */
    uint32_t tx_power_custom;   /* TX_POWER value for UWB_TX_POWER_CUSTOM */
} DW1000_RadioConfig_t;

extern DW1000_RadioConfig_t dw1000_radio_config;

/**
 * @brief Factory values read from the DW1000 OTP during DW1000_Init().
 *        Always read (for diagnostics); only applied when reference tuning
 *        is enabled.
 */
typedef struct {
    uint32_t part_id;
    uint32_t lot_id;
    uint32_t ldotune;
    uint8_t  xtal_trim;         /* OTP 0x1E bits 4:0; 0 = crystal never trimmed */
    uint8_t  otp_rev;           /* OTP 0x1E bits 15:8 */
    uint8_t  vbat_cal;          /* SAR reading at 3.3 V (OTP 0x08) */
    uint8_t  vtemp_cal;         /* SAR reading at 23 degC (OTP 0x09) */
    uint8_t  applied_xtal_trim; /* value written to FS_XTALT, 0xFF = untouched */
    uint8_t  ldo_kicked;        /* 1 if the LDOTUNE kick was issued */
    uint8_t  valid;             /* 1 after a successful OTP read */
} DW1000_OtpInfo_t;

extern DW1000_OtpInfo_t dw1000_otp;

/* ========================================================================== */
/*                     RANGING RESULT STRUCTURE                                */
/* ========================================================================== */

/**
 * @brief Ranging result data — designed to be watched in the debugger
 */
typedef struct {
    int32_t  distance_mm;       /* Computed distance in millimeters */
    int32_t  distance_cm;       /* Computed distance in centimeters */
    uint8_t  poll_rx_ts[5];     /* Anchor: Poll RX timestamp (40-bit) */
    uint8_t  resp_tx_ts[5];     /* Anchor: Response TX timestamp (40-bit) */
    uint8_t  poll_tx_ts[5];     /* Tag: Poll TX timestamp (40-bit) */
    uint8_t  resp_rx_ts[5];     /* Tag: Response RX timestamp (40-bit) */
    uint32_t ranging_count;     /* Total successful ranging cycles */
    uint32_t timeout_count;     /* Total RX timeouts */
    uint32_t last_status;       /* Last SYS_STATUS value for debugging */
} DW1000_RangingResult;

/* ========================================================================== */
/*                     DIAGNOSTICS STRUCTURE                                   */
/* ========================================================================== */

/**
 * @brief Receive-quality diagnostics of the last frame (DW1000 UM 4.7).
 */
typedef struct {
    uint16_t rxpacc;    /* Preamble accumulation count (N) */
    uint16_t fp_ampl1;  /* First path amplitude 1 (F1) */
    uint16_t fp_ampl2;  /* First path amplitude 2 (F2) */
    uint16_t fp_ampl3;  /* First path amplitude 3 (F3) */
    uint16_t std_noise; /* Standard deviation of CIR noise */
    uint16_t cir_pwr;   /* Channel impulse response power (C) */
    uint16_t fp_index;  /* First path index, 10.6 fixed point */
} DW1000_SignalDiag_t;

/* ========================================================================== */
/*                     API FUNCTION PROTOTYPES                                 */
/* ========================================================================== */

/**
 * @brief  Hardware reset, verify Device ID, read OTP and load LDE microcode.
 *         Applies LDOTUNE/crystal trim only when reference tuning is enabled.
 * @retval 0 = success, -1 = device ID mismatch, -2 = platform setup failure
 */
int DW1000_Init(void);

/**
 * @brief  Switch SPI from the initialization rate to the runtime rate.
 *         Call only after DW1000_Init() and DW1000_Configure().
 * @return Selected rate in MHz (8 or 2); 0 if even the safe rate fails.
 */
uint8_t DW1000_EnableFastSPI(void);

/**
 * @brief  Configure DW1000 for Fast-256 ranging: Channel 5, PRF 16 MHz,
 *         Preamble 256, PAC16 and 6.8 Mbps, using dw1000_radio_config.
 * @retval 0 on success; -1 if the clock PLL did not lock within 100 ms.
 */
int DW1000_Configure(void);

/**
 * @brief  Write the antenna delays held in dw1000_radio_config (or zero in
 *         the legacy-offset profile). Safe to call while the radio is idle.
 */
void DW1000_ApplyAntennaDelay(void);

/**
 * @brief  TX antenna delay that the hardware actually adds to RMARKER
 *         (0 in the legacy-offset profile). Anchors need it to predict the
 *         delayed-TX RMARKER.
 */
uint16_t DW1000_ActiveTxAntennaDelay(void);

/** @brief TX_POWER register value implied by dw1000_radio_config. */
uint32_t DW1000_TxPowerRegisterValue(void);

/** @brief Expected SYS_CFG value implied by dw1000_radio_config. */
uint32_t DW1000_ExpectedSysCfg(void);

/**
 * @brief Stable identity of all radio/PHY inputs that affect range bias.
 *
 * Stored calibration is accepted only when this value and the DW1000 PARTID
 * match the values captured when the calibration was saved.
 */
uint32_t DW1000_CalibrationProfileId(void);

/**
 * @brief  Set the PAN ID and Short Address in DW1000's PANADR register.
 */
void DW1000_SetAddress(uint16_t pan_id, uint16_t short_addr);

/** @brief Write frame data to the TX buffer. */
void DW1000_WriteTxData(const uint8_t *data, uint16_t len);

/** @brief Set TX_FCTRL: frame length (including 2-byte FCS) + PHY bits. */
void DW1000_SetTxFrameCtrl(uint16_t len);

/** @brief Start an immediate transmission (TXSTRT). */
void DW1000_StartTx(void);

/**
 * @brief  Start an immediate transmission and let the DW1000 turn its
 *         receiver on as soon as the frame is sent (TXSTRT | WAIT4RESP).
 *         Removes the MCU latency between TX done and RX enable.
 */
void DW1000_StartTxWait4Resp(void);

/** @brief Set the delayed TX time (DX_TIME register). */
void DW1000_SetDelayedTxTime(const uint8_t tx_time[5]);

/**
 * @brief  Start a delayed transmission and check HPDWARN/TXPUTE at once.
 * @param  wait4resp  1 = turn the receiver on after the frame (WAIT4RESP)
 * @retval 0  = TX scheduled
 * @retval -1 = too late / power-up error; caller must recover immediately
 */
int DW1000_StartTxDelayedEx(uint8_t wait4resp);

/** @brief Delayed TX without WAIT4RESP (compatibility wrapper). */
int DW1000_StartTxDelayed(void);

/**
 * @brief  Wait for TX to complete (poll SYS_STATUS for TXFRS).
 * @retval 1 = TX done, 0 = timeout
 */
int DW1000_WaitTxDone(uint32_t timeout_ms);

/** @brief Enable the receiver (RXENAB). */
void DW1000_StartRx(void);

/**
 * @brief  Wait for a frame to be received.
 * @retval 0 = timeout, 1 = good frame, 2 = RX error
 */
int DW1000_WaitRxDone(uint32_t timeout_ms);

/**
 * @brief  Read the received frame (length from RX_FINFO, including FCS).
 * @retval Frame length in bytes, 0 when it does not fit in max_len.
 */
uint16_t DW1000_ReadRxData(uint8_t *data, uint16_t max_len);

/** @brief Read the 5-byte RX timestamp (RX_TIME, adjusted RMARKER). */
void DW1000_ReadRxTimestamp(uint8_t ts[5]);

/** @brief Read the 5-byte TX timestamp (TX_TIME, adjusted RMARKER). */
void DW1000_ReadTxTimestamp(uint8_t ts[5]);

/** @brief Read the 40-bit system time counter. */
void DW1000_ReadSysTime(uint8_t ts[5]);

/** @brief Read the receive-quality diagnostics of the last frame. */
void DW1000_ReadSignalDiag(DW1000_SignalDiag_t *diag);

/** @brief First path power in dBm (DW1000 UM 4.7.1). */
float DW1000_GetFirstPathPower(const DW1000_SignalDiag_t *diag);

/** @brief Estimated total receive power in dBm (DW1000 UM 4.7.2). */
float DW1000_GetRxPower(const DW1000_SignalDiag_t *diag);

/** @brief Read the 21-bit signed carrier integrator. */
int32_t DW1000_ReadCarrierIntegrator(void);

/**
 * @brief  Clock offset of the remote transmitter relative to this receiver,
 *         in units of 0.01 ppm (Ch5, 6.8 Mbps; Decawave sign convention).
 */
int32_t DW1000_CarrierIntegratorToPpmX100(int32_t carrier_integrator);

/** @brief Clear every event flag in SYS_STATUS (all 5 bytes). */
void DW1000_ClearAllStatus(void);

/**
 * @brief  Clear only the TX event flags. Used after WAIT4RESP so an RX event
 *         that may already be pending is not lost.
 */
void DW1000_ClearTxStatus(void);

/** @brief Force the transceiver off (TRXOFF). */
void DW1000_ForceRxOff(void);

/**
 * @brief  Reset the receiver block (PMSC soft reset) so the LDE is correctly
 *         re-initialised after an RX error, as the Decawave examples do.
 *         Call with the transceiver off.
 */
void DW1000_RxSoftReset(void);

/** @brief Read the SYS_STATUS register (5 bytes, 40 bits used). */
uint64_t DW1000_ReadStatus(void);

/** @brief Read the DW1000 Device ID register (0xDECA0130 expected). */
uint32_t DW1000_ReadDeviceID(void);

/**
 * @brief  Enable the devicetree DW1000 IRQ pin as a rising-edge interrupt.
 *         Must be called after DW1000_Init() + DW1000_Configure() +
 *         DW1000_ClearAllStatus().
 */
void DW1000_EnableIRQ(void);

/**
 * @brief  IRQ flag set by the Zephyr GPIO callback; cleared by the
 *         application after handling. The IRQ is level-held by the DW1000, so
 *         the main loops also poll DW1000_IrqLineActive().
 */
extern volatile uint8_t dw1000_irq_flag;

/* Transport failures since boot. The health module turns an increase into a
 * radio recovery instead of publishing data read through a failed transfer. */
extern volatile uint32_t dw1000_spi_error_count;

/* ========================================================================== */
/*                     µs TIMER, IRQ LEVEL, CONFIG VERIFY                      */
/* ========================================================================== */

/** @brief Initialize the platform cycle timer abstraction. */
void MCU_TimerInit(void);

/**
 * @brief  Microseconds since boot (wraps with the 32-bit cycle counter).
 *         Only for absolute values; use MCU_CycleNow() + MCU_ElapsedUs()
 *         for deltas (FIX-02).
 */
uint32_t MCU_Micros(void);

/** @brief Raw cycle-counter stamp used as the start point of MCU_ElapsedUs(). */
typedef uint32_t McuCycleStamp_t;

/** @brief Read the raw cycle counter. */
McuCycleStamp_t MCU_CycleNow(void);

/** @brief Microseconds elapsed since `start`, wrap-safe (FIX-02). */
uint32_t MCU_ElapsedUs(McuCycleStamp_t start);

/** @brief 1 if the DW1000 IRQ line is currently asserted. */
uint8_t DW1000_IrqLineActive(void);

/* Bits returned by DW1000_VerifyConfig(): which register differs. */
#define DW_VERIFY_DEVID      0x0001U
#define DW_VERIFY_SYS_CFG    0x0002U
#define DW_VERIFY_CHAN_CTRL  0x0004U
#define DW_VERIFY_SFDTOC     0x0008U
#define DW_VERIFY_DRX_TUNE2  0x0010U
#define DW_VERIFY_TX_POWER   0x0020U
#define DW_VERIFY_TX_ANTD    0x0040U
#define DW_VERIFY_RX_ANTD    0x0080U
#define DW_VERIFY_LDE_CFG1   0x0100U
#define DW_VERIFY_XTALT      0x0200U
#define DW_VERIFY_SYS_MASK   0x0400U

/**
 * @brief  Read back the persistent PHY registers and compare with what
 *         DW1000_Configure() wrote. A DW1000 brown-out silently restores
 *         reset defaults, which this detects.
 * @retval Bitmask of DW_VERIFY_* (0 = everything matches).
 */
uint32_t DW1000_VerifyConfig(void);

/**
 * @brief  Read the die temperature and supply voltage (SAR), ~1 ms BLOCKING.
 *         Only call while the radio is idle.
 */
void DW1000_ReadTempVbatRaw(uint8_t *temp_raw, uint8_t *vbat_raw);

/** @brief Die temperature SAR reading only (compatibility wrapper). */
uint8_t DW1000_ReadTemperatureRaw(void);

/**
 * @brief  Convert SAR readings with the OTP calibration points.
 * @retval INT16_MIN / 0 when the OTP calibration is unavailable.
 */
int16_t DW1000_TemperatureCentiDeg(uint8_t temp_raw);
uint16_t DW1000_VoltageMilliVolt(uint8_t vbat_raw);

#endif /* DW1000_HW_H */

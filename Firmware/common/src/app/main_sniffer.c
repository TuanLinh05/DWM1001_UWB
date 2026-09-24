/**
 ******************************************************************************
 * @file    main_sniffer.c
 * @brief   Passive UWB sniffer (Bitcraze LPS "sniffer" mode, plan item 2.7).
 *
 * Listens continuously with the ranging PHY and emits every received frame
 * as a SNIFFER_FRAME telemetry packet (TYPE 0x14):
 *   schema[1]=1 rx_ts_lo[4] rx_ts_hi[1] fp_cdbm[2] rx_cdbm[2] status[1]
 *   length[1] frame[length]        (frame without FCS)
 * status bit0 = frame good, bit1 = RX error (length 0), bit2 = truncated.
 *
 * Tools/uwb_sniffer.py decodes the stream into an exchange timeline. The
 * sniffer never transmits.
 ******************************************************************************
 */

#include "dw1000_hw.h"
#include "telemetry_frame.h"
#include "uart_tx.h"
#include "uwb_health.h"
#include "uwb_platform.h"

#include <limits.h>
#include <string.h>
#include <zephyr/kernel.h>

#define SNIFFER_MAX_FRAME       127U
#define SNIFFER_IDLE_RESTART_MS 200U
#define SNIFFER_STATUS_GOOD     0x01U
#define SNIFFER_STATUS_ERROR    0x02U
#define SNIFFER_STATUS_TRUNC    0x04U
#define SNIFFER_ADDRESS         0xFFFEU   /* never matches a ranging node */

static uint32_t s_frames;

static void platform_fault_forever(void)
{
    while (true) {
        uwb_platform_led_toggle();
        k_msleep(150);
    }
}

static int16_t to_cdbm(float dbm)
{
    const float centi = dbm * 100.0f;
    if (!(centi > (float)INT16_MIN)) {
        return INT16_MIN;
    }
    return centi > (float)INT16_MAX ? INT16_MAX : (int16_t)centi;
}

static int sniffer_radio_init(void)
{
    if (DW1000_Init() != 0) {
        return -1;
    }
    if (DW1000_Configure() != 0) {
        return -2;
    }
    DW1000_SetAddress(DW_PAN_ID, SNIFFER_ADDRESS);
    if (DW1000_EnableFastSPI() == 0U) {
        return -3;
    }
    if (DW1000_VerifyConfig() != 0U) {
        return -4;
    }
    DW1000_ClearAllStatus();
    DW1000_StartRx();
    return 0;
}

static void emit(uint8_t status, const uint8_t *frame, uint8_t length,
                 const uint8_t ts[5], float fpp, float rx)
{
    uint8_t payload[12U + SNIFFER_MAX_FRAME];
    uint16_t off = 0U;

    payload[off++] = 1U;
    off += telem_put_u32(&payload[off], (uint32_t)ts[0] | ((uint32_t)ts[1] << 8)
                         | ((uint32_t)ts[2] << 16) | ((uint32_t)ts[3] << 24));
    payload[off++] = ts[4];
    off += telem_put_u16(&payload[off], (uint16_t)to_cdbm(fpp));
    off += telem_put_u16(&payload[off], (uint16_t)to_cdbm(rx));
    payload[off++] = status;
    payload[off++] = length;
    if (length > 0U) {
        memcpy(&payload[off], frame, length);
        off = (uint16_t)(off + length);
    }
    Telem_SendRaw(TELEM_TYPE_SNIFFER_FRAME, s_frames++, payload, off);
}

static void handle_event(void)
{
    const uint64_t status = DW1000_ReadStatus();
    const DW1000_RxEvent_t event = DW1000_ClassifyRx(status);
    static const uint8_t no_ts[5] = {0};

    if (event == DW_RX_EVENT_GOOD) {
        uint8_t frame[SNIFFER_MAX_FRAME];
        uint8_t ts[5];
        DW1000_SignalDiag_t diag;
        uint16_t length = DW1000_ReadRxData(frame, SNIFFER_MAX_FRAME);
        uint8_t flags = SNIFFER_STATUS_GOOD;

        if (length == 0U) {
            flags |= SNIFFER_STATUS_TRUNC;
        } else if (length >= 2U) {
            length = (uint16_t)(length - 2U);      /* drop the FCS */
        }
        DW1000_ReadRxTimestamp(ts);
        DW1000_ReadSignalDiag(&diag);
        DW1000_ClearAllStatus();
        DW1000_StartRx();                          /* re-arm before the UART work */
        emit(flags, frame, (uint8_t)length, ts, DW1000_GetFirstPathPower(&diag),
             DW1000_GetRxPower(&diag));
    } else if (event == DW_RX_EVENT_ERROR) {
        DW1000_ForceRxOff();
        DW1000_RxSoftReset();
        DW1000_ClearAllStatus();
        DW1000_StartRx();
        emit(SNIFFER_STATUS_ERROR, NULL, 0U, no_ts, -120.0f, -120.0f);
    } else {
        DW1000_ClearAllStatus();
    }
}

int main(void)
{
    if (uwb_platform_init() != 0) {
        platform_fault_forever();
    }
    MCU_TimerInit();
    (void)uwb_health_init();
    if (UART_TX_Init() != 0) {
        platform_fault_forever();
    }

    uint8_t radio_ok = (sniffer_radio_init() == 0) ? 1U : 0U;
    if (!radio_ok) {
        radio_ok = (uwb_health_recover_radio(sniffer_radio_init, UWB_FAULT_INIT) == 0);
    }
    DW1000_EnableIRQ();

    uint32_t last_event_ms = uwb_platform_time_ms();
    uint32_t spi_errors_seen = dw1000_spi_error_count;

    while (true) {
        const uint32_t now = uwb_platform_time_ms();

        if (DW1000_IrqLineActive() != 0U) {
            dw1000_irq_flag = 1U;
        }

        if (radio_ok) {
            if (dw1000_irq_flag) {
                dw1000_irq_flag = 0U;
                handle_event();
                last_event_ms = now;
                uwb_health_note_progress();
            } else if ((uint32_t)(now - last_event_ms) > SNIFFER_IDLE_RESTART_MS) {
                /* Quiet air: restart RX (DW1000 RX lock-up guard). */
                DW1000_ForceRxOff();
                DW1000_ClearAllStatus();
                DW1000_StartRx();
                last_event_ms = now;
                uwb_health_note_progress();
                uwb_platform_led_toggle();
            }
            if (dw1000_spi_error_count != spi_errors_seen) {
                radio_ok = (uwb_health_recover_radio(sniffer_radio_init, UWB_FAULT_SPI) == 0);
                spi_errors_seen = dw1000_spi_error_count;
            }
        } else if (uwb_health_hold_retry_due()) {
            radio_ok = (uwb_health_recover_radio(sniffer_radio_init, UWB_FAULT_INIT) == 0);
            spi_errors_seen = dw1000_spi_error_count;
        }

        uwb_health_service();
    }

    return 0;
}

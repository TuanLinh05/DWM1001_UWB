/* TAG command executor: ACKs, safety rules and radio side effects. */
#include <stdio.h>
#include <string.h>

#include "dw1000_sim.h"
#include "uart_tx.h"
#include "uwb_cmd.h"
#include "uwb_health.h"
#include "uwb_settings.h"
#include "../common/src/ranging/tag_ranging.c"
#include "telemetry.h"

#define CHECK(cond) do { if (!(cond)) { \
    fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
    return 1; } } while (0)

/* ---- stubs for modules that need Zephyr ---------------------------------- */
UwbHealth_t uwb_health;
uint8_t uwb_settings_status;
volatile uint32_t uart_tx_overflow_count;
volatile uint32_t uart_rx_overflow_count;
volatile uint16_t uart_tx_high_water;
static int s_saves;

void uwb_health_feed(void) {}
void uwb_health_note_config_check(uint32_t mismatch) { uwb_health.last_config_mismatch = mismatch; }
int uwb_settings_save(void) { s_saves++; return 0; }
int uwb_settings_factory_reset(void) { return 0; }

static uint8_t s_out[4096];
static size_t s_out_len;
static uint8_t s_in[512];
static uint16_t s_in_len;
static uint16_t s_in_pos;

uint32_t UART_TX_Write(const uint8_t *data, uint16_t length)
{
    if (s_out_len + length > sizeof(s_out))
        return 0U;
    memcpy(&s_out[s_out_len], data, length);
    s_out_len += length;
    return length;
}

uint16_t UART_TX_Pending(void) { return 0U; }

uint16_t UART_RX_Read(uint8_t *data, uint16_t max_length)
{
    uint16_t n = 0U;
    while (n < max_length && s_in_pos < s_in_len)
        data[n++] = s_in[s_in_pos++];
    return n;
}

/* ---- helpers -------------------------------------------------------------- */
static uint8_t s_ack_id, s_ack_result, s_ack_data[64];
static uint16_t s_ack_len;

/* Send a command frame through the real parser/executor, return the ACK. */
static int send_cmd(uint32_t seq, const uint8_t *payload, uint16_t length)
{
    uint8_t frame[TELEM_MAX_FRAME];
    memcpy(&frame[TELEM_HDR_LEN], payload, length);
    const uint16_t total = telem_frame_pack(frame, TELEM_TYPE_CMD, seq, 0U, length);
    memcpy(s_in, frame, total);
    s_in_len = total;
    s_in_pos = 0U;
    s_out_len = 0U;
    uwb_cmd_poll();

    /* Find the CMD_ACK among the emitted frames. */
    size_t pos = 0U;
    while (pos + TELEM_HDR_LEN + TELEM_CRC_LEN <= s_out_len)
    {
        const uint16_t len = (uint16_t)(s_out[pos + 4] | (s_out[pos + 5] << 8));
        const uint32_t fseq = (uint32_t)s_out[pos + 6] | ((uint32_t)s_out[pos + 7] << 8)
                            | ((uint32_t)s_out[pos + 8] << 16) | ((uint32_t)s_out[pos + 9] << 24);
        if (s_out[pos + 3] == TELEM_TYPE_CMD_ACK && fseq == seq)
        {
            s_ack_id = s_out[pos + TELEM_HDR_LEN];
            s_ack_result = s_out[pos + TELEM_HDR_LEN + 1];
            s_ack_len = (uint16_t)(len - 2U);
            memcpy(s_ack_data, &s_out[pos + TELEM_HDR_LEN + 2], s_ack_len);
            return 0;
        }
        pos += (size_t)TELEM_HDR_LEN + len + TELEM_CRC_LEN;
    }
    return -1;
}

static int pause_tag(void)
{
    const uint8_t pause[] = { UWB_CMD_PAUSE };
    if (send_cmd(900U, pause, 1U) != 0 || s_ack_result != UWB_CMD_OK)
        return -1;
    Tag_Task();                      /* IDLE: pause takes effect */
    return Tag_IsPaused() ? 0 : -1;
}

int main(void)
{
    sim_reset();
    CHECK(Tag_Init() == 0);
    uwb_cmd_init();
    memset(&uwb_cmd_stats, 0, sizeof(uwb_cmd_stats));

    /* PING */
    const uint8_t ping[] = { UWB_CMD_PING };
    CHECK(send_cmd(1U, ping, 1U) == 0);
    CHECK(s_ack_id == UWB_CMD_PING && s_ack_result == UWB_CMD_OK && s_ack_len == 4U);

    /* Unknown command and bad length */
    const uint8_t unknown[] = { 0x7E };
    CHECK(send_cmd(2U, unknown, 1U) == 0 && s_ack_result == UWB_CMD_ERR_UNKNOWN);
    const uint8_t short_cal[] = { UWB_CMD_SET_DS_CAL, 1, 0 };
    CHECK(send_cmd(3U, short_cal, sizeof(short_cal)) == 0 && s_ack_result == UWB_CMD_ERR_LENGTH);

    /* Runtime calibration is allowed while ranging and read back. */
    const uint8_t set_cal[] = { UWB_CMD_SET_DS_CAL, 0x02, 0x00, 0xD4, 0xCF, 0xFF, 0xFF, 0x01 };
    CHECK(send_cmd(4U, set_cal, sizeof(set_cal)) == 0 && s_ack_result == UWB_CMD_OK);
    const uint8_t get_cal[] = { UWB_CMD_GET_DS_CAL };
    CHECK(send_cmd(5U, get_cal, 1U) == 0 && s_ack_result == UWB_CMD_OK && s_ack_len == 56U);
    int32_t bias = 0;
    memcpy(&bias, &s_ack_data[7 + 2], 4);       /* second record: A2 */
    CHECK(bias == -12332 && s_ack_data[7 + 6] == 1U);

    /* Radio changes need a pause. */
    const uint8_t ant[] = { UWB_CMD_SET_ANT_DELAY, 0x40, 0x40, 0x41, 0x40 };
    CHECK(send_cmd(6U, ant, sizeof(ant)) == 0 && s_ack_result == UWB_CMD_ERR_NOT_PAUSED);
    CHECK(pause_tag() == 0);
    CHECK(send_cmd(7U, ant, sizeof(ant)) == 0 && s_ack_result == UWB_CMD_OK);
    CHECK((sim_read_u32(DW_REG_TX_ANTD, 0) & 0xFFFFU) == 0x4040U);
    CHECK((sim_read_u32(DW_REG_LDE_IF, DW_SUB_LDE_RXANTD) & 0xFFFFU) == 0x4041U);
    CHECK(Tag_DsCalibratedMask() == 0U);       /* RF change fails closed */

    /* Re-calibrate, then repeat an identical RF command: no invalidation. */
    CHECK(send_cmd(70U, set_cal, sizeof(set_cal)) == 0 && s_ack_result == UWB_CMD_OK);
    CHECK(Tag_DsCalibratedMask() == 0x02U);
    CHECK(send_cmd(71U, ant, sizeof(ant)) == 0 && s_ack_result == UWB_CMD_OK);
    CHECK(Tag_DsCalibratedMask() == 0x02U);

    const uint8_t txp[] = { UWB_CMD_SET_TX_POWER, UWB_TX_POWER_REFERENCE, 0, 0, 0, 0 };
    CHECK(send_cmd(8U, txp, sizeof(txp)) == 0 && s_ack_result == UWB_CMD_OK);
    CHECK(sim_read_u32(DW_REG_TX_POWER, 0) == 0x48484848UL);
    CHECK(Tag_DsCalibratedMask() == 0U);
    const uint8_t bad_txp[] = { UWB_CMD_SET_TX_POWER, UWB_TX_POWER_CUSTOM, 0, 0, 0, 0 };
    CHECK(send_cmd(9U, bad_txp, sizeof(bad_txp)) == 0 && s_ack_result == UWB_CMD_ERR_ARGUMENT);

    /* Candidate failure + verified rollback keeps calibration and old config. */
    CHECK(send_cmd(72U, set_cal, sizeof(set_cal)) == 0 && s_ack_result == UWB_CMD_OK);
    const uint8_t smart_txp[] = { UWB_CMD_SET_TX_POWER, UWB_TX_POWER_SMART, 0, 0, 0, 0 };
    sim_pll_failures_remaining = 1U;
    CHECK(send_cmd(73U, smart_txp, sizeof(smart_txp)) == 0
          && s_ack_result == UWB_CMD_ERR_RADIO);
    CHECK(dw1000_radio_config.tx_power_mode == UWB_TX_POWER_REFERENCE);
    CHECK(Tag_DsCalibratedMask() == 0x02U);

    /* If rollback also cannot prove the radio state, calibration fails closed. */
    sim_force_pll_unlock = 1;
    CHECK(send_cmd(74U, smart_txp, sizeof(smart_txp)) == 0
          && s_ack_result == UWB_CMD_ERR_RADIO);
    CHECK(Tag_DsCalibratedMask() == 0U);
    sim_force_pll_unlock = 0;
    CHECK(DW1000_Configure() == 0);      /* restore simulator hardware */

    const uint8_t save[] = { UWB_CMD_SAVE_SETTINGS };
    CHECK(send_cmd(10U, save, 1U) == 0 && s_ack_result == UWB_CMD_OK && s_saves == 1);

    /* LOCK: configuration refused, monitoring still allowed. */
    const uint8_t lock[] = { UWB_CMD_SET_LOCK, 1 };
    CHECK(send_cmd(11U, lock, sizeof(lock)) == 0 && s_ack_result == UWB_CMD_OK);
    CHECK(send_cmd(12U, set_cal, sizeof(set_cal)) == 0 && s_ack_result == UWB_CMD_ERR_LOCKED);
    const uint8_t pause_cmd[] = { UWB_CMD_PAUSE };
    CHECK(send_cmd(13U, pause_cmd, 1U) == 0 && s_ack_result == UWB_CMD_ERR_LOCKED);
    CHECK(send_cmd(14U, ping, 1U) == 0 && s_ack_result == UWB_CMD_OK);
    const uint8_t unlock[] = { UWB_CMD_SET_LOCK, 0 };
    CHECK(send_cmd(15U, unlock, sizeof(unlock)) == 0 && s_ack_result == UWB_CMD_OK);

    /* RANGE_MEAS needs >= 460800 baud: refused on the 115200 host config. */
    const uint8_t telem[] = { UWB_CMD_SET_TELEMETRY, TELEM_FEATURE_RANGE_MEAS };
    CHECK(send_cmd(16U, telem, sizeof(telem)) == 0 && s_ack_result == UWB_CMD_ERR_UNSUPPORTED);

    /* TIME_SYNC echoes t1 and stamps t2/t3 with the TAG clock. */
    const uint8_t sync[] = { UWB_CMD_TIME_SYNC, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x00 };
    CHECK(send_cmd(17U, sync, sizeof(sync)) == 0 && s_ack_result == UWB_CMD_OK && s_ack_len == 24U);
    uint64_t t1 = 0, t2 = 0;
    memcpy(&t1, &s_ack_data[0], 8);
    memcpy(&t2, &s_ack_data[8], 8);
    CHECK(t1 == 0x0077665544332211ULL && t2 == sim_us);

    CHECK(uwb_cmd_stats.rejected >= 6U && uwb_cmd_stats.executed >= 8U);
    puts("command executor tests passed (ACK, LOCK, pause rule, radio writes, time sync)");
    return 0;
}

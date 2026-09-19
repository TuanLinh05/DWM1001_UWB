/* Run the real TAG state machine against a deterministic SPI/time model. */
#include <assert.h>
#include <stdio.h>
#include "../Tag/src/ranging/tag_ranging.c"

static uint32_t test_us;
static uint64_t test_status;
static uint8_t test_frame[16];
static uint8_t reg;
static bool header;
int uwb_platform_init(void) { return 0; }
int uwb_platform_spi_set_frequency(uint32_t hz) { (void)hz; return 0; }
void uwb_platform_cs_set(bool active) { if (active) header = true; }
int uwb_platform_spi_transfer(const uint8_t *tx, uint8_t *rx, size_t n)
{
    if (header && tx) { reg = tx[0] & 0x3FU; header = false; }
    else if (rx) {
        memset(rx, 0, n);
        if (reg == DW_REG_SYS_STATUS)
            for (size_t i=0; i<n; i++) rx[i] = (uint8_t)(test_status >> (i*8));
        if (reg == DW_REG_RX_FINFO) rx[0] = sizeof(test_frame);
        if (reg == DW_REG_RX_BUFFER) memcpy(rx, test_frame, n);
    } else if (tx && reg == DW_REG_SYS_STATUS) test_status = 0;
    return 0;
}
void uwb_platform_reset_radio(void) {}
void uwb_platform_enable_irq(void) {}
bool uwb_platform_irq_active(void) { return test_status != 0; }
void uwb_platform_delay_ms(uint32_t ms) { (void)ms; }
void uwb_platform_delay_us(uint32_t us) { (void)us; }
uint32_t uwb_platform_time_ms(void) { return test_us/1000; }
uint32_t uwb_platform_cycle_now(void) { return test_us; }
uint32_t uwb_platform_elapsed_us(uint32_t start) { return test_us-start; }
void uwb_platform_led_toggle(void) {}
void uwb_platform_led_set(bool on) { (void)on; }

int main(void)
{
    /* The production topology and scheduler must stay aligned. */
    _Static_assert(TAG_NUM_ANCHORS == 8U, "TAG must poll all eight anchors");
    _Static_assert(TAG_CYCLE_MS == 20U, "Eight-anchor cycle must stay at 50 Hz");
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++) {
        assert(anchor_id_at(i) == (uint16_t)(i + 1U));
    }

    /* A previously valid result becomes invalid when the next POLL cannot TX. */
    s_current_anchor = 0;
    s_track[0].valid = 1;
    s_state = TAG_STATE_TX_POLL;
    s_state_cycle = 0;
    test_us = TAG_TX_TIMEOUT_US + 1;
    dw1000_irq_flag = 0;
    Tag_Task();
    assert(s_track[0].valid == 0 && s_track[0].status == TAG_ST_TIMEOUT);
    assert(anchor_poll_tx_timeout_count[0] == 1);
    assert(rx_timeout_count == 1);

    /* A pending TX-complete event wins even if the task was scheduled late. */
    s_current_anchor = 0;
    s_state = TAG_STATE_TX_POLL;
    s_state_cycle = 0;
    test_status = DW_TXFRS_BIT;
    dw1000_irq_flag = 1;
    Tag_Task();
    assert(s_state == TAG_STATE_WAIT_RESP);
    assert(anchor_poll_tx_timeout_count[0] == 1);

    /* Unrelated but correctly framed traffic must not extend the deadline. */
    test_frame[0] = 0x41; test_frame[1] = 0x88;
    test_frame[3] = (uint8_t)DW_PAN_ID;
    test_frame[4] = (uint8_t)(DW_PAN_ID >> 8);
    test_frame[9] = 0xFF;
    test_status = DW_ALL_RX_GOOD;
    dw1000_irq_flag = 1;
    s_state_cycle = 0;
    test_us = TAG_RESP_TIMEOUT_US + 1;
    Tag_Task();
    assert(s_state != TAG_STATE_WAIT_RESP);
    assert(s_track[0].status == TAG_ST_TIMEOUT);
    puts("TAG timeout/IRQ/deadline tests passed");
    return 0;
}

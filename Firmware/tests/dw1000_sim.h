/*
 * Register-level DW1000 simulator for host tests.
 *
 * Implements the uwb_platform.h API on a PC: SPI transactions are decoded
 * (register file, sub-address, read/write) and served from a register image.
 * SYS_STATUS is write-one-to-clear, SYS_CTRL writes are logged, TX_BUFFER
 * writes are captured, OTP reads follow the OTP_ADDR/OTP_CTRL/OTP_RDAT
 * sequence. Time is simulated: 1 cycle = 1 µs.
 */
#ifndef DW1000_SIM_H
#define DW1000_SIM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SIM_REG_COUNT   64U
#define SIM_SUB_SPAN    0x3000U
#define SIM_CTRL_LOG    64U

extern uint64_t sim_us;               /* simulated time */
extern uint64_t sim_status;           /* SYS_STATUS (40 bits) */
extern uint8_t  sim_reg[SIM_REG_COUNT][SIM_SUB_SPAN];
extern uint32_t sim_otp[0x800];
extern uint8_t  sim_tx_frame[128];
extern uint16_t sim_tx_len;           /* bytes written to TX_BUFFER */
extern uint32_t sim_tx_count;         /* TX starts (TXSTRT writes) */
extern uint8_t  sim_ctrl_log[SIM_CTRL_LOG];
extern uint32_t sim_ctrl_count;
extern uint32_t sim_rx_enable_count;  /* RXENAB writes */
extern uint32_t sim_rx_soft_resets;   /* PMSC RX soft reset sequences */
extern int      sim_fail_spi;
extern int      sim_force_hpdwarn;    /* next delayed TX start sets HPDWARN */
extern int      sim_force_pll_unlock; /* PLL tune never raises CPLOCK */
extern int      sim_pll_lock_event_enabled; /* tune write may latch CPLOCK */
extern uint32_t sim_pll_failures_remaining; /* fail this many PLL tune writes */
extern uint32_t sim_spi_calls;

void sim_reset(void);

/** Last value written to SYS_CTRL byte 0 (TX/RX commands). */
uint8_t sim_last_ctrl(void);

/** Mark a TX as finished: TX_TIME = ts, TXFRS set, IRQ flag raised. */
void sim_complete_tx(uint64_t ts);

/**
 * Deliver a received frame (without FCS; the simulator appends 2 bytes).
 * extra_status is OR-ed into SYS_STATUS (e.g. DW_LDEERR_BIT).
 */
void sim_deliver_rx(const uint8_t *frame, uint16_t len, uint64_t rx_ts,
                    uint64_t extra_status);

/** Set RXPACC and first-path amplitudes / CIR power for the next frame. */
void sim_set_rx_quality(uint16_t rxpacc, uint16_t f1, uint16_t f2, uint16_t f3,
                        uint16_t cir_pwr, uint16_t std_noise);

void sim_write_bytes(uint8_t reg, uint16_t sub, const uint8_t *data, uint16_t len);
uint32_t sim_read_u32(uint8_t reg, uint16_t sub);

#endif

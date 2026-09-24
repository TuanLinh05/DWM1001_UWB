/*
 * End-to-end motion test of the TAG ranging pipeline.
 *
 * The real TAG state machine (tag_ranging.c: scheduler, DS-TWR maths,
 * runtime calibration, range conditioner and measurement queue) runs against
 * the register-level DW1000 simulator. A radio-channel model plays the eight
 * anchors: every POLL and FINAL that the TAG transmits becomes a RESP and a
 * REPORT whose timestamps follow from
 *   - a physical trajectory (radial ramps, a sine, or a 3-D flight through a
 *     room with eight anchors),
 *   - two independent 40-bit clocks with ppm drift and a start near the wrap,
 *   - the Fast-256 PHY airtime (preamble 256, 6.8 Mb/s, standard SFD),
 *   - the anchors' real reply rule (ANCHOR_REPLY_DELAY_UUS, 9-bit DX_TIME
 *     truncation, TX antenna delay) including late delayed TX,
 *   - RX timestamp noise, a per-pair antenna-delay bias that the TAG is
 *     calibrated for, NLOS excess path and anchor dropouts.
 *
 * Every published measurement (RANGE_MEAS queue) is compared with the true
 * distance of its own exchange. The conditioner under test is chosen at build
 * time (UWB_RANGE_FILTER_MODE, UWB_C9_2_MOTION_MODE, MOTION_LEGACY_ADAPTIVE_MODE),
 * so the runner compiles this file once per candidate.
 *
 * Usage: test_tag_motion [--report] [--seeds N]
 *   default    exit 1 when a scenario misses its acceptance gate
 *   --report   print the same table but always exit 0 (characterisation run)
 *   --seeds N  repeat every scenario with N noise/NLOS seeds and print, per
 *              scenario, in how many seeds it missed the gate and the worst
 *              value of each metric (robustness run; seed 0 is the default)
 *
 * The gates encode what a range conditioner must deliver before its output
 * may feed a flight estimator. They are proposals to be confirmed with
 * hardware logs, not measured requirements.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dw1000_sim.h"
#include "anchor_ranging.h"   /* ANCHOR_REPLY_DELAY_UUS: the anchors' reply rule */
#include "../common/src/ranging/tag_ranging.c"

/* ========================================================================== */
/*                     PHYSICAL AND PHY MODEL                                 */
/* ========================================================================== */

#define PI_D              3.14159265358979323846
#define TICKS_PER_US      (499.2 * 128.0)             /* DW1000 time base */
#define TS40_MASK         0xFFFFFFFFFFULL
#define PSYM_US           0.99359                     /* preamble symbol, PRF16 */
#define SFD_SYMBOLS       8.0                         /* standard SFD, 6.8 Mb/s */
#ifndef MOTION_PREAMBLE_SYMBOLS
#define MOTION_PREAMBLE_SYMBOLS DW_PHY_PREAMBLE_SYMBOLS  /* PHY the firmware uses */
#endif
#define SHR_US            (((double)MOTION_PREAMBLE_SYMBOLS + SFD_SYMBOLS) * PSYM_US)
#define PHR_US            (19.0 * 1.02564)            /* 19-bit PHR at 850 kb/s */
#define CODED_BIT_US      0.12821                     /* 6.8 Mb/s incl. Reed-Solomon */
#define TX_STARTUP_US     10.0                        /* model: TXSTRT -> preamble */
#define ANT_DLY_TICKS     ((uint64_t)UWB_TX_ANT_DLY)  /* programmed = physical here */
/* Model knobs for sensitivity runs; the defaults are the gated model. The
 * anchors' reply delay defaults to the value their firmware really uses. */
#ifndef MOTION_REPLY_DELAY_UUS
#define MOTION_REPLY_DELAY_UUS  ANCHOR_REPLY_DELAY_UUS
#endif
#ifndef MOTION_TS_NOISE_TICKS
#define MOTION_TS_NOISE_TICKS   10.4   /* per RX timestamp: ~30 mm DS-TWR sigma */
#endif
#ifndef MOTION_TAG_RESP_US
#define MOTION_TAG_RESP_US      350.0  /* TAG CPU: RESP RXFCG -> FINAL TXSTRT */
#endif
#define REPLY_TICKS       ((uint64_t)MOTION_REPLY_DELAY_UUS * 65536ULL)
#define TS_NOISE_TICKS    MOTION_TS_NOISE_TICKS
#define FALSE_ACCEPT_MM   300.0
#define RECOVERED_MM      150.0
/* The mean of a few hundred noisy ranges is only known to a few mm, so the
 * lag gate never asks for a mean error below LAG_FLOOR_MM: at 0.1 m/s a pure
 * 60 ms gate (6 mm) would judge the noise. A Legacy freeze at 0.1 m/s is a
 * 290 mm mean error, far outside either limit. */
#define LAG_FLOOR_MM      10.0
#define TRUTH_RING        256U
#define MAX_EVENTS        8U
#define MAX_SAMPLES       40000U
#define NO_GATE           (-1.0)

/* Per-pair bias from antenna-delay error (mm). The TAG is calibrated with the
 * same values, so the corrected range must be unbiased: this also checks the
 * sign convention corrected = measured - bias end to end. */
static const double k_pair_bias_mm[TAG_NUM_ANCHORS] = {
    87.0, -43.0, 125.0, -12.0, 64.0, -95.0, 31.0, 150.0
};

/* Room used by the eight-anchor flight: corners of a 10 x 8 x 3 m box. */
static const double k_room_anchor_mm[TAG_NUM_ANCHORS][3] = {
    {     0.0,    0.0,  300.0 }, { 10000.0,    0.0,  300.0 },
    { 10000.0, 8000.0,  300.0 }, {     0.0, 8000.0,  300.0 },
    {     0.0,    0.0, 2700.0 }, { 10000.0,    0.0, 2700.0 },
    { 10000.0, 8000.0, 2700.0 }, {     0.0, 8000.0, 2700.0 },
};

static double frame_tail_us(unsigned bytes_with_fcs)
{
    const unsigned bits = 8U * bytes_with_fcs;
    const unsigned blocks = (bits + 329U) / 330U;
    return PHR_US + (double)(bits + 48U * blocks) * CODED_BIT_US;
}

static double flight_us(double path_mm)
{
    return path_mm / (UWB_SPEED_OF_LIGHT * 1e-3);   /* mm / (mm per us) */
}

/* ========================================================================== */
/*                     SCENARIOS AND GATES                                    */
/* ========================================================================== */

typedef struct {
    double   min_availability;   /* valid published ranges / cycles the anchor was up */
    double   max_p95_err_mm;     /* p95 |published - truth| */
    double   max_abs_lag_ms;     /* |mean(truth - published)| / radial speed,
                                    at least LAG_FLOOR_MM of mean error */
    uint32_t max_false_accept;   /* valid ranges more than 300 mm from truth */
    double   max_recovery_ms;    /* disturbance end -> first valid within 150 mm */
    double   max_raw_bias_mm;    /* |mean(corrected raw - truth)|, at least 3 SE;
                                    10 mm: a calibration sign error is >= 24 mm */
} Gate;

typedef struct {
    const char *name;
    const char *what;
    uint8_t  anchors;            /* 1: A1 on a radial line; 8: room flight */
    double   duration_s;
    double   eval_from_s;
    double   d0_mm, v_mm_s;      /* radial trajectory */
    double   sine_amp_mm, sine_period_s;
    double   flight_omega;       /* room flight speed scale (rad/s), 0 = n/a */
    double   spike_prob, spike_mm;
    double   dropout_from_s, dropout_to_s;
    double   nlos_from_s, nlos_to_s, nlos_mm;
    double   tag_ppm, anchor_ppm;
    uint64_t clock_start_ticks;  /* both counters start here (near 2^40 = wrap) */
    double   fpp_1m_dbm;
    double   nrf_poll_latency_us;   /* POLL RXFCG -> TXDLYS, A1-A4 */
    double   stm32_poll_latency_us; /* POLL RXFCG -> TXDLYS, A5-A8 */
    Gate     gate;
    /* Optional NLOS burst (spike_mm with burst_prob inside the window). */
    double   burst_from_s, burst_to_s, burst_prob;
    /* 1: a documented weakness of the current conditioners. Its failure is
     * printed as KNOWN and does not fail a strict run; a pass prints XPASS
     * so the flag gets removed once the firmware is fixed. */
    uint8_t  known_issue;
} Scenario;

#define NO_BURST     -1.0, -1.0, 0.0, 0U
#define GATE_STATIC  { 0.95, 60.0, NO_GATE, 0U, NO_GATE, 10.0 }
#define GATE_MOVE(p) { 0.95, (p), 60.0, 0U, NO_GATE, 10.0 }

static const Scenario k_scenarios[] = {
    { "static_3m",  "hover, A1 at 3 m", 1, 10.0, 2.0,
      3000.0, 0.0, 0.0, 1.0, 0.0,  0.0, 0.0,  -1, -1,  -1, -1, 0.0,
      0.0, 0.0, 0ULL, -62.0, 350.0, 500.0, GATE_STATIC, NO_BURST },
    { "drift_0.1",  "radial 0.1 m/s", 1, 12.0, 4.0,
      3000.0, 100.0, 0.0, 1.0, 0.0,  0.0, 0.0,  -1, -1,  -1, -1, 0.0,
      0.0, 0.0, 0ULL, -62.0, 350.0, 500.0, GATE_MOVE(100.0), NO_BURST },
    { "walk_0.5",   "radial 0.5 m/s", 1, 12.0, 4.0,
      2000.0, 500.0, 0.0, 1.0, 0.0,  0.0, 0.0,  -1, -1,  -1, -1, 0.0,
      0.0, 0.0, 0ULL, -62.0, 350.0, 500.0, GATE_MOVE(120.0), NO_BURST },
    { "run_1.0",    "radial 1.0 m/s", 1, 10.0, 3.0,
      2000.0, 1000.0, 0.0, 1.0, 0.0,  0.0, 0.0,  -1, -1,  -1, -1, 0.0,
      0.0, 0.0, 0ULL, -62.0, 350.0, 500.0, GATE_MOVE(150.0), NO_BURST },
    { "fast_2.0",   "radial 2.0 m/s", 1, 6.0, 2.0,
      2000.0, 2000.0, 0.0, 1.0, 0.0,  0.0, 0.0,  -1, -1,  -1, -1, 0.0,
      0.0, 0.0, 0ULL, -62.0, 350.0, 500.0, GATE_MOVE(200.0), NO_BURST },
    { "sine_1m_4s", "4 m +/-1 m, 4 s period (peak 1.6 m/s)", 1, 12.0, 4.0,
      4000.0, 0.0, 1000.0, 4.0, 0.0,  0.0, 0.0,  -1, -1,  -1, -1, 0.0,
      0.0, 0.0, 0ULL, -62.0, 350.0, 500.0,
      { 0.95, 150.0, NO_GATE, 0U, NO_GATE, 10.0 }, NO_BURST },
    { "spikes_hold", "hover + 10% isolated +0.8 m NLOS", 1, 12.0, 2.0,
      3000.0, 0.0, 0.0, 1.0, 0.0,  0.10, 800.0,  -1, -1,  -1, -1, 0.0,
      0.0, 0.0, 0ULL, -62.0, 350.0, 500.0,
      { 0.85, 60.0, NO_GATE, 0U, NO_GATE, 10.0 }, NO_BURST },
    { "spikes_walk", "0.5 m/s + 10% isolated +0.8 m NLOS", 1, 12.0, 4.0,
      2000.0, 500.0, 0.0, 1.0, 0.0,  0.10, 800.0,  -1, -1,  -1, -1, 0.0,
      0.0, 0.0, 0ULL, -62.0, 350.0, 500.0,
      { 0.85, 150.0, NO_GATE, 0U, NO_GATE, 10.0 }, NO_BURST },
    { "dropout_1s", "1.0 m/s, A1 silent 1 s", 1, 10.0, 2.0,
      2000.0, 1000.0, 0.0, 1.0, 0.0,  0.0, 0.0,  5.0, 6.0,  -1, -1, 0.0,
      0.0, 0.0, 0ULL, -62.0, 350.0, 500.0,
      { 0.85, 150.0, NO_GATE, 0U, 200.0, 10.0 }, NO_BURST },
    { "nlos_3s",    "hover, +0.6 m NLOS for 3 s", 1, 12.0, 2.0,
      3000.0, 0.0, 0.0, 1.0, 0.0,  0.0, 0.0,  -1, -1,  5.0, 8.0, 600.0,
      0.0, 0.0, 0ULL, -62.0, 350.0, 500.0,
      { 0.85, 60.0, NO_GATE, 0U, 300.0, 10.0 }, NO_BURST },
    { "wrap_drift", "1.0 m/s, +/-20 ppm, 40-bit wrap at 1 s", 1, 6.0, 0.5,
      2000.0, 1000.0, 0.0, 1.0, 0.0,  0.0, 0.0,  -1, -1,  -1, -1, 0.0,
      -20.0, 20.0, TS40_MASK - (uint64_t)(1.0e6 * TICKS_PER_US),
      -62.0, 350.0, 500.0, GATE_MOVE(150.0), NO_BURST },
    /* Reacquisition must not lock onto NLOS: after the dropout 30% of the
     * exchanges carry +0.8 m for 0.3 s. Known issue: the C9 conditioners
     * restart from the first sample after a gap and reacquire from any three
     * rejected samples that agree, NLOS or not. The NLOS-aware reacquisition
     * in Plan/patches/ passes this gate; see
     * Plan/BAO_CAO_KIEM_TRA_FIRMWARE_V2_2026-09-24.md. */
    { "reacq_nlos", "1.0 m/s, A1 silent 1 s, 30% +0.8 m NLOS for 0.3 s after", 1, 10.0, 2.0,
      2000.0, 1000.0, 0.0, 1.0, 0.0,  0.0, 800.0,  5.0, 6.0,  -1, -1, 0.0,
      0.0, 0.0, 0ULL, -62.0, 350.0, 500.0,
      { 0.85, 150.0, NO_GATE, 0U, 300.0, 10.0 },
      6.0, 6.3, 0.30, 1U },
    { "flight_8",   "8 anchors, figure-8 up to 1.5 m/s, 2% NLOS", 8, 24.0, 3.0,
      0.0, 0.0, 0.0, 1.0, 0.3,  0.02, 800.0,  -1, -1,  -1, -1, 0.0,
      -5.0, 8.0, 0ULL, -58.0, 350.0, 500.0,
      { 0.90, 150.0, NO_GATE, 0U, NO_GATE, 10.0 }, NO_BURST },
    /* A weak but usable link must still start and restart. The conditioners
     * see FPP + UWB_FILTER_FPP_COMPAT_DB (-110 dBm here), below
     * reacquire_min_fpp_dbm, so no FPP gate may guard the first track.
     * Appended last: the seed of every scenario depends on its index. */
    { "weak_gap",   "hover 5 m, FPP -98 dBm, A1 silent 1 s", 1, 10.0, 2.0,
      5000.0, 0.0, 0.0, 1.0, 0.0,  0.0, 0.0,  5.0, 6.0,  -1, -1, 0.0,
      0.0, 0.0, 0ULL, -84.0, 350.0, 500.0,
      { 0.85, 60.0, NO_GATE, 0U, 300.0, 10.0 }, NO_BURST },
};

#define SCENARIO_COUNT (sizeof(k_scenarios) / sizeof(k_scenarios[0]))

/* ========================================================================== */
/*                     DETERMINISTIC RANDOMNESS                               */
/* ========================================================================== */

static uint64_t s_rng;

static uint64_t rng_next(void)
{
    uint64_t z = (s_rng += 0x9E3779B97F4A7C15ULL);             /* splitmix64 */
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

static double rng_uniform(void)
{
    return ((double)(rng_next() >> 11) + 0.5) * (1.0 / 9007199254740992.0);
}

static double rng_gauss(void)
{
    return sqrt(-2.0 * log(rng_uniform())) * cos(2.0 * PI_D * rng_uniform());
}

/* ========================================================================== */
/*                     CHANNEL STATE                                          */
/* ========================================================================== */

typedef enum { EV_TX_DONE = 0, EV_RX_FRAME = 1 } EventKind;

typedef struct {
    uint8_t   used;
    EventKind kind;
    double    t_us;
    uint64_t  ts;                       /* TX_TIME / RX_TIME, TAG clock */
    uint8_t   frame[UWB_FRAME_MAX_RX_LEN];
    uint16_t  len;                      /* without FCS */
    double    fpp_dbm;
    double    tag_latency_us;           /* TAG CPU time charged after an RX */
    uint8_t   is_resp;
} Event;

typedef struct {
    uint8_t  open;
    uint8_t  txn;
    uint8_t  spike, nlos;
    double   excess_mm;
    uint64_t a3_rm;                     /* anchor clock of the RESP RMARKER */
} Exchange;

typedef struct {
    uint8_t valid, spike, nlos;
    double  t_us, truth_mm;
} Truth;

typedef struct {
    uint32_t expected, valid, false_accept, late_tx;
    uint32_t n_err;
    double   err_signed_sum;
    double   raw_sum, raw_sq;
    uint32_t raw_n;
    double   recovery_ms;
    uint64_t spi_resp_to_final;
    uint32_t spi_samples;
} Metrics;

static const Scenario *s_sc;
static double   s_t0_us;
static Event    s_ev[MAX_EVENTS];
static Exchange s_ex[TAG_NUM_ANCHORS];
static Truth    s_truth[TAG_NUM_ANCHORS][TRUTH_RING];
static double   s_abs_err[MAX_SAMPLES];
static Metrics  s_m;
static uint32_t s_last_tx_count;
static uint32_t s_last_cycle;
static uint8_t  s_mac_seq;
static uint32_t s_spi_at_resp;
static uint8_t  s_resp_pending;
static double   s_recovery_from_us;

static double sc_time_s(double t_us)
{
    return (t_us - s_t0_us) * 1e-6;
}

static uint64_t clock_at(double t_us, double ppm)
{
    const double ticks = t_us * TICKS_PER_US * (1.0 + ppm * 1e-6);
    return ((uint64_t)llround(ticks) + s_sc->clock_start_ticks) & TS40_MASK;
}

static uint64_t tag_clock(double t_us)
{
    return clock_at(t_us - s_t0_us, s_sc->tag_ppm);
}

static uint64_t anchor_clock(double t_us)
{
    return clock_at(t_us - s_t0_us, s_sc->anchor_ppm);
}

static uint64_t add_noise(uint64_t ts)
{
    const int64_t n = (int64_t)llround(TS_NOISE_TICKS * rng_gauss());
    return (uint64_t)((int64_t)ts + n) & TS40_MASK;
}

static void flight_position(double t_s, double p[3])
{
    const double w = s_sc->flight_omega;
    p[0] = 5000.0 + 3000.0 * sin(w * t_s);
    p[1] = 4000.0 + 2000.0 * sin(2.0 * w * t_s);
    p[2] = 1500.0 + 300.0 * sin(0.5 * w * t_s);
}

static double true_distance_mm(uint8_t idx, double t_us)
{
    const double t = sc_time_s(t_us);

    if (s_sc->anchors == 1U)
    {
        double d = s_sc->d0_mm + s_sc->v_mm_s * t
                 + s_sc->sine_amp_mm * sin(2.0 * PI_D * t / s_sc->sine_period_s);
        return d < 300.0 ? 300.0 : d;
    }
    double p[3];
    flight_position(t, p);
    const double dx = p[0] - k_room_anchor_mm[idx][0];
    const double dy = p[1] - k_room_anchor_mm[idx][1];
    const double dz = p[2] - k_room_anchor_mm[idx][2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

static uint8_t in_window(double t_s, double from_s, double to_s)
{
    return (from_s >= 0.0 && t_s >= from_s && t_s < to_s) ? 1U : 0U;
}

static uint8_t anchor_up(uint8_t idx, double t_us)
{
    if (idx >= s_sc->anchors)
        return 0U;
    /* The single-anchor dropout silences A1; in the room it silences A1 too. */
    return (idx == 0U && in_window(sc_time_s(t_us), s_sc->dropout_from_s,
                                   s_sc->dropout_to_s)) ? 0U : 1U;
}

static double fpp_model_dbm(double distance_mm, uint8_t nlos)
{
    const double d_m = distance_mm < 500.0 ? 0.5 : distance_mm / 1000.0;
    return s_sc->fpp_1m_dbm - 20.0 * log10(d_m) - (nlos ? 8.0 : 0.0);
}

static double poll_latency_us(uint8_t idx)
{
    return idx < 4U ? s_sc->nrf_poll_latency_us : s_sc->stm32_poll_latency_us;
}

static double final_latency_us(uint8_t idx)
{
    return idx < 4U ? 300.0 : 450.0;
}

/* ========================================================================== */
/*                     EVENT QUEUE                                            */
/* ========================================================================== */

static Event *event_alloc(EventKind kind, double t_us)
{
    for (unsigned i = 0U; i < MAX_EVENTS; i++)
    {
        if (!s_ev[i].used)
        {
            memset(&s_ev[i], 0, sizeof(s_ev[i]));
            s_ev[i].used = 1U;
            s_ev[i].kind = kind;
            s_ev[i].t_us = t_us;
            return &s_ev[i];
        }
    }
    fprintf(stderr, "motion: event queue overflow\n");
    exit(2);
}

static Event *event_next(void)
{
    Event *best = NULL;
    for (unsigned i = 0U; i < MAX_EVENTS; i++)
    {
        if (s_ev[i].used && (best == NULL || s_ev[i].t_us < best->t_us))
            best = &s_ev[i];
    }
    return best;
}

/* Program the RX diagnostics so the TAG computes the requested FPP. */
static void set_rx_quality(double fpp_dbm)
{
    const double ratio = sqrt(pow(10.0, (fpp_dbm + (double)UWB_FPP_A_CONST) / 10.0) / 3.0);
    double n = 200.0;
    if (ratio * n > 60000.0)
        n = floor(60000.0 / ratio);
    if (n < 16.0)
        n = 16.0;
    double f = ratio * n;
    if (f > 65535.0)
        f = 65535.0;
    if (f < 1.0)
        f = 1.0;
    const double rx_dbm = fpp_dbm + 3.0;
    double c = n * n * pow(10.0, (rx_dbm + (double)UWB_FPP_A_CONST) / 10.0) / 131072.0;
    if (c > 65535.0)
        c = 65535.0;
    if (c < 1.0)
        c = 1.0;
    sim_set_rx_quality((uint16_t)n, (uint16_t)f, (uint16_t)f, (uint16_t)f,
                       (uint16_t)c, 40U);
}

/* ========================================================================== */
/*                     ANCHOR BEHAVIOUR                                        */
/* ========================================================================== */

static void anchor_on_poll(uint8_t idx, uint8_t txn, double t_tx_start)
{
    const double t1 = t_tx_start + TX_STARTUP_US + SHR_US;
    Event *tx = event_alloc(EV_TX_DONE, t1 + frame_tail_us(UWB_POLL_V2_RX_LEN));
    tx->ts = tag_clock(t1);

    if (!anchor_up(idx, t1))
        return;

    Exchange *ex = &s_ex[idx];
    memset(ex, 0, sizeof(*ex));
    ex->txn = txn;
    ex->nlos = in_window(sc_time_s(t1), s_sc->nlos_from_s, s_sc->nlos_to_s);
    const double spike_prob = in_window(sc_time_s(t1), s_sc->burst_from_s, s_sc->burst_to_s)
        ? s_sc->burst_prob : s_sc->spike_prob;
    ex->spike = (spike_prob > 0.0 && rng_uniform() < spike_prob) ? 1U : 0U;
    ex->excess_mm = (ex->nlos ? s_sc->nlos_mm : 0.0) + (ex->spike ? s_sc->spike_mm : 0.0);
    const double extra_mm = k_pair_bias_mm[idx] + ex->excess_mm;

    /* POLL arrives; the anchor schedules RESP at poll_rx + reply (anchor clock),
     * DX_TIME keeps bits [39:9], the RMARKER leaves one antenna delay later. */
    const double t2 = t1 + flight_us(true_distance_mm(idx, t1) + extra_mm);
    const uint64_t a2_true = anchor_clock(t2);
    const uint64_t a2 = add_noise(a2_true);
    const uint64_t dx_time = (a2 + REPLY_TICKS) & 0xFFFFFFFE00ULL;
    const uint64_t a3_rm = (dx_time + ANT_DLY_TICKS) & TS40_MASK;
    const uint32_t da = (uint32_t)((a3_rm - a2) & TS40_MASK);
    const double k_anchor = TICKS_PER_US * (1.0 + s_sc->anchor_ppm * 1e-6);
    const double t_dx = t2 + (double)((dx_time - a2_true) & TS40_MASK) / k_anchor;
    const double t3 = t2 + (double)((a3_rm - a2_true) & TS40_MASK) / k_anchor;

    /* Late delayed TX (HPDWARN/TXPUTE): the anchor aborts and stays silent. */
    const double t_cmd = t2 + frame_tail_us(UWB_POLL_V2_RX_LEN) + poll_latency_us(idx);
    if (t_cmd + TX_STARTUP_US + SHR_US > t_dx)
    {
        s_m.late_tx++;
        return;
    }

    const double t4 = t3 + flight_us(true_distance_mm(idx, t3) + extra_mm);
    uint8_t frame[UWB_FRAME_MAX_RX_LEN];
    const UwbResp_t resp = {
        .version = UWB_FRAME_V2, .txn = txn, .reply_ticks = da,
        .anchor_status = 0U, .tlv_len = 0U, .tlv = NULL,
    };
    const uint16_t len = uwb_frame_build_resp(frame, s_mac_seq++, DW_PAN_ID, TAG_ADDR,
                                              anchor_id_at(idx), &resp);
    Event *rx = event_alloc(EV_RX_FRAME, t4 + frame_tail_us(len + UWB_FRAME_FCS_LEN));
    memcpy(rx->frame, frame, len);
    rx->len = len;
    rx->ts = add_noise(tag_clock(t4));
    rx->fpp_dbm = fpp_model_dbm(true_distance_mm(idx, t4), ex->nlos || ex->spike);
    rx->tag_latency_us = MOTION_TAG_RESP_US;  /* read, parse, diag, build FINAL */
    rx->is_resp = 1U;

    ex->open = 1U;
    ex->a3_rm = a3_rm;
    Truth *truth = &s_truth[idx][txn];
    truth->valid = 1U;
    truth->t_us = t3;
    truth->truth_mm = true_distance_mm(idx, t3);
    truth->spike = ex->spike;
    truth->nlos = ex->nlos;
}

static void anchor_on_final(uint8_t idx, uint8_t txn, double t_tx_start)
{
    const double t5 = t_tx_start + TX_STARTUP_US + SHR_US;
    Event *tx = event_alloc(EV_TX_DONE, t5 + frame_tail_us(UWB_FINAL_V2_RX_LEN));
    tx->ts = tag_clock(t5);

    Exchange *ex = &s_ex[idx];
    if (!ex->open || ex->txn != txn || !anchor_up(idx, t5))
        return;
    ex->open = 0U;

    const double extra_mm = k_pair_bias_mm[idx] + ex->excess_mm;
    const double t6 = t5 + flight_us(true_distance_mm(idx, t5) + extra_mm);
    const uint64_t a6 = add_noise(anchor_clock(t6));
    const uint32_t rb = (uint32_t)((a6 - ex->a3_rm) & TS40_MASK);
    const double t_rep = t6 + frame_tail_us(UWB_FINAL_V2_RX_LEN) + final_latency_us(idx)
                       + TX_STARTUP_US + SHR_US;
    const double t_arr = t_rep + flight_us(true_distance_mm(idx, t_rep) + extra_mm);
    const double fpp = fpp_model_dbm(true_distance_mm(idx, t6), ex->nlos || ex->spike);

    uint8_t frame[UWB_FRAME_MAX_RX_LEN];
    const UwbReport_t rep = {
        .version = UWB_FRAME_V2, .txn = txn, .round_ticks = rb,
        .final_fp_cdbm = (int16_t)(fpp * 100.0), .final_rx_cdbm = (int16_t)((fpp + 3.0) * 100.0),
    };
    const uint16_t len = uwb_frame_build_report(frame, s_mac_seq++, DW_PAN_ID, TAG_ADDR,
                                                anchor_id_at(idx), &rep);
    Event *rx = event_alloc(EV_RX_FRAME, t_arr + frame_tail_us(len + UWB_FRAME_FCS_LEN));
    memcpy(rx->frame, frame, len);
    rx->len = len;
    rx->ts = tag_clock(t_arr);
    rx->fpp_dbm = fpp;
    rx->tag_latency_us = 100.0;          /* REPORT: DS maths + conditioner */
}

/* The TAG started a transmission: hand the frame to the addressed anchor. */
static void on_tag_tx(void)
{
    if (sim_tx_count == s_last_tx_count)
        return;
    s_last_tx_count = sim_tx_count;

    const uint8_t func = sim_tx_frame[9];
    const uint16_t dst = (uint16_t)(sim_tx_frame[5] | (sim_tx_frame[6] << 8));
    const uint8_t txn = sim_tx_frame[11];
    if (dst == 0U || dst > TAG_NUM_ANCHORS)
        return;
    const uint8_t idx = (uint8_t)(dst - 1U);

    if (func == FRAME_POLL_FUNC)
        anchor_on_poll(idx, txn, (double)sim_us);
    else if (func == FRAME_FINAL_FUNC)
    {
        if (s_resp_pending)
        {
            s_m.spi_resp_to_final += sim_spi_calls - s_spi_at_resp;
            s_m.spi_samples++;
            s_resp_pending = 0U;
        }
        anchor_on_final(idx, txn, (double)sim_us);
    }
}

/* ========================================================================== */
/*                     MEASUREMENT COLLECTION                                 */
/* ========================================================================== */

static uint8_t excluded(double t_s)
{
    return (in_window(t_s, s_sc->dropout_from_s, s_sc->dropout_to_s)
            || in_window(t_s, s_sc->nlos_from_s, s_sc->nlos_to_s)) ? 1U : 0U;
}

static void count_cycles(void)
{
    if (tag_cycle_count == s_last_cycle)
        return;
    s_last_cycle = tag_cycle_count;
    const double t_s = sc_time_s((double)sim_us);
    if (t_s < s_sc->eval_from_s || excluded(t_s))
        return;
    for (uint8_t i = 0U; i < s_sc->anchors; i++)
    {
        if (anchor_up(i, (double)sim_us))
            s_m.expected++;
    }
}

static void collect(void)
{
    TagMeasurement_t m;

    while (Tag_PopMeasurement(&m))
    {
        if (m.anchor_id == 0U || m.anchor_id > TAG_NUM_ANCHORS)
            continue;
        const uint8_t idx = (uint8_t)(m.anchor_id - 1U);
        Truth *truth = &s_truth[idx][m.txn];
        if (!truth->valid)
            continue;
        truth->valid = 0U;

        const double t_s = sc_time_s(truth->t_us);
        const uint8_t valid = (m.flags & TAG_MEAS_FLAG_FILTER_OK) != 0U;
        const double err = (double)m.filtered_mm - truth->truth_mm;

        const uint8_t far = (valid && fabs(err) > FALSE_ACCEPT_MM) ? 1U : 0U;

        /* An NLOS spike published as valid is an accepted outlier, whenever
         * it happens (also during a recovery transient). */
        if (far && truth->spike && t_s >= s_sc->eval_from_s)
            s_m.false_accept++;

        /* After a dropout or an NLOS episode the transient is judged by the
         * recovery gate: until the first valid range within 150 mm, other
         * samples are not scored. */
        if (s_recovery_from_us > 0.0 && truth->t_us >= s_recovery_from_us
            && s_m.recovery_ms < 0.0)
        {
            if (valid && fabs(err) <= RECOVERED_MM)
                s_m.recovery_ms = (truth->t_us - s_recovery_from_us) * 1e-3;
            else
                continue;
        }
        /* NLOS-affected exchanges carry a bias the TAG cannot observe. */
        if (t_s < s_sc->eval_from_s || excluded(t_s) || truth->nlos)
            continue;

        if ((m.flags & TAG_MEAS_FLAG_CAL_OK) != 0U && !truth->spike)
        {
            const double raw_err = (double)m.corrected_mm - truth->truth_mm;
            s_m.raw_sum += raw_err;
            s_m.raw_sq += raw_err * raw_err;
            s_m.raw_n++;
        }
        if (!valid)
            continue;
        s_m.valid++;
        s_m.err_signed_sum += truth->truth_mm - (double)m.filtered_mm;
        if (s_m.n_err < MAX_SAMPLES)
            s_abs_err[s_m.n_err++] = fabs(err);
        if (far && !truth->spike)
            s_m.false_accept++;          /* stale or lagging published range */
    }
}

/* ========================================================================== */
/*                     SCENARIO RUNNER                                        */
/* ========================================================================== */

static void scenario_reset(const Scenario *sc, unsigned scenario_index, unsigned seed)
{
    s_sc = sc;
    sim_reset();
    if (Tag_Init() != 0)
    {
        fprintf(stderr, "motion: Tag_Init failed\n");
        exit(2);
    }
    memset(s_track, 0, sizeof(s_track));
    /* Tag_Init runs once at boot and keeps the slot pointer, the transaction
     * and MAC counters and the queue; restart them so that every scenario
     * starts from the same state whatever ran before it. */
    discard_pending_measurements();
    s_current_anchor = 0U;
    s_txn = 0U;
    s_seq_num = 0U;
    s_slot_rx_error = 0U;
    s_cycle_probe_idx = -1;
    s_info_request_idx = -1;
    s_info_rr = 0U;
    s_mac_seq = 0U;
    memset(s_ev, 0, sizeof(s_ev));
    memset(s_ex, 0, sizeof(s_ex));
    memset(s_truth, 0, sizeof(s_truth));
    memset(&s_m, 0, sizeof(s_m));
    memset(&tag_rx_error_stats, 0, sizeof(tag_rx_error_stats));
    s_m.recovery_ms = -1.0;
    for (uint8_t i = 0U; i < TAG_NUM_ANCHORS; i++)
    {
        reset_anchor_filter(i);          /* Tag_Init keeps Legacy filter memory */
        anchor_response_timeout_streak[i] = 0U;
        anchor_ds_incomplete_streak[i] = 0U;
        s_anchor_next_probe_cycle[i] = 0U;
        anchor_slot_duration_max_us[i] = 0U;
        (void)Tag_SetDsCalibration((uint16_t)(i + 1U),
                                   (int32_t)llround(k_pair_bias_mm[i] * 1000.0), 1U);
    }
    tag_cycle_duration_max_us = 0U;
    cycle_overrun_count = 0U;
    Tag_SetActiveAnchorMask((uint8_t)((1U << sc->anchors) - 1U));
    Tag_RequestPause(0U);
    Tag_EnableMeasurementQueue(1U);
    tag_cycle_count = 1U;
    s_last_cycle = tag_cycle_count;
    s_last_tx_count = sim_tx_count;
    s_resp_pending = 0U;
    s_rng = 0x5EEDULL + 7919ULL * (uint64_t)scenario_index + 104729ULL * (uint64_t)seed;
    s_t0_us = (double)sim_us;
    s_recovery_from_us = -1.0;
    if (sc->dropout_to_s > 0.0)
        s_recovery_from_us = s_t0_us + sc->dropout_to_s * 1e6;
    else if (sc->nlos_to_s > 0.0)
        s_recovery_from_us = s_t0_us + sc->nlos_to_s * 1e6;
}

static void step_tag(void)
{
    Tag_Task();
    on_tag_tx();
    count_cycles();
    collect();
}

static void run_scenario(const Scenario *sc, unsigned scenario_index, unsigned seed)
{
    scenario_reset(sc, scenario_index, seed);
    const double t_end = s_t0_us + sc->duration_s * 1e6;

    while ((double)sim_us < t_end)
    {
        Event *ev = event_next();
        if (ev != NULL && ev->t_us <= (double)sim_us)
        {
            const Event e = *ev;
            ev->used = 0U;
            if (e.kind == EV_TX_DONE)
            {
                sim_complete_tx(e.ts);
            }
            else
            {
                set_rx_quality(e.fpp_dbm);
                sim_deliver_rx(e.frame, e.len, e.ts, 0U);
                if (e.is_resp)
                {
                    s_spi_at_resp = sim_spi_calls;
                    s_resp_pending = 1U;
                }
                /* The TAG CPU needs this long before it acts on the frame. */
                sim_us += (uint64_t)llround(e.tag_latency_us);
            }
            step_tag();
            continue;
        }

        step_tag();
        const double next = ev != NULL ? ev->t_us : 1e300;
        const uint64_t step_to = sim_us + 5U;
        sim_us = (next < (double)step_to) ? (uint64_t)ceil(next) : step_to;
    }
}

/* ========================================================================== */
/*                     REPORT                                                 */
/* ========================================================================== */

static int cmp_double(const void *a, const void *b)
{
    const double x = *(const double *)a;
    const double y = *(const double *)b;
    return (x > y) - (x < y);
}

static double percentile(double p)
{
    if (s_m.n_err == 0U)
        return NAN;
    qsort(s_abs_err, s_m.n_err, sizeof(s_abs_err[0]), cmp_double);
    size_t k = (size_t)ceil(p * (double)s_m.n_err) - 1U;
    if (k >= s_m.n_err)
        k = s_m.n_err - 1U;
    return s_abs_err[k];
}

static const char *filter_name(void)
{
#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_MEDIAN_GATE
    return "MEDIAN_GATE (UWB_RANGE_FILTER_MODE=1)";
#elif UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_CV_KALMAN_V2
    return "CV_KALMAN_V2 (UWB_RANGE_FILTER_MODE=2)";
#elif UWB_C9_2_MOTION_MODE == UWB_C9_2_MOTION_ACTIVE
    return "LEGACY + C9.2 motion ACTIVE";
#elif UWB_LEGACY_ADAPTIVE_MODE == UWB_LEGACY_ADAPTIVE_ACTIVE
    return "LEGACY + C9.1 adaptive ACTIVE";
#else
    return "LEGACY median-3 + scalar Kalman (build default)";
#endif
}

/* Not gated: CPU time an STM32 anchor may spend between the POLL interrupt
 * and the delayed-TX command before its RESP is late (HPDWARN/TXPUTE). */
static void stm32_latency_sensitivity(void)
{
    static const double latency_us[] = { 500.0, 800.0, 900.0, 950.0 };
    const Scenario *flight = NULL;

    for (unsigned i = 0U; i < SCENARIO_COUNT; i++)
        if (k_scenarios[i].anchors == TAG_NUM_ANCHORS)
            flight = &k_scenarios[i];
    if (flight == NULL)
        return;

    printf("STM32 POLL->TXDLYS latency (A5-A8), flight_8, not gated:\n");
    for (unsigned k = 0U; k < sizeof(latency_us) / sizeof(latency_us[0]); k++)
    {
        Scenario sc = *flight;
        sc.stm32_poll_latency_us = latency_us[k];
        run_scenario(&sc, 100U + k, 0U);
        printf("  %4.0f us -> availability %5.1f%%, late RESP %u\n", latency_us[k],
               s_m.expected ? 100.0 * (double)s_m.valid / (double)s_m.expected : 0.0,
               (unsigned)s_m.late_tx);
    }
}

typedef struct {
    double availability, mean_abs, p95, max_abs, lag_ms, raw_bias, raw_sd;
    double recovery_ms;
    uint32_t false_accept, late_tx;
    char why[160];               /* failed gates, "" when within gate */
} Outcome;

/* Judge the scenario that just ran against its gate. */
static void evaluate(const Scenario *sc, Outcome *o)
{
    const Gate *g = &sc->gate;
    size_t used = 0U;

    memset(o, 0, sizeof(*o));
    o->availability = s_m.expected ? (double)s_m.valid / (double)s_m.expected : 0.0;
    for (uint32_t k = 0U; k < s_m.n_err; k++)
    {
        o->mean_abs += s_abs_err[k];
        if (s_abs_err[k] > o->max_abs)
            o->max_abs = s_abs_err[k];
    }
    o->mean_abs = s_m.n_err ? o->mean_abs / (double)s_m.n_err : NAN;
    o->p95 = percentile(0.95);
    o->lag_ms = (sc->anchors == 1U && sc->sine_amp_mm == 0.0
                 && sc->v_mm_s != 0.0 && s_m.valid != 0U)
        ? (s_m.err_signed_sum / (double)s_m.valid) / sc->v_mm_s * 1000.0 : NAN;
    o->raw_bias = s_m.raw_n ? s_m.raw_sum / (double)s_m.raw_n : NAN;
    o->raw_sd = s_m.raw_n > 1U
        ? sqrt(fmax(0.0, s_m.raw_sq / (double)s_m.raw_n - o->raw_bias * o->raw_bias)) : NAN;
    o->recovery_ms = s_m.recovery_ms;
    o->false_accept = s_m.false_accept;
    o->late_tx = s_m.late_tx;

#define FAIL_IF(cond, text) do { if (cond) used += (size_t)snprintf(o->why + used, \
        sizeof(o->why) - used, "%s%s", used ? "," : "", text); } while (0)
    FAIL_IF(o->availability < g->min_availability, "avail");
    FAIL_IF(!(o->p95 <= g->max_p95_err_mm), "p95");
    if (g->max_abs_lag_ms >= 0.0 && isfinite(o->lag_ms))
    {
        const double lag_limit_ms = fmax(g->max_abs_lag_ms,
                                         LAG_FLOOR_MM / fabs(sc->v_mm_s) * 1000.0);
        FAIL_IF(fabs(o->lag_ms) > lag_limit_ms, "lag");
    }
    FAIL_IF(o->false_accept > g->max_false_accept, "false-accept");
    FAIL_IF(g->max_recovery_ms >= 0.0
            && (o->recovery_ms < 0.0 || o->recovery_ms > g->max_recovery_ms), "recovery");
    /* The mean of raw_n noisy ranges has a standard error sd/sqrt(n);
     * the bias gate never asks for more than three of those. */
    double bias_limit = g->max_raw_bias_mm;
    if (s_m.raw_n > 1U)
        bias_limit = fmax(bias_limit, 3.0 * o->raw_sd / sqrt((double)s_m.raw_n));
    FAIL_IF(!(fabs(o->raw_bias) <= bias_limit), "raw-bias");
#undef FAIL_IF
}

static const char *verdict_text(const Scenario *sc, uint8_t failed_gate)
{
    if (sc->known_issue)
        return failed_gate ? "KNOWN (" : "XPASS, drop known_issue";
    return failed_gate ? "FAIL (" : "pass";
}

static void print_timing(const Scenario *sc)
{
    const double window_s = sc->duration_s - sc->eval_from_s;
    uint32_t slot_max = 0U;

    for (uint8_t a = 0U; a < TAG_NUM_ANCHORS; a++)
        if (anchor_slot_duration_max_us[a] > slot_max)
            slot_max = anchor_slot_duration_max_us[a];
    printf("             timing: %.1f cycles/s (%.0f ms/cycle), longest cycle %lu us,"
           " overruns %lu, longest slot %lu us\n",
           (double)s_m.expected / (double)sc->anchors / window_s,
           1000.0 * window_s * (double)sc->anchors / (double)s_m.expected,
           (unsigned long)tag_cycle_duration_max_us,
           (unsigned long)cycle_overrun_count, (unsigned long)slot_max);
}

int main(int argc, char **argv)
{
    int report_only = 0;
    unsigned seeds = 1U;
    unsigned failed = 0U;
    unsigned known = 0U;
    uint64_t spi_sum = 0U;
    uint32_t spi_n = 0U;

    for (int a = 1; a < argc; a++)
    {
        if (strcmp(argv[a], "--report") == 0)
            report_only = 1;
        else if (strcmp(argv[a], "--seeds") == 0 && a + 1 < argc)
            seeds = (unsigned)strtoul(argv[++a], NULL, 10);
        else
        {
            fprintf(stderr, "usage: test_tag_motion [--report] [--seeds N]\n");
            return 2;
        }
    }
    if (seeds == 0U)
        seeds = 1U;

    printf("motion test: %s, FPP compat %.2f dB\n", filter_name(),
           (double)UWB_FILTER_FPP_COMPAT_DB);
    printf("model: preamble %u, SHR %.1f us, POLL/RESP/FINAL/REPORT %.1f/%.1f/%.1f/%.1f us,"
           " reply %lu UUS, anchor TXDLYS budget %.0f us after POLL RXFCG\n",
           (unsigned)MOTION_PREAMBLE_SYMBOLS, SHR_US,
           SHR_US + frame_tail_us(UWB_POLL_V2_RX_LEN),
           SHR_US + frame_tail_us(UWB_RESP_V2_MIN_RX_LEN),
           SHR_US + frame_tail_us(UWB_FINAL_V2_RX_LEN),
           SHR_US + frame_tail_us(UWB_REPORT_V2_RX_LEN),
           (unsigned long)MOTION_REPLY_DELAY_UUS,
           (double)REPLY_TICKS / TICKS_PER_US - frame_tail_us(UWB_POLL_V2_RX_LEN)
               - TX_STARTUP_US - SHR_US);
    for (unsigned i = 0U; i < SCENARIO_COUNT; i++)
        printf("  %-12s %s\n", k_scenarios[i].name, k_scenarios[i].what);
    if (seeds == 1U)
        printf("%-12s %6s %6s %6s %7s %7s %4s %7s %9s %5s  %s\n",
               "scenario", "avail", "mean", "p95", "max", "lag_ms", "FA", "rec_ms",
               "raw_b/sd", "late", "verdict");
    else
        printf("%u seeds; worst value over the seeds, FA summed:\n"
               "%-12s %6s %6s %6s %7s %7s %4s %7s %6s %5s  %s\n", seeds,
               "scenario", "missed", "avail", "p95", "max", "|lag|", "FA", "rec_ms",
               "|raw_b|", "late", "verdict");

    for (unsigned i = 0U; i < SCENARIO_COUNT; i++)
    {
        const Scenario *sc = &k_scenarios[i];
        Outcome o, worst;
        unsigned missed = 0U;
        char why_all[160] = "";

        memset(&worst, 0, sizeof(worst));
        worst.availability = 1.0;
        for (unsigned seed = 0U; seed < seeds; seed++)
        {
            run_scenario(sc, i, seed);
            evaluate(sc, &o);
            spi_sum += s_m.spi_resp_to_final;
            spi_n += s_m.spi_samples;
            if (o.why[0] != '\0')
            {
                missed++;
                /* Union of the gates missed in any seed, in gate order. */
                static const char *const k_gate_names[] = {
                    "avail", "p95", "lag", "false-accept", "recovery", "raw-bias" };
                char merged[160] = "";
                size_t used = 0U;
                for (unsigned k = 0U; k < sizeof(k_gate_names) / sizeof(k_gate_names[0]); k++)
                    if (strstr(o.why, k_gate_names[k]) || strstr(why_all, k_gate_names[k]))
                        used += (size_t)snprintf(merged + used, sizeof(merged) - used, "%s%s",
                                                 used ? "," : "", k_gate_names[k]);
                memcpy(why_all, merged, sizeof(why_all));
            }
            if (seeds == 1U)
            {
                char lag_text[16], rec_text[16];
                if (isfinite(o.lag_ms))
                    snprintf(lag_text, sizeof(lag_text), "%7.0f", o.lag_ms);
                else
                    snprintf(lag_text, sizeof(lag_text), "%7s", "-");
                if (sc->gate.max_recovery_ms >= 0.0)
                    snprintf(rec_text, sizeof(rec_text), "%7.0f", o.recovery_ms);
                else
                    snprintf(rec_text, sizeof(rec_text), "%7s", "-");
                printf("%-12s %5.1f%% %6.0f %6.0f %7.0f %s %4u %s %4.0f/%-4.0f %5u  %s%s%s\n",
                       sc->name, 100.0 * o.availability, o.mean_abs, o.p95, o.max_abs,
                       lag_text, (unsigned)o.false_accept, rec_text, o.raw_bias, o.raw_sd,
                       (unsigned)o.late_tx, verdict_text(sc, o.why[0] != '\0'), o.why,
                       o.why[0] != '\0' ? ")" : "");
                if (sc->anchors == TAG_NUM_ANCHORS)
                    print_timing(sc);
                continue;
            }
            worst.availability = fmin(worst.availability, o.availability);
            worst.p95 = fmax(worst.p95, o.p95);
            worst.max_abs = fmax(worst.max_abs, o.max_abs);
            if (isfinite(o.lag_ms))
                worst.lag_ms = fmax(worst.lag_ms, fabs(o.lag_ms));
            worst.false_accept += o.false_accept;
            worst.recovery_ms = (o.recovery_ms < 0.0 || worst.recovery_ms < 0.0)
                ? -1.0 : fmax(worst.recovery_ms, o.recovery_ms);
            worst.raw_bias = fmax(worst.raw_bias, fabs(o.raw_bias));
            worst.late_tx += o.late_tx;
        }
        if (sc->known_issue)
            known += missed ? 1U : 0U;
        else if (missed != 0U)
            failed++;
        if (seeds == 1U)
            continue;

        char missed_text[16], lag_text[16], rec_text[16];
        snprintf(missed_text, sizeof(missed_text), "%u/%u", missed, seeds);
        if (sc->anchors == 1U && sc->sine_amp_mm == 0.0 && sc->v_mm_s != 0.0)
            snprintf(lag_text, sizeof(lag_text), "%7.0f", worst.lag_ms);
        else
            snprintf(lag_text, sizeof(lag_text), "%7s", "-");
        if (sc->gate.max_recovery_ms >= 0.0)
            snprintf(rec_text, sizeof(rec_text), "%7.0f", worst.recovery_ms);
        else
            snprintf(rec_text, sizeof(rec_text), "%7s", "-");
        printf("%-12s %6s %5.1f%% %6.0f %7.0f %s %4u %s %6.1f %5u  %s%s%s\n",
               sc->name, missed_text, 100.0 * worst.availability, worst.p95, worst.max_abs,
               lag_text, (unsigned)worst.false_accept, rec_text, worst.raw_bias,
               (unsigned)worst.late_tx, verdict_text(sc, missed != 0U), why_all,
               missed != 0U ? ")" : "");
    }

    stm32_latency_sensitivity();
    printf("TAG SPI transactions from RESP RXFCG to FINAL TXSTRT: %.1f (critical Db path)\n",
           spi_n ? (double)spi_sum / (double)spi_n : 0.0);
    printf("motion scenarios: %u/%u within gate, %u known issue(s)%s\n",
           (unsigned)(SCENARIO_COUNT - failed - known), (unsigned)SCENARIO_COUNT, known,
           report_only && failed ? " (report only, not enforced)" : "");
    return (failed != 0U && !report_only) ? 1 : 0;
}

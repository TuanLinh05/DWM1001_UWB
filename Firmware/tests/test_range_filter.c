/*
 * Unit tests of the C9 range conditioner (common/src/filters/range_filter.c)
 * on its own, without the TAG state machine. Built once per mode:
 *   -DUWB_RANGE_FILTER_MODE=1U  MEDIAN_GATE
 *   -DUWB_RANGE_FILTER_MODE=2U  CV_KALMAN_V2
 *
 * The rules under test are the reacquisition rules: a track starts or
 * restarts from the SHORTEST group of recent samples that agree (NLOS only
 * lengthens a range), a group longer than the current track needs more
 * evidence, rejected line-of-sight samples survive accepted NLOS samples,
 * and no FPP threshold may keep a weak but usable link from starting.
 * test_tag_motion.c checks the same rules end to end (reacq_nlos, weak_gap).
 */
#include <stdio.h>
#include <stdlib.h>

#include "range_filter.h"

#if UWB_RANGE_FILTER_MODE == UWB_RANGE_FILTER_LEGACY_KALMAN
#error "build with -DUWB_RANGE_FILTER_MODE=1U (MEDIAN_GATE) or 2U (CV_KALMAN_V2)"
#endif

#define STRONG_FPP  (-70.0f)
#define WEAK_FPP    (-115.0f)   /* below reacquire_min_fpp_dbm (-105) */
#define LOS_MM      3000
#define NLOS_MM     3800        /* +0.8 m excess path */
#define STEP_MS     20U
#define NLOS_LIMIT  3300        /* a published range above this is NLOS */

static int s_failures;

#define CHECK(cond, what) do {                                              \
        if (!(cond)) {                                                      \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, (what));         \
            s_failures++;                                                   \
        }                                                                   \
    } while (0)

static RangeFilterState_t s_state;
static uint32_t s_now_ms;
static unsigned s_published_nlos;   /* valid outputs above NLOS_LIMIT */

static RangeFilterOutput_t feed_at(int32_t mm, float fpp, uint32_t now_ms)
{
    const RangeFilterInput_t in = {
        .corrected_raw_mm = mm, .fpp_dbm = fpp, .now_ms = now_ms, .radio_status = 0U,
    };
    s_now_ms = now_ms;
    const RangeFilterOutput_t out = RangeFilter_Update(&s_state, &in, &g_range_filter_config);
    if (out.publish_valid && out.filtered_mm > NLOS_LIMIT)
        s_published_nlos++;
    return out;
}

static RangeFilterOutput_t feed(int32_t mm, float fpp)
{
    return feed_at(mm, fpp, s_now_ms + STEP_MS);
}

static void reset(void)
{
    RangeFilter_Init(&s_state);
    s_now_ms = 1000U;
    s_published_nlos = 0U;
}

/* Feed until the track publishes; returns the first published range or -1. */
static int32_t first_publish(const int32_t *mm, unsigned n, float fpp, unsigned *index)
{
    for (unsigned i = 0U; i < n; i++)
    {
        const RangeFilterOutput_t out = feed(mm[i], fpp);
        if (out.publish_valid)
        {
            *index = i;
            return out.filtered_mm;
        }
    }
    return -1;
}

/* A fresh track waits for reacquire_min_samples + 1 samples and starts from
 * the mean of the shortest group. */
static void test_boot_starts_from_group(void)
{
    static const int32_t seq[] = { 3010, 2990, 3000, 3004, 2996, 3002 };
    unsigned index = 0U;

    reset();
    const int32_t start = first_publish(seq, 6U, STRONG_FPP, &index);
    CHECK(index == g_range_filter_config.reacquire_min_samples,
          "first publish after reacquire_min_samples + 1 samples");
    CHECK(start >= 2990 && start <= 3004, "start is the mean of the shortest group");
}

/* A weak but usable link (FPP below reacquire_min_fpp_dbm) must still start. */
static void test_boot_weak_signal(void)
{
    static const int32_t seq[] = { 5010, 4990, 5000, 5004, 4996, 5002 };
    unsigned index = 0U;

    reset();
    const int32_t start = first_publish(seq, 6U, WEAK_FPP, &index);
    CHECK(start > 0, "a weak link starts");
    CHECK(start >= 4990 && start <= 5010, "weak start value");
}

/* After a gap longer than stale_reset_ms the first samples are NLOS: they
 * must not become the new track while shorter samples disagree. */
static void test_gap_nlos_first(void)
{
    static const int32_t after_gap[] = { NLOS_MM, NLOS_MM + 10, LOS_MM + 5, LOS_MM, LOS_MM - 5,
                                         LOS_MM + 3, LOS_MM };
    unsigned index = 0U;

    reset();
    for (unsigned i = 0U; i < 12U; i++)
        (void)feed(LOS_MM + (int32_t)(i % 3U) - 1, STRONG_FPP);
    s_now_ms += g_range_filter_config.stale_reset_ms + 100U;     /* anchor silent */
    const int32_t start = first_publish(after_gap, 7U, STRONG_FPP, &index);
    CHECK(start > 0 && start < NLOS_LIMIT, "restart from the line-of-sight group");
    CHECK(s_published_nlos == 0U, "no NLOS range published after the gap");
}

/* While tracking, rejected NLOS samples that agree with each other must not
 * reacquire the track on the NLOS level after three of them. */
static void test_longer_group_needs_more_evidence(void)
{
    reset();
    for (unsigned i = 0U; i < 15U; i++)
        (void)feed(LOS_MM + (int32_t)(i % 3U) - 1, STRONG_FPP);
    for (unsigned i = 0U; i < 4U; i++)
        (void)feed(NLOS_MM + (int32_t)i * 5, STRONG_FPP);
    for (unsigned i = 0U; i < 6U; i++)
        (void)feed(LOS_MM, STRONG_FPP);
    CHECK(s_published_nlos == 0U, "a short NLOS burst is never published as valid");
}

/* The track started on an NLOS level (every sample after boot was NLOS).
 * Line-of-sight samples, mixed with NLOS ones, must win the track back. */
static void test_nlos_escape(void)
{
    static const int32_t mixed[] = { LOS_MM, LOS_MM + 4, NLOS_MM, LOS_MM - 3, LOS_MM + 1,
                                     NLOS_MM, LOS_MM, LOS_MM - 2, NLOS_MM, LOS_MM + 2,
                                     LOS_MM, NLOS_MM, LOS_MM - 1, LOS_MM };
    RangeFilterOutput_t out;

    reset();
    for (unsigned i = 0U; i < 6U; i++)
        (void)feed(NLOS_MM + (int32_t)(i % 3U), STRONG_FPP);
    for (unsigned i = 0U; i < sizeof(mixed) / sizeof(mixed[0]); i++)
        out = feed(mixed[i], STRONG_FPP);
    out = feed(LOS_MM, STRONG_FPP);
    CHECK(out.publish_valid && out.filtered_mm < NLOS_LIMIT,
          "line-of-sight samples reacquire the track");
}

/* An unusable input clears the candidate group: reacquisition needs
 * consecutive usable samples. */
static void test_invalid_input_clears_group(void)
{
    reset();
    (void)feed(3000, STRONG_FPP);
    (void)feed(3004, STRONG_FPP);
    (void)feed(0, STRONG_FPP);                   /* below min_range_mm */
    CHECK(s_state.candidate_count == 0U, "invalid input clears the candidates");
    const RangeFilterOutput_t out = feed(3002, STRONG_FPP);
    CHECK(!out.publish_valid, "no start from samples before the invalid input");
}

/* Candidates older than stale_reset_ms do not join a group. */
static void test_candidates_expire(void)
{
    reset();
    (void)feed(3000, STRONG_FPP);
    (void)feed(3004, STRONG_FPP);
    (void)feed(2998, STRONG_FPP);
    s_now_ms += g_range_filter_config.stale_reset_ms + 1U;
    const RangeFilterOutput_t out = feed(3002, STRONG_FPP);
    CHECK(!out.publish_valid, "expired candidates do not complete a group");
    CHECK(s_state.candidate_count == 1U, "only the new candidate is kept");
}

int main(void)
{
    test_boot_starts_from_group();
    test_boot_weak_signal();
    test_gap_nlos_first();
    test_longer_group_needs_more_evidence();
    test_nlos_escape();
    test_invalid_input_clears_group();
    test_candidates_expire();

    if (s_failures != 0)
    {
        printf("range filter tests FAILED (%d)\n", s_failures);
        return 1;
    }
    printf("range filter tests passed (mode %u: boot group, weak start, gap NLOS, "
           "longer group, NLOS escape, invalid input, expiry)\n",
           (unsigned)UWB_RANGE_FILTER_MODE);
    return 0;
}

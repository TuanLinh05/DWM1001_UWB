/*
 * Build configuration for test_tag_motion.c.
 *
 * The motion test must exercise the real TAG DevKit profile, so this header
 * includes that node's uwb_app_config.h unchanged. Tag_DevKit pins
 * UWB_LEGACY_ADAPTIVE_MODE to OFF without an #ifndef, so a candidate build
 * that evaluates the C9.1 Adaptive Legacy tracker overrides it here through
 * MOTION_LEGACY_ADAPTIVE_MODE instead of editing the production header.
 * Every other conditioner switch (UWB_RANGE_FILTER_MODE, UWB_C9_2_MOTION_MODE,
 * UWB_FILTER_FPP_COMPAT_DB) is already #ifndef-guarded and is passed with -D.
 * C9.1 and C9.2 are Legacy-path features: build them with
 * -DUWB_RANGE_FILTER_MODE=0U too, since the DevKit default is MEDIAN_GATE.
 */
#ifndef MOTION_TEST_APP_CONFIG_H
#define MOTION_TEST_APP_CONFIG_H

#include "../../Tag_DevKit/include/uwb_app_config.h"

#ifdef MOTION_LEGACY_ADAPTIVE_MODE
#undef UWB_LEGACY_ADAPTIVE_MODE
#define UWB_LEGACY_ADAPTIVE_MODE MOTION_LEGACY_ADAPTIVE_MODE
#endif

#endif /* MOTION_TEST_APP_CONFIG_H */

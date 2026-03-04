/**
 * @file test_main.c
 * @brief Master test runner for the Phoenix Avionics EKF test suite.
 *
 * Build:  cmake --build build_host --target run_ekf_tests_bin
 * Run:    cmake --build build_host --target run_ekf_tests
 *
 * Suite layout:
 *   test_ekf_core.c        — init, reset, covariance math
 *   test_ekf_fusion.c      — stationary bias convergence, GPS + Baro + Mag fusion
 *   test_ekf_dynamic.c     — 1D launch profile, high-vibration motor burn
 *   test_ekf_faults.c      — outlier rejection, GPS dropout, NaN guards
 *   test_ekf_state.c       — 5-stage warmup state machine transitions
 *   test_mission_profile.c — 60-second scripted pad-to-apogee mission timeline
 */

#include <stdio.h>
#include "test_utils.h"

int main(void)
{
    printf("\n╔══════════════════════════════════════════════════╗\n");
    printf(  "║  Phoenix Avionics — EKF Test Suite               ║\n");
    printf(  "╚══════════════════════════════════════════════════╝\n");

    run_core_tests();
    run_fusion_tests();
    run_dynamic_tests();
    run_fault_tests();
    run_state_tests();
    run_mission_tests();

    printf("\n══════════════════════════════════════════════════════\n");
    printf("  %d / %d tests passed.\n", g_tests_passed, g_tests_run);
    printf("══════════════════════════════════════════════════════\n\n");

    return (g_tests_passed == g_tests_run) ? 0 : 1;
}

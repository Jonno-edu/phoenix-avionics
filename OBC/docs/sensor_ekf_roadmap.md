# Sensor + EKF Implementation Roadmap

*Last updated: 5 March 2026*

---

## Architecture Overview

Phoenix Avionics implements a PX4-inspired dual-path IMU architecture split across two distinct data paths, and utilizes a **Delayed Horizon (Time Travel)** architecture to correctly fuse asynchronous, delayed sensor data.

### 1. The Estimator Path (Delayed Horizon)
To handle the reality of sensor processing latency (e.g., GPS taking 150ms+ to compute a fix), the EKF does not run in the present. 
* **The Delayed State (Source of Truth):** The heavy 15-state Error-State Kalman Filter (ESKF) lives strictly ~200ms in the past. When delayed GPS/Baro measurements arrive, they are placed in a "Waiting Room" until the delayed EKF's timeline naturally catches up to their exact timestamp. 
* **The Head State (Navigation Output):** To fly the rocket, the flight controller needs the state *right now*. The estimator maintains a 128-sample ring buffer of IMU history. Every 4ms, it copies the fully-fused past state and rapidly integrates the ring buffer forward using lightweight non-linear kinematics to produce a real-time `head_state`.

### 2. The Rate Path (Consumer - Deferred)
Raw gyro data has static calibration AND the EKF's live bias estimate subtracted from it, then is passed through a low-pass filter and published as `vehicle_angular_velocity`. The future rate controller receives perfectly zeroed, debiased angular rates in real time. **This path (`angvel.c`) is deliberately deferred** — it is not needed until a rate controller exists.

```text
Raw IMU (TOPIC_SENSOR_IMU, 250 Hz)
  │
  ├──→ Integrator → TOPIC_VEHICLE_IMU (250 Hz)
  │         │
  │         ├──→ [ Ring Buffer ] ──(Fast Forward)──→ Head State (Present) ──→ TOPIC_VEHICLE_STATE
  │         │
  │         └──→ (Wait 200ms) ──→ Delayed EKF (Past) ←── GPS/Baro/Mag (Waiting Rooms)
  │                                    │
  │                    TOPIC_ESTIMATOR_SENSOR_BIAS (bias feedback)
  │                                    │
  └──→ Rate Conditioner ←──────────────┘
            │  (deferred — no rate controller yet)
            └──→ TOPIC_VEHICLE_ANGULAR_VELOCITY (250 Hz) ──→ Rate Controller (future)

```

---

## Roadmap Status Summary

| Phase | Title | Status |
| --- | --- | --- |
| 1 | Foundation: Topics, Build, BSP Plumbing | ✅ Complete |
| 2 | Simulated IMU Driver + Integrator | ✅ Complete |
| 3 | ESKF Core: Delayed Horizon & Propagation | ✅ Complete |
| 4 | ESKF Fusion Math, Chi-Squared Gates & Resets | ✅ Complete |
| 5 | Kinematic Lever Arm Compensation & Attitude | ✅ Complete |
| 6 | Sensor Plumbing, Waiting Rooms & Voting | ⏳ Pending |
| 7 | Launchpad Alignment & Flight State Machine | ⏳ Pending |
| 8 | Commander Arming Gate | ⏳ Pending |
| 9 | EKF Structural Hardening & Configuration | ⏳ Pending |
| — | Rate Conditioner (`angvel.c`) | 🔁 Deferred (no consumer yet) |

---

## Phase 3 — Error-State EKF Core: Delayed Horizon (✅ Complete)

**Goal:** A running ESKF that manages the dual-state timeline, propagates the nominal state, and updates the 15×15 error-state covariance using SymForce-generated C++ math.

*Status: Implemented a production-grade delayed-horizon modular architecture. The filter runs at 250 Hz, pushing `TOPIC_VEHICLE_IMU` frames into a 128-sample ring buffer, evaluating the heavy SymForce predictor on the 200ms delayed tail, and fast-forwarding the `head_state` to the present via `imu_propagate_kinematics()`.*

### ESKF State Definition

We implement a **15-state ESKF** optimised for rocket ascent. Magnetometer data is used exclusively for pre-flight heading alignment; continuous magnetic fusion is unreliable during high-dynamic rocket flight.

| Symbol | Dim | Description |
| --- | --- | --- |
| `q` | 4 | Nominal orientation quaternion (NED ← body) |
| `v_ned` | 3 | Nominal NED velocity (m/s) |
| `p_ned` | 3 | Nominal NED position (m) |
| `bg` | 3 | Nominal gyro bias estimate (rad/s) |
| `ba` | 3 | Nominal accel bias estimate (m/s²) |
| `P[15][15]` | 225 | Error-state covariance, row-major float array |

---

## Phase 4 — Global Fusion Math & Dynamic Gating (✅ Complete)

**Goal:** Expand the SymForce mathematical foundation to include measurement updates for GPS, Barometer, and Magnetometer. Implement dynamic Chi-Squared innovation gates to reject outliers, and timeout logic to handle state resets.

*Status: All fusion math generated and bridged. Hard limits have been replaced by dynamic Chi-Squared Test Ratios that scale with the EKF's internal state variance.*

### Fusion Implementation Details

#### GPS Fusion — `fusion/gps_fuse.{h,c}`

* **Measurement:** 6-DOF position + velocity in NED
* **Gate:** 3-sigma Chi-Squared Test Ratio on both Position and Velocity.
* **Timeout & Reset:** If the Test Ratio exceeds 1.0 for 50 consecutive samples (5 seconds), the filter performs a **Hard Reset**. It overwrites the state and variance, and mathematically decorrelates the covariance matrix by zeroing off-diagonal elements to prevent violent attitude shifts.

#### Barometer Fusion — `fusion/baro_fuse.{h,c}`

* **Measurement:** Altitude = −p_ned[2]
* **Gate:** 5-sigma Chi-Squared Test Ratio.
* **Transonic Lockout:** Configured as **Reject Only**. No timeouts or resets are permitted. When the transonic pressure wave induces a massive fake altitude spike, the test ratio fires and safely locks out the barometer. The EKF dead-reckons through the supersonic phase and automatically resumes fusion once pressure normalizes.

#### Magnetometer Fusion — `fusion/mag_fuse.{h,c}`

* **Measurement:** 3D magnetic field vector.
* **Gate:** Dual gate system. An angular constraint (`MAG_GATE_COS`) prevents wild yaw shifts, while a 3-sigma **Magnitude Chi-Squared Gate** instantly rejects electromagnetic pulses caused by pyrotechnic deployment charges in the avionics bay.

---

### Test Suite — Full Results (5 March 2026)

All 101 host tests pass across 7 domain-separated test files.

**Build command:** `cmake --build build_host --target run_ekf_tests`

```text
=== EKF Core Suite (init / reset / covariance math) ===
  test_init_quaternion_is_identity ............... OK
  test_init_covariance_diagonal .................. OK
  test_covariance_grows_after_predict ............ OK (trace: 600.4529 → 600.4578)

=== EKF Fusion Suite (stationary / GPS / Baro / Mag) ===
  test_stationary_bias_convergence ............... OK (Target: [0.0050, -0.0020, 0.0150])
  test_gps_baro_fusion ........................... OK
  test_mag_fusion ................................ OK

=== EKF Dynamic Flight Suite (launch profile / vibration) ===
  test_launch_profile ............................ OK (True v_ned[2]: -29.413 | EKF: -29.666)
  test_motor_vibration ........................... OK (±50 m/s² noise handled)
  test_high_roll_rate ............................ OK (Lever arm centripetal stripped)

=== EKF Fault Injection Suite (gating / dropouts / NaN) ===
  test_baro_outlier_rejection .................... OK (-200m spike rejected)
  test_gps_velocity_spike ........................ OK (+50m/s glitch rejected)
  test_gps_dropout_dead_reckoning ................ OK (Baro holds Z, X/Y drift bounded)

=== EKF State Machine Suite ===
  test_liftoff_detection ......................... OK
  test_flight_ready_gate ......................... OK

=== EKF Advanced Suite (latency / jitter / unmodeled forces) ===
  test_gps_latency ............................... OK (200ms delayed buffer sync verified)
  test_imu_jitter_and_drops ...................... OK (Dynamic dt handling)
  test_crosswind_drag ............................ OK (Tracks unmodeled East wind push)

══════════════════════════════════════════════════════
  101 / 101 tests passed.
══════════════════════════════════════════════════════

```

---

## Phase 6 — Sensor Plumbing & Waiting Rooms (⏳ Pending)

**Goal:** Establish the pre-EKF FreeRTOS sensor pipeline for Magnetometer, GPS, and dual Barometers, and bridge them into the EKF's timeline.

### Steps

1. **nORB EKF Subscriptions:** Update `ekf_task` to subscribe to `TOPIC_GPS`, `TOPIC_BAROMETER`, and `TOPIC_MAGNETOMETER`.
2. **Baro / Mag Waiting Rooms:** Duplicate the `ekf_delayed_obs_t` logic used for the GPS in `ekf_core.h/c` to create `waiting_baro` and `waiting_mag`. This ensures I2C bus latency does not disrupt the SymForce math.
3. **Publish Head State:** At the end of the `ekf_task` loop, read the real-time position, velocity, and attitude from `ekf.head_state`, construct a `vehicle_state_t` message, and publish it to `TOPIC_VEHICLE_STATE`.

---

## Phase 7 — Launchpad Alignment & Flight State Machine (⏳ Pending)

**Goal:** Implement the hybrid EKF warmup sequence in `ekf_state.c`.

### The 5-Stage Warmup Sequence

1. **Static Check:** Wait for IMU variance to drop below threshold.
2. **Absolute Attitude Init:** Use gravity to set Roll/Pitch. Use Magnetometer + WMM to set initial Yaw.
3. **Absolute Position Init:** Wait for 3D GPS lock. Set `p_ned = {0, 0, 0}` as the launchpad origin.
4. **Bias Convergence (ZVU):** Run at 250 Hz. Apply `symforce_update_stationary()` to calibrate biases.
5. **Liftoff / Flight Mode:** On accel > 2g, disable ZVU, transition to flight fusion (IMU + GPS + Baro).

---

## Phase 8 — Commander Arming Gate (⏳ Pending)

**Goal:** Prevent flight on invalid estimates. Mirrors PX4's `ARMING_STATE_INIT` → `STANDBY` logic.

### Steps

1. Implement `src/modules/commander/commander.c` (10 Hz state machine).
2. Gate `INIT` → `STANDBY` on `gyro_bias_converged == true` AND `pos_valid == true`.
3. Update tracking radio: only transmit beacon if `arming_state >= STANDBY`.

---

## Phase 9 — EKF Structural Hardening & Configuration (⏳ Pending)

**Goal:** Eliminate hardcoded physical constants (`imu_lever_arm`), local gravity assumptions, and tuning variances (`BARO_NOISE_VAR`, `GPS_VEL_GATE`). Replace them with a nORB-published parameter message (`ekf_params.msg`) and dynamically calculated WGS84 gravity, mirroring the modularity of PX4 ECL.

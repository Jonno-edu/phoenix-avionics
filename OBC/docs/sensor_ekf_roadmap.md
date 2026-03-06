
# Sensor + EKF Implementation Roadmap

*Last updated: 6 March 2026*

---

## Architecture Overview

Phoenix Avionics implements a PX4-inspired dual-path IMU architecture split across two distinct data paths, and utilizes a **Delayed Horizon (Time Travel)** architecture to correctly fuse asynchronous, delayed sensor data.

### 1. The Estimator Path (Delayed Horizon)
To handle the reality of sensor processing latency (e.g., GPS taking 150ms+ to compute a fix), the EKF does not run in the present. 
* **The Delayed State (Source of Truth):** The heavy 15-state Error-State Kalman Filter (ESKF) lives strictly ~200ms in the past. When delayed GPS/Baro measurements arrive, they are placed in a "Waiting Room" queue until the delayed EKF's timeline naturally catches up to their exact timestamp. *(Note: SIL testing revealed a temporal starvation bug with a 1-element waiting room; this is being upgraded to a multi-element queue).*
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
  │         └──→ (Wait 200ms) ──→ Delayed EKF (Past) ←── GPS/Baro/Mag (Queued Waiting Rooms)
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
| 6 | SIL CSV Runner & Rocket Dynamics | ✅ Complete |
| 7 | Sensor Plumbing & Queued Waiting Rooms | ⏳ Pending |
| 8 | Launchpad Alignment & Flight State Machine | ✅ Complete |
| 9 | Commander Arming Gate | 🟡 Partial / WIP |
| 10 | EKF Structural Hardening & Configuration | 🟡 Partial / WIP |
| — | Rate Conditioner (`angvel.c`) | 🔁 Deferred |

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
* **Gate:** 3-sigma (or tuned 5-sigma) Chi-Squared Test Ratio on both Position and Velocity.
* **Timeout & Reset:** If the Test Ratio exceeds 1.0 for 50 consecutive samples (5 seconds), the filter performs a **Hard Reset**. It overwrites the state and variance, and mathematically decorrelates the covariance matrix by zeroing off-diagonal elements to prevent violent attitude shifts.

#### Barometer Fusion — `fusion/baro_fuse.{h,c}`

* **Measurement:** Altitude = −p_ned[2]
* **Gate:** 5-sigma Chi-Squared Test Ratio.
* **Transonic Lockout:** Configured as **Reject Only**. No timeouts or resets are permitted. When the transonic pressure wave induces a massive fake altitude spike, the test ratio fires and safely locks out the barometer.

---

## Phase 6 — SIL CSV Runner & Rocket Dynamics (✅ Complete)

**Goal:** Run the EKF against real flight data (`phoenix_sensor_stream.csv`) using a Software-In-the-Loop (SIL) runner to validate fusion, tuning, and real-world edge cases.

*Status: The SIL runner is fully operational against real flight data. The delayed-horizon is no longer bypassed — GPS measurements are correctly queued in the `waiting_gps` room and fused when the delayed tail's timeline catches up. The full ZVU → LEVELING → FLIGHT transition pipeline runs end-to-end. GPS tracks altitude to sub-meter accuracy during coast/descent; Chi-Squared gates correctly reject GPS during the 7G boost phase and transonic baro spikes. Note: `waiting_gps` is still a 1-element struct; upgrading to a multi-element ring buffer (to eliminate any remaining temporal starvation risk at real hardware rates) is tracked as Phase 7 Step 1.*

### Key Discoveries & Implementation Fixes:

1. **Flat-Earth NED Projection:** GPS Lat/Lon/MSL is now converted to a local metric NED grid relative to a dynamically latched launch pad origin.
2. **Boost-Phase Masking:** Fusing GPS during the 7G motor burn poisons the filter due to GPS phase-lock lag and scale-factor errors. GPS fusion is now explicitly disabled while acceleration exceeds `25 m/s²`.
3. **Liftoff Bias Locking:** High-G scale factor errors were causing the filter to hallucinate massive accelerometer biases (creating a "Kalman Tug-of-War"). IMU biases are now mathematically locked (covariance zeroed) the moment FLIGHT mode is engaged.
4. **SIL Dashboard:** A robust Python plotting tool (`plot_csv_results.py`) with 6 subplots tracks Phase Shading, Innovations, Chi-Squared health, and Covariance bounds.
5. **PX4-Style Trapezoidal Pre-Integration:** Raw 1000 Hz IMU samples are averaged with a prev+curr trapezoidal integrator before accumulation into `imu_history_t` frames, reducing aliasing during high-vibration engine burns.
6. **Launch Rail Bypass:** When `FUSE_MAG=false`, the `LEVELING` and `HEADING_ALIGN` warmup phases are skipped via `ekf_state_set_launch_rail()`, which hard-locks the attitude to the known rail geometry and fast-forwards directly to `ZVU_CALIBRATING`.

---

## Phase 7 — Sensor Plumbing & Queued Waiting Rooms (⏳ Pending)

**Goal:** Establish the pre-EKF FreeRTOS sensor pipeline for Magnetometer, GPS, and dual Barometers, and bridge them into the EKF's timeline. This phase bridges the fully-validated SIL EKF into the live FreeRTOS `ekf_task()`.

### Steps

1. **Upgrade Waiting Room to Multi-Element Queue:** `ekf_delayed_obs_t waiting_gps` is currently a 1-element struct. Upgrade to `waiting_gps_queue[4]` so that back-to-back 10 Hz GPS arrivals cannot overwrite a pending measurement while the delayed tail is catching up. (Risk is low in SIL due to controlled timing, but real hardware I2C scheduling makes starvation a realistic hazard.)
2. **nORB EKF Subscriptions:** Update `ekf_task` to subscribe to `TOPIC_GPS`, `TOPIC_BAROMETER`, and `TOPIC_MAGNETOMETER`. Currently `ekf.c` only subscribes to `TOPIC_VEHICLE_IMU` and `TOPIC_EKF_PARAMS`.
3. **Baro / Mag Waiting Rooms:** Implement queued waiting rooms for Barometer and Magnetometer analogous to the GPS waiting room, to absorb I2C bus latency without disrupting the SymForce fusion timing.
4. **GPS Home Latch in ekf_task:** On the first valid GPS fix after entering `ZVU_CALIBRATING`, record the position as the NED origin. Currently this is implemented in the SIL runner (`test_csv_runner.c`) but not in the hardware `ekf.c`.
5. **Publish Head State:** At the end of the `ekf_task` loop, read position, velocity, and attitude from `ekf.head_state`, construct a `vehicle_state_t` message, and publish to `TOPIC_VEHICLE_STATE`.

---

## Phase 8 — Launchpad Alignment & Flight State Machine (✅ Complete)

**Goal:** Implement the hybrid EKF warmup sequence in `ekf_state.c`.

*Status: Fully implemented in `ekf_state.c`. All 5 stages of the warmup sequence are running in SIL and verified against real flight data. The state machine drives the EKF through `UNINITIALIZED → LEVELING → HEADING_ALIGN → ZVU_CALIBRATING → FLIGHT` with correct transition conditions and sensor fusion at each stage. `ekf_state_is_flight_ready()` is implemented to gate arming once biases have converged. Note: The GPS origin latch (Step 3 below) is currently performed in the SIL runner; the hardware wiring into `ekf_task()` and `ekf.c` is tracked under Phase 7.*

### The 5-Stage Warmup Sequence

1. **Static Check / Leveling** (`EKF_MODE_LEVELING`): Accumulates `LEVELING_DURATION_S` (2 s) of accelerometer data and runs ZVU to begin converging biases. Sets pitch/roll from the mean gravity vector on completion.
2. **Absolute Attitude Init** (`EKF_MODE_HEADING_ALIGN`): Continues ZVU. On the first valid magnetometer sample, computes a tilt-compensated heading using the WMM NED reference field and applies a yaw correction via `_apply_yaw()`. If magnetometer is unavailable, `ekf_state_set_launch_rail()` bypasses both this and Leveling, hard-coding attitude from the known rail geometry.
3. **Absolute Position Init:** Waits for first valid GPS fix. Latches that position as the NED origin (`p_ned = {0, 0, 0}`). Implemented in the SIL runner; hardware wiring pending (Phase 7).
4. **Bias Convergence (ZVU)** (`EKF_MODE_ZVU_CALIBRATING`): Runs `symforce_update_stationary()` at 250 Hz. Monitors gyro bias change rate; sets `biases_converged = true` once the rate stays below `ZVU_BIAS_RATE_THRESHOLD` for `ZVU_CONVERGED_DURATION_S` (5 s).
5. **Liftoff / Flight Mode** (`EKF_MODE_FLIGHT`): On accel > 2g, ZVU is disabled, biases are locked (covariance rows/columns zeroed), and the filter transitions to full flight fusion.

---

## Phase 9 — Commander Arming Gate (🟡 Partial / WIP)

**Goal:** Prevent flight on invalid estimates. Mirrors PX4's `ARMING_STATE_INIT` → `STANDBY` logic.

*Status: `ekf_state_is_flight_ready()` is implemented in `ekf_state.c` — it returns `true` once in `ZVU_CALIBRATING` with `biases_converged == true`, or when in `FLIGHT`. The `commander.c` file exists but is currently empty; the arming state machine has not yet been written.*

### Steps

1. ✅ **EKF readiness predicate:** `ekf_state_is_flight_ready()` implemented in `ekf_state.c`.
2. ⏳ Implement `src/modules/commander/commander.c` (10 Hz state machine).
3. ⏳ Gate `INIT` → `STANDBY` on `gyro_bias_converged == true` AND `pos_valid == true`.
4. ⏳ Update tracking radio: only transmit beacon if `arming_state >= STANDBY`.

---

## Phase 10 — EKF Structural Hardening & Configuration (🟡 Partial / WIP)

**Goal:** Eliminate hardcoded physical constants (`imu_lever_arm`), local gravity assumptions, and tuning variances (`BARO_NOISE_VAR`, `GPS_VEL_GATE`). Replace them with a nORB-published parameter message (`ekf_params.msg`) and dynamically calculated WGS84 gravity, mirroring the modularity of PX4 ECL.

*Status: The `ekf_params_t` struct is defined and `TOPIC_EKF_PARAMS` is live — `ekf_task()` polls for parameter updates each cycle and hot-reloads them without restart. The message already carries `imu_lever_arm[3]` and `gravity_ms2`, replacing the formerly hardcoded constants. GPS noise variances and Chi-Squared gates are fully runtime-configurable via `ekf.params`. Outstanding: WGS84 latitude-dependent gravity calculation has not yet been implemented — the gravity field is still set as a static scalar. Additionally, some noise floor constants in `baro_fuse.c` and the SymForce wrappers have not yet been migrated to the params struct.*


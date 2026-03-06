
# Sensor + EKF Implementation Roadmap

*Last updated: 5 March 2026*

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
| 6 | SIL CSV Runner & Rocket Dynamics | 🟡 Partial / WIP |
| 7 | Sensor Plumbing & Queued Waiting Rooms | ⏳ Pending |
| 8 | Launchpad Alignment & Flight State Machine | ⏳ Pending |
| 9 | Commander Arming Gate | ⏳ Pending |
| 10 | EKF Structural Hardening & Configuration | ⏳ Pending |
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

## Phase 6 — SIL CSV Runner & Rocket Dynamics (🟡 Partial / WIP)

**Goal:** Run the EKF against real flight data (`phoenix_sensor_stream.csv`) using a Software-In-the-Loop (SIL) runner to validate fusion, tuning, and real-world edge cases.

*Status: The CSV runner successfully tracks altitude and velocity to sub-meter accuracy using GPS. However, the true delayed horizon is currently **bypassed**. To get around a temporal starvation bug in the 1-element waiting room, SIL currently forces the GPS timestamp to match the delayed tail. This must be fixed in Phase 7 before SIL is considered complete.*

### Key Discoveries & Implementation Fixes:

1. **Flat-Earth NED Projection:** GPS Lat/Lon/MSL is now converted to a local metric NED grid relative to a dynamically latched launch pad origin.
2. **Boost-Phase Masking:** Fusing GPS during the 7G motor burn poisons the filter due to GPS phase-lock lag and scale-factor errors. GPS fusion is now explicitly disabled while acceleration exceeds `25 m/s²`.
3. **Liftoff Bias Locking:** High-G scale factor errors were causing the filter to hallucinate massive accelerometer biases (creating a "Kalman Tug-of-War"). IMU biases are now mathematically locked (covariance zeroed) the moment FLIGHT mode is engaged.
4. **SIL Dashboard:** A robust Python plotting tool (`plot_csv_results.py`) with 6 subplots tracks Phase Shading, Innovations, Chi-Squared health, and Covariance bounds.

---

## Phase 7 — Sensor Plumbing & Queued Waiting Rooms (⏳ Pending)

**Goal:** Establish the pre-EKF FreeRTOS sensor pipeline for Magnetometer, GPS, and dual Barometers, and bridge them into the EKF's timeline.

### Steps

1. **Fix the Temporal Starvation Bug:** Upgrade `ekf_delayed_obs_t waiting_gps` from a 1-element struct to a multi-element ring buffer (`waiting_gps_queue[4]`). This allows the 10Hz GPS to arrive asynchronously without overwriting itself while waiting for the 200ms delayed horizon to catch up.
2. **nORB EKF Subscriptions:** Update `ekf_task` to subscribe to `TOPIC_GPS`, `TOPIC_BAROMETER`, and `TOPIC_MAGNETOMETER`.
3. **Baro / Mag Waiting Rooms:** Implement identical queued waiting rooms for the Barometer and Magnetometer to ensure I2C bus latency does not disrupt the SymForce math.
4. **Publish Head State:** At the end of the `ekf_task` loop, read the real-time position, velocity, and attitude from `ekf.head_state`, construct a `vehicle_state_t` message, and publish it to `TOPIC_VEHICLE_STATE`.

---

## Phase 8 — Launchpad Alignment & Flight State Machine (⏳ Pending)

**Goal:** Implement the hybrid EKF warmup sequence in `ekf_state.c`.

### The 5-Stage Warmup Sequence

1. **Static Check:** Wait for IMU variance to drop below threshold.
2. **Absolute Attitude Init:** Use gravity to set Roll/Pitch. Use Magnetometer + WMM to set initial Yaw.
3. **Absolute Position Init:** Wait for 3D GPS lock. Set `p_ned = {0, 0, 0}` as the launchpad origin.
4. **Bias Convergence (ZVU):** Run at 250 Hz. Apply `symforce_update_stationary()` to calibrate biases.
5. **Liftoff / Flight Mode:** On accel > 2g, disable ZVU, lock biases, and transition to flight fusion.

---

## Phase 9 — Commander Arming Gate (⏳ Pending)

**Goal:** Prevent flight on invalid estimates. Mirrors PX4's `ARMING_STATE_INIT` → `STANDBY` logic.

### Steps

1. Implement `src/modules/commander/commander.c` (10 Hz state machine).
2. Gate `INIT` → `STANDBY` on `gyro_bias_converged == true` AND `pos_valid == true`.
3. Update tracking radio: only transmit beacon if `arming_state >= STANDBY`.

---

## Phase 10 — EKF Structural Hardening & Configuration (⏳ Pending)

**Goal:** Eliminate hardcoded physical constants (`imu_lever_arm`), local gravity assumptions, and tuning variances (`BARO_NOISE_VAR`, `GPS_VEL_GATE`). Replace them with a nORB-published parameter message (`ekf_params.msg`) and dynamically calculated WGS84 gravity, mirroring the modularity of PX4 ECL.


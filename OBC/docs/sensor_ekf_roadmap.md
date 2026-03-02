# Sensor + EKF Implementation Roadmap

## Architecture Overview

Phoenix Avionics implements a PX4-inspired dual-path IMU architecture split across two distinct data paths:

- **The EKF Path (Observer):** Statically calibrated IMU data is integrated into delta-angles and delta-velocities and fed to the error-state EKF. The EKF propagates the nominal state and estimates in-flight gyro and accel bias drift. It intentionally never receives bias-corrected data — it needs to *see* the drift to estimate it.

- **The Rate Path (Consumer):** Raw gyro data has static calibration AND the EKF's live bias estimate subtracted from it, then is passed through a low-pass filter and published as `vehicle_angular_velocity`. The future rate controller receives perfectly zeroed, debiased angular rates in real time.

```
Raw IMU (TOPIC_SENSOR_IMU, 250 Hz)
  │
  ├──→ Integrator → TOPIC_VEHICLE_IMU (250 Hz) ──→ Error-State EKF
  │                                                        │
  │                                        TOPIC_ESTIMATOR_SENSOR_BIAS (bias feedback)
  │                                                        │
  └──→ Rate Conditioner ←───────────────────────────────────
            │
            └──→ TOPIC_VEHICLE_ANGULAR_VELOCITY (250 Hz) ──→ Rate Controller (future)
```

The EKF uses an **Error-State Kalman Filter (ESKF)** formulation:
- The **nominal state** (`q`, `v_ned`, `p_ned`, `bg`, `ba`) is propagated deterministically via non-linear kinematics using IMU delta angles and delta velocities.
- The **15-element error state** (`δθ[3]`, `δv[3]`, `δp[3]`, `δbg[3]`, `δba[3]`) is maintained as a Gaussian distribution `N(0, P)`.
- Measurement corrections update the error state via the standard linear Kalman update, then the correction is injected back into the nominal state and the error state is reset to zero.
- This avoids the quaternion unit-norm constraint problem and gives significantly better linearisation accuracy than a direct quaternion EKF.

---

## Phase 1 — Foundation: Topics, Build, and BSP Plumbing

**Goal:** Establish every structural pre-requisite. Nothing else can start without this phase.

### Steps

1. **Define five new `.msg` files** in `msg/`:

   | File | Key Fields | Purpose |
   |---|---|---|
   | `vehicle_imu.msg` | `timestamp_us uint64_t`, `delta_angle float[3]`, `delta_velocity float[3]`, `dt_s float` | Integrator → EKF input |
   | `estimator_sensor_bias.msg` | `timestamp_us uint64_t`, `gyro_bias_rads float[3]`, `accel_bias_ms2 float[3]`, `gyro_bias_valid bool`, `accel_bias_valid bool` | EKF bias → Rate Conditioner feedback |
   | `vehicle_angular_velocity.msg` | `timestamp_us uint64_t`, `xyz_rads float[3]` | Rate Conditioner → future Rate Controller |
   | `estimator_status.msg` | `timestamp_us uint64_t`, `pos_test_ratio float`, `vel_test_ratio float`, `gyro_bias_converged bool`, `pos_valid bool`, `vel_valid bool` | EKF health → Commander arming gate |
   | `gps.msg` | `timestamp_us uint64_t`, `pos_ned_m float[3]`, `vel_ned_ms float[3]`, `fix_type uint8_t`, `accuracy_m float` | GPS driver → EKF measurement update |
   | `vehicle_status.msg` | `timestamp_us uint64_t`, `arming_state uint8_t`, `ekf_healthy bool` | Commander state machine output |

2. **Enable `esl-math` in `CMakeLists.txt`**: uncomment `${MATH_LIB_SOURCES}` and add `lib/esl-math/include` to `target_include_directories`. The quaternion, DCM, and matrix primitives are required by the ESKF.

3. **Wire `bsp_peripheral_init()` into the boot sequence** in `src/main.c` (before `sensors_init()`). This initialises the I2C bus that all physical sensor drivers depend on.

4. **Regenerate nORB**: run CMake (the Python generator runs automatically on configure). Verify all new `TOPIC_*` enum values appear in `build/norb_generated/norb/topics.h`.

### Verification
Clean CMake build succeeds. All new topic enums are visible. No linker errors from esl-math.

---

## Phase 2 — Simulated IMU Driver + Integrator

**Goal:** Continuous, physically plausible data flowing through `TOPIC_SENSOR_IMU` → `TOPIC_VEHICLE_IMU` at 250 Hz.

### Steps

1. **Improve the simulated IMU driver** in `src/modules/sensors/imu.c` (currently stubs `accel_ms2[2] = 9.81f`):
   - Output gravity-aligned accelerometer: `accel = {0, 0, 9.80665}` (NED convention, Z-down) plus configurable Gaussian white noise terms.
   - Output zero mean angular rate plus a configurable **simulated DC gyro bias** (e.g. `0.01 rad/s` on Z). This gives the ESKF something real to identify during testing.
   - The existing 250 Hz loop at `configMAX_PRIORITIES - 1` is correct — no task changes needed.

2. **Implement `src/modules/sensors/integrator.c`** (currently an empty file):
   - `integrator_init()` creates a task or runs inline within the IMU task after publish.
   - Each 4 ms IMU cycle: `norb_subscribe_poll(TOPIC_SENSOR_IMU, &imu)`, accumulate `delta_angle[i] += gyro_rads[i] * dt` and `delta_velocity[i] += accel_ms2[i] * dt`.
   - Publish `TOPIC_VEHICLE_IMU`, then reset accumulators.
   - Wire `integrator_init()` into `sensors_init()` in `src/modules/sensors/sensors.c`.

### Verification
USB serial logging shows `vehicle_imu` delta-velocity ~`0.0393 m/s` per cycle (= 9.80665 × 0.004 s). Delta-angle magnitude matches the simulated bias rate × dt.

---

## Phase 3 — Error-State EKF Core (IMU Propagation Only)

**Goal:** A running ESKF that propagates both the nominal state and the 15×15 error-state covariance from IMU data, and begins converging on gyro and accel bias estimates — without any measurement updates yet.

### ESKF State Definition

| Symbol | Dimension | Description |
|---|---|---|
| `q` | 4 | Nominal orientation quaternion (NED ← body) |
| `v_ned` | 3 | Nominal NED velocity (m/s) |
| `p_ned` | 3 | Nominal NED position (m) |
| `bg` | 3 | Nominal gyro bias estimate (rad/s) |
| `ba` | 3 | Nominal accel bias estimate (m/s²) |
| `P[15][15]` | 15×15 | Error-state covariance |

Error state ordering: `[δθ, δv, δp, δbg, δba]`.

### Steps

1. **Define the nominal state struct and `P[15][15]`** at the top of `src/modules/estimator/ekf.c`. Use `Quaternion_t`, `Matrix_t`, `Vector3_t` from `lib/esl-math/include/` throughout.

2. **Implement the ESKF predict step** (runs at 250 Hz, triggered by `norb_subscribe_poll(TOPIC_VEHICLE_IMU)`):

   - **Bias-corrected inputs:**
     ```
     da = delta_angle  - bg * dt
     dv = delta_velocity - ba * dt
     ```
   - **Nominal state propagation (non-linear, no Jacobians here):**
     ```
     q    ← q ⊗ exp(da/2)          [quaternion kinematic integration]
     q    ← normalize(q)
     dv_n ← R(q_prev) * dv         [rotate delta-velocity to NED]
     v_ned ← v_ned + dv_n - g * dt [subtract gravity in NED]
     p_ned ← p_ned + v_ned * dt
     bg, ba unchanged (random walk, driven by Q)
     ```
   - **Error-state covariance propagation:**
     ```
     P ← F * P * Fᵀ + Q
     ```
     where `F` is the 15×15 Jacobian of the error dynamics (derived analytically from the process model) and `Q` is the process noise matrix. Both `F` and `Q` are defined as named `#define` or `static const` constants in `ekf.c` to make tuning accessible.

3. **Publish after each predict step:**
   - `TOPIC_VEHICLE_STATE`: fill quaternion, velocity, position, and the current `bg` / `ba` nominal state.
   - `TOPIC_ESTIMATOR_SENSOR_BIAS`: `gyro_bias_rads = bg`, `accel_bias_ms2 = ba`. Set `gyro_bias_valid = true` once the diagonal variance `P[9][9]` (the bias variance) drops below a configurable threshold `GYRO_BIAS_CONVERGENCE_VAR`.

4. **Set initial P conservatively large** (high uncertainty allows convergence from any start). All tuning constants (`Q_GYRO`, `Q_ACCEL`, `Q_GBIAS`, `Q_ABIAS`, `P0_*`) are defined as named constants at the top of `ekf.c`.

### Note on Yaw Observability
Without a magnetometer or dual-antenna GPS, yaw rotation about the gravity vector is **unobservable while stationary**. The yaw component of `P` will not decrease. This is an accepted limitation for Phase 3. Yaw can be initialised from the BMM350 magnetometer (driver already exists in `lib/esl-sensors/`) as a future extension without modifying Phase 3 code.

### Verification
Log `vehicle_state.quaternion` and `estimator_sensor_bias.gyro_bias_rads` over USB CDC. Quaternion should remain near `{1, 0, 0, 0}`. The Z-axis gyro bias estimate should converge toward the simulated `0.01 rad/s` offset within 10–60 seconds (depends on `Q_GBIAS` and `P0_GBIAS` tuning).

---

## Phase 4 — Rate Conditioner Module

**Goal:** Close the EKF bias feedback loop. The Rate Conditioner applies the ESKF's live bias estimate to produce perfectly zeroed, filtered angular rates for the future rate controller.

### Steps

1. **Create `src/modules/sensors/angvel.c`** (or add `angvel_task` inline in `sensors.c`):
   - Task priority: `configMAX_PRIORITIES - 1` (same as IMU task — runs immediately after each IMU publish).
   - Loop at 250 Hz using `vTaskDelayUntil`:
     1. `norb_subscribe_poll(TOPIC_SENSOR_IMU, &raw)`
     2. `norb_subscribe_poll(TOPIC_ESTIMATOR_SENSOR_BIAS, &bias)`
     3. `corrected[i] = raw.gyro_rads[i] - (bias.gyro_bias_valid ? bias.gyro_bias_rads[i] : 0.0f)`
     4. Apply a 1st-order IIR low-pass filter: `alpha = dt / (dt + 1/(2π * f_cutoff))`, `y[i] = alpha * corrected[i] + (1 - alpha) * y_prev[i]`. Default cutoff `f_cutoff = 30 Hz` (configurable `#define`).
     5. `norb_publish(TOPIC_VEHICLE_ANGULAR_VELOCITY, &angvel)`

2. **Wire `angvel_task` creation** into `sensors_init()` in `sensors.c` using `xTaskCreateStatic()` with a pre-declared static stack array (required — no dynamic allocation).

3. **Add `angvel.c` to `CMakeLists.txt`** source list.

### Verification
Without bias correction active (pre-convergence), `vehicle_angular_velocity.xyz[2]` should show the simulated `0.01 rad/s` DC offset plus noise. After ESKF gyro bias converges (Phase 3), the DC offset should disappear from the published rate.

---

## Phase 5 — GPS Plumbing + ESKF Measurement Update

**Goal:** Real (or simulated) GPS position and velocity corrections closing the ESKF loop.

### Steps

1. **Implement `src/modules/sensors/gps.c`**:
   - Call `GPSNEOM9N_init(i2c_port, I2C_GPS_ADDRESS)` from the existing driver at `lib/esl-sensors/src/gpsNEOM9N.c`.
   - Create `gps_task` at 5 Hz (200 ms period), priority 10, using `xTaskCreateStatic()`.
   - `GPSNEOM9N_readNavPVT(&pvt)` → convert geodetic lat/lon/alt to local NED (using the first valid fix as the origin) → publish `TOPIC_GPS`.
   - **Simulation fallback:** when the physical GPS driver is not yet available, the task publishes a static `{0, 0, 0}` position fix with high `accuracy_m` — sufficient to exercise the measurement update code path.

2. **Implement the ESKF GPS measurement update step** in `ekf.c` (runs at 5 Hz on each `TOPIC_GPS` message):
   - Innovation: `y = [gps.pos_ned_m; gps.vel_ned_ms] - H * x_nominal` (H selects `p_ned` and `v_ned`, 6×15)
   - Innovation covariance: `S = H * P * Hᵀ + R_gps` (R_gps diagonal, tuned from GPS `accuracy_m`)
   - **Innovation gate:** compute `χ² = yᵀ * S⁻¹ * y`; reject the measurement and log a warning if `χ² > GATE_THRESHOLD` (prevents corrupted GPS from corrupting the filter). Publish `pos_test_ratio = χ² / GATE_THRESHOLD` to `TOPIC_ESTIMATOR_STATUS`.
   - Kalman gain: `K = P * Hᵀ * S⁻¹`
   - Error-state update: `δx = K * y`
   - **Inject error into nominal state:**
     ```
     p_ned ← p_ned + δp
     v_ned ← v_ned + δv
     q     ← q ⊗ exp(δθ/2)
     bg    ← bg + δbg
     ba    ← ba + δba
     ```
   - Reset error state to zero, update covariance: `P ← (I - K*H) * P`

3. **Wire `gps_init()` / `gps_task` creation** into `sensors_init()`.

4. **Add `gps.c` to `CMakeLists.txt`** source list.

### Verification
With the simulated static GPS at origin, `vehicle_state.position_m` should converge to and hold near `{0, 0, 0}`. `pos_test_ratio` and `vel_test_ratio` in `estimator_status` should remain well below `1.0`.

---

## Phase 6 — Commander Arming Gate

**Goal:** Prevent downstream modules (beacon sender, future flight controller) from acting on invalid estimates. Mirrors PX4's `ARMING_STATE_INIT → STANDBY` transition.

### Steps

1. **Implement `src/modules/commander/commander.c`**:
   - Create `commander_task` at 10 Hz (100 ms period), priority 3 (above housekeeping at 2, below datalink at 5), using `xTaskCreateStatic()`.
   - State machine with three states (using the `arming_state` field from `vehicle_status.msg`):

   | State | Value | Transition Condition |
   |---|---|---|
   | `ARMING_STATE_INIT` | 0 | (start state) |
   | `ARMING_STATE_STANDBY` | 1 | `gyro_bias_converged == true` AND `pos_valid == true` AND `pos_test_ratio < 1.0` AND `vel_test_ratio < 1.0` |
   | `ARMING_STATE_ARMED` | 2 | Future — manual arm command via TC |

   - Subscribe to `TOPIC_ESTIMATOR_STATUS` on each cycle to drive transitions.
   - Publish `TOPIC_VEHICLE_STATUS` on every state change (and on first cycle).
   - Add a `commander_init()` declaration to a new `src/modules/commander/commander.h`.

2. **Wire `commander_init()` into `src/main.c`** after `estimator_init()` in the boot sequence.

3. **Update the tracking radio beacon** in `src/modules/nodes/tracking_radio/tracking_radio.c`:
   - Subscribe to `TOPIC_VEHICLE_STATE` and `TOPIC_GPS` at 1 Hz.
   - Replace the hardcoded test lat/lon/alt/vel values in `tracking_radio_send_beacon()` with real nORB data.
   - Only transmit the beacon if `TOPIC_VEHICLE_STATUS.arming_state >= ARMING_STATE_STANDBY` (gating on valid estimator output).

### Verification
On USB CDC monitor, confirm Commander stays in `ARMING_STATE_INIT` for the full gyro bias convergence window (~10–60 s). After transition to `STANDBY`, confirm beacon fields reflect real `vehicle_state` data instead of hardcoded test values. Simulate a GPS glitch that pushes `pos_test_ratio > 1.0` and confirm the Commander does not transition to `STANDBY` (or drops back from it).

---

## Design Decisions

| Decision | Choice | Rationale |
|---|---|---|
| **EKF formulation** | Error-State Kalman Filter (ESKF) | Better linearisation accuracy; avoids quaternion unit-norm constraint; standard approach in aerospace GNSS/INS fusion |
| **Nominal state propagation** | Non-linear kinematics (no Jacobians in predict nominal) | Accurate propagation; Jacobians only needed for error-state covariance `F * P * Fᵀ` |
| **EKF math library** | Written from scratch in C using `esl-math` quaternion/DCM/matrix primitives | Full control; stays within the existing toolchain; no porting overhead |
| **Sensor redundancy** | Single IMU; no voting in this plan | Reduces complexity; `TOPIC_ESTIMATOR_STATUS` design allows a future multi-EKF selector module to be added without modifying existing code |
| **Yaw** | Unobservable without magnetometer; accepted limitation | BMM350 driver exists in `lib/esl-sensors/`; adding a mag measurement update is a self-contained future extension |
| **Barometer** | Excluded from EKF for now | `TOPIC_BAROMETER` already exists; MS5607 and BMP581 drivers are ready; adding baro altitude measurement update is self-contained |
| **FreeRTOS allocation** | All tasks use `xTaskCreateStatic()` | `configSUPPORT_DYNAMIC_ALLOCATION = 0` throughout the codebase |
| **IMU driver** | Simulated driver for now | Real hardware driver to be implemented separately when chip selection is finalised |

---

## Module Status Reference

| Module | File | Status |
|---|---|---|
| nORB bus | `src/norb/norb.c` | ✅ Complete |
| FreeRTOS integration | `FreeRTOSConfig.h` / `src/main.c` | ✅ Complete |
| Datalink / RS485 | `src/modules/datalink/` | ✅ Functional |
| EPS node | `src/modules/nodes/eps/eps.c` | ✅ Functional |
| Tracking radio | `src/modules/nodes/tracking_radio/` | ⚠️ Partial (hardcoded beacon data) |
| Housekeeping task | `src/modules/housekeeping/` | ⚠️ Partial (no sensor telemetry streams) |
| IMU task | `src/modules/sensors/imu.c` | ⚠️ Stub (fakes gravity, no hardware) |
| Integrator | `src/modules/sensors/integrator.c` | ❌ Empty |
| Rate Conditioner | `src/modules/sensors/angvel.c` | ❌ Not yet created |
| Barometer | `lib/esl-sensors/src/baroMS5607.c` | ❌ Driver ready; not wired in |
| GPS | `src/modules/sensors/gps.c` + `lib/esl-sensors/src/gpsNEOM9N.c` | ❌ Driver ready; task empty |
| Magnetometer | `lib/esl-sensors/src/magBMM350.c` | ❌ Driver ready; not wired in |
| ESKF / Estimator | `src/modules/estimator/ekf.c` | ❌ Stub (loop only) |
| Commander | `src/modules/commander/commander.c` | ❌ Empty file |
| Datalink streams | `src/modules/datalink/streams/` | ❌ Framework stub |

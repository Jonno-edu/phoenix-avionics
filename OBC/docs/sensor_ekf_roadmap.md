# Sensor + EKF Implementation Roadmap

## Architecture Overview

Phoenix Avionics implements a PX4-inspired dual-path IMU architecture split across two distinct data paths:

* **The EKF Path (Observer):** Statically calibrated IMU data is integrated into delta-angles and delta-velocities and fed to the error-state EKF. The EKF propagates the nominal state and estimates in-flight gyro and accel bias drift. It intentionally never receives bias-corrected data — it needs to *see* the drift to estimate it.
* **The Rate Path (Consumer):** Raw gyro data has static calibration AND the EKF's live bias estimate subtracted from it, then is passed through a low-pass filter and published as `vehicle_angular_velocity`. The future rate controller receives perfectly zeroed, debiased angular rates in real time.

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

The EKF uses an **Error-State Kalman Filter (ESKF)** formulation derived from Joan Solà's *"Quaternion kinematics for the error-state Kalman filter"*.

* The **nominal state** (`q`, `v_ned`, `p_ned`, `bg`, `ba`) is propagated deterministically via non-linear kinematics using IMU delta angles and delta velocities.
* The **15-element error state** (`δθ`, `δv`, `δp`, `δbg`, `δba`) is maintained as a Gaussian distribution `N(0, P)`.
* Measurement corrections update the error state via the standard linear Kalman update, then the correction is injected back into the nominal state and the error state is reset to zero.
* This avoids the quaternion unit-norm constraint problem and gives significantly better linearisation accuracy than a direct quaternion EKF.

---

## Phase 1 — Foundation: Topics, Build, and BSP Plumbing (✅ Complete)

**Goal:** Establish every structural pre-requisite. Nothing else can start without this phase.
*Status: `.msg` files created, nORB topics generated, esl-math enabled, and build succeeds.*

---

## Phase 2 — Simulated IMU Driver + Integrator (✅ Complete)

**Goal:** Continuous, physically plausible data flowing through `TOPIC_SENSOR_IMU` → `TOPIC_VEHICLE_IMU` at 250 Hz.
*Status: 1kHz sim driver functional with gravity + DC bias on Z-axis. Integrator task accurately accumulating 4x1ms windows into 250Hz outputs.*

---

## Phase 3 — Error-State EKF Core: IMU Propagation (✅ Complete)

**Goal:** A running ESKF that performs static alignment, propagates the nominal state, updates the 15×15 error-state covariance using SymPy-generated C math, and begins converging on gyro and accel bias estimates.

*Status: Implemented a production-grade modular EKF architecture. The filter runs at 250Hz, integrating `TOPIC_VEHICLE_IMU` delta-angles/velocities into a 15-state ESKF. High-performance SymForce C++ math is successfully bridged to the C firmware via a zero-copy Eigen wrapper.*

### Module Implementation Details

The estimator module follows a strict separation of concerns:

* **OS Layer (`src/modules/estimator/ekf.c`)**:
* Handles the 250Hz FreeRTOS task and nORB messaging.


* **Filter Core (`src/modules/estimator/ekf_core.c`)**:
* Owns the 15-state memory (`q`, `v`, `p`, `bg`, `ba`) and the 15x15 covariance matrix.


* **Fusion Layer (`src/modules/estimator/fusion/imu_predictor.c`)**:
* Implements non-linear kinematics for nominal state propagation.


* **Math Bridge (`src/modules/estimator/ekf_math/symforce_wrapper.cpp`)**:
* A C++ "bridge" that uses `Eigen::Map` to cast raw C arrays into Eigen types.


* **Generation Logic (`src/modules/estimator/ekf_math/generate_eskf_math.py`)**:
* SymForce Python script that derives the ESKF Jacobians and unrolls the $F P F^T$ math into pure C++.


* **Testing Infrastructure (`src/modules/estimator/tests/test_ekf_predict.c`)**:
* Pure C standalone test suite configured to build and run natively on the host machine.



### ESKF State Definition

We implement a **15-state ESKF** optimized for rocket ascent. Magnetometer data is used exclusively for pre-flight heading alignment on the pad, as continuous magnetic fusion is unreliable during high-dynamic rocket flight.

| Symbol | Dimension | Description |
| --- | --- | --- |
| `q` | 4 | Nominal orientation quaternion (NED ← body) |
| `v_ned` | 3 | Nominal NED velocity (m/s) |
| `p_ned` | 3 | Nominal NED position (m) |
| `bg` | 3 | Nominal gyro bias estimate (rad/s) |
| `ba` | 3 | Nominal accel bias estimate (m/s²) |
| `P[15][15]` | 15×15 | Error-state covariance (Single-precision float array) |

Error state ordering: `[δθ, δv, δp, δbg, δba]`.

---

## Phase 4 — ESKF Global Fusion Math & Unit Tests (🔄 Next)

**Goal:** Expand the SymForce mathematical foundation to include measurement updates for GPS, Barometer, and Magnetometer. Prove that the filter correctly converges using host-compiled unit tests and synthetic data *before* integrating with FreeRTOS or hardware drivers.

### Steps

1. **SymPy Code Generation:** - Add `update_gps` (6D Pos/Vel), `update_baro` (1D Alt), and `update_heading` (1D Yaw) to `generate_eskf_math.py`.
* Symbolically evaluate the Kalman Gain $K = P H^T (H P H^T + R)^{-1}$ and state update $\delta x = K y$ for each sensor.


2. **Implement the C++ Wrappers:** - Add the new update functions to `symforce_wrapper.cpp`.
3. **Build the Convergence Test Suite:**
* Write standalone host tests (e.g., `test_mag_heading_convergence()`, `test_gps_position_convergence()`) injecting synthetic Gaussian noise and specific hidden biases.



### Verification

Pure C standalone test suite successfully runs on the host machine. The filter snaps to the true heading/position/bias hidden within the simulated noise.

---

## Phase 5 — Rate Conditioner Module (⏳ Pending)

**Goal:** Close the EKF bias feedback loop. Produce perfectly zeroed, filtered angular rates for the future rate controller.

### Steps

1. **Implement `src/modules/sensors/angvel.c**` at 250 Hz.
2. Subscribe to `TOPIC_SENSOR_IMU` and `TOPIC_ESTIMATOR_SENSOR_BIAS`.
3. Subtract the live bias: `corrected = raw_gyro - (bias.valid ? bias.gyro_bias : 0.0f)`.
4. Apply a 1st-order IIR low-pass filter (e.g., $f_{cutoff} = 30\text{ Hz}$).
5. Publish to `TOPIC_VEHICLE_ANGULAR_VELOCITY`.

### Verification

Once the ESKF gyro bias converges, the DC offset in `vehicle_angular_velocity` should smoothly drop to zero.

---

## Phase 6 — Sensor Plumbing & Voting (⏳ Pending)

**Goal:** Establish the pre-EKF FreeRTOS sensor pipeline for Magnetometer, GPS, and dual Barometers.

### Steps

1. **Redundant Barometer Voting**:
* Implement `voted_sensors_update.c` to read both MS5607 and BMP581.
* Select primary baro based on health/variance and publish unified `TOPIC_BAROMETER`.


2. **Magnetometer & WMM**:
* Integrate `mag.c` driver.
* Feed GPS lat/lon into an onboard World Magnetic Model (WMM) lookup to calculate local declination.


3. **GPS Driver**:
* Utilize the `GPSNEOM9N` driver to publish `TOPIC_GPS` at 5 Hz.



---

## Phase 7 — Launchpad Alignment & Flight State Machine (⏳ Pending)

**Goal:** Implement the hybrid EKF warmup sequence in `ekf.c`. Use GPS/Mag for absolute references, and the ZVU/ZRU math for high-precision bias calibration.

### Steps (The 5-Stage Warmup)

1. **Static Check:** Wait for IMU variance to drop below a threshold (proves the rocket is untouched on the pad).
2. **Absolute Attitude Init:** Use gravity to mathematically set the Roll/Pitch. Use the Magnetometer + WMM to set the initial Yaw.
3. **Absolute Position Init:** Wait for a 3D GPS lock. Set `p_ned = {0,0,0}` as the launchpad origin.
4. **Bias Convergence (ZVU/ZRU):** Run the filter at 250Hz. Apply the 100% trusted `symforce_update_stationary()` to rapidly zero out the `gyro_bias` and `accel_bias`.
5. **Liftoff / Flight Mode:** The moment the accelerometer exceeds 2Gs, **disable** the ZVU, dynamically de-weight the Magnetometer variance, and transition to standard flight fusion (IMU + GPS + Baro).

---

## Phase 8 — Commander Arming Gate (⏳ Pending)

**Goal:** Prevent flight on invalid estimates (Mirrors PX4's `ARMING_STATE_INIT` → `STANDBY` logic).

### Steps

1. **Implement `src/modules/commander/commander.c**` (10 Hz state machine).
2. Gate the transition from `INIT` to `STANDBY` on `gyro_bias_converged == true` AND `pos_valid == true` from `TOPIC_ESTIMATOR_STATUS`.
3. Publish `TOPIC_VEHICLE_STATUS` changes.
4. **Update Tracking Radio:** Only transmit beacon if `arming_state >= STANDBY`.

### Verification

Commander holds in `INIT` for the bias convergence window, then transitions to `STANDBY` and tracking beacon begins transmitting valid EKF data.

---

## Design & Academic References

| Element | Reference / Rationale |
| --- | --- |
| **EKF formulation** | Error-State Kalman Filter (ESKF). Based on Joan Solà's *"Quaternion kinematics for the error-state Kalman filter"* (2017). |
| **Covariance Math** | Derived symbolically via SymPy and exported as unrolled C, matching the architecture of PX4 ECL. Guarantees real-time execution on Cortex-M33 single-precision FPU. |
| **Sensor redundancy** | Single IMU; single GPS. |
| **Yaw Observability** | Initialized via Magnetometer on the pad; Integrated purely from gyroscopes during ascent. |


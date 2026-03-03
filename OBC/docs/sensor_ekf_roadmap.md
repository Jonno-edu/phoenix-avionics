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

The EKF uses an **Error-State Kalman Filter (ESKF)** formulation derived from Joan Solà's *"Quaternion kinematics for the error-state Kalman filter"*.
- The **nominal state** (`q`, `v_ned`, `p_ned`, `bg`, `ba`) is propagated deterministically via non-linear kinematics using IMU delta angles and delta velocities.
- The **15-element error state** (`δθ`, `δv`, `δp`, `δbg`, `δba`) is maintained as a Gaussian distribution `N(0, P)`.
- Measurement corrections update the error state via the standard linear Kalman update, then the correction is injected back into the nominal state and the error state is reset to zero.
- This avoids the quaternion unit-norm constraint problem and gives significantly better linearisation accuracy than a direct quaternion EKF.

***

## Phase 1 — Foundation: Topics, Build, and BSP Plumbing (✅ Complete)

**Goal:** Establish every structural pre-requisite. Nothing else can start without this phase.
*Status: `.msg` files created, nORB topics generated, esl-math enabled, and build succeeds.*

***

## Phase 2 — Simulated IMU Driver + Integrator (✅ Complete)

**Goal:** Continuous, physically plausible data flowing through `TOPIC_SENSOR_IMU` → `TOPIC_VEHICLE_IMU` at 250 Hz.
*Status: 1kHz sim driver functional with gravity + DC bias on Z-axis. Integrator task accurately accumulating 4x1ms windows into 250Hz outputs.*

***

## Phase 3 — Error-State EKF Core: IMU Propagation (✅ Complete)

**Goal:** A running ESKF that performs static alignment, propagates the nominal state, updates the 15×15 error-state covariance using SymPy-generated C math, and begins converging on gyro and accel bias estimates.

*Status: Implemented a production-grade modular EKF architecture. The filter runs at 250Hz, integrating `TOPIC_VEHICLE_IMU` delta-angles/velocities into a 15-state ESKF. High-performance SymForce C++ math is successfully bridged to the C firmware via a zero-copy Eigen wrapper.*

### Module Implementation Details

The estimator module follows a strict separation of concerns:

- **OS Layer ([src/modules/estimator/ekf.c](src/modules/estimator/ekf.c))**: 
    - Handles the 250Hz FreeRTOS task and nORB messaging. 
    - Subscribes to integrated IMU data and eventually publishes state estimates.
- **Filter Core ([src/modules/estimator/ekf_core.c](src/modules/estimator/ekf_core.c))**: 
    - Owns the 15-state memory (`q`, `v`, `p`, `bg`, `ba`) and the 15x15 covariance matrix.
    - Manages lifecycle (init/reset).
- **Fusion Layer ([src/modules/estimator/fusion/imu_predictor.c](src/modules/estimator/fusion/imu_predictor.c))**: 
    - Implements non-linear kinematics for nominal state propagation.
    - Orchestrates the covariance update call.
- **Math Bridge ([src/modules/estimator/ekf_math/symforce_wrapper.cpp](src/modules/estimator/ekf_math/symforce_wrapper.cpp))**: 
    - A C++ "bridge" that uses `Eigen::Map` to cast raw C arrays into Eigen types.
    - Calls the highly-optimized SymForce symbolic math templates.
- **Generation Logic ([src/modules/estimator/ekf_math/generate_eskf_math.py](src/modules/estimator/ekf_math/generate_eskf_math.py))**: 
    - SymForce Python script that derives the ESKF Jacobians and unrolls the $F P F^T$ math into pure C++.
    - **Build Integration**: CMake automatically runs this script using the project `.venv` whenever the script changes.

### ESKF State Definition
We implement a **15-state ESKF** (omitting the wind and magnetic field states from the full 24-state PX4 filter).

| Symbol | Dimension | Description |
|---|---|---|
| `q` | 4 | Nominal orientation quaternion (NED ← body) |
| `v_ned` | 3 | Nominal NED velocity (m/s) |
| `p_ned` | 3 | Nominal NED position (m) |
| `bg` | 3 | Nominal gyro bias estimate (rad/s) |
| `ba` | 3 | Nominal accel bias estimate (m/s²) |
| `P[15][15]` | 15×15 | Error-state covariance (Single-precision float array) |

Error state ordering: `[δθ, δv, δp, δbg, δba]`.

### Steps

1. **Create the SymForce Code Generator:** (✅ Done)
   - Created `src/modules/estimator/ekf_math/generate_eskf_math.py`.
   - SymForce generates optimized C++ headers in `gen/cpp/symforce/sym/`.

2. **Implement the Math Bridge:** (✅ Done)
   - Created `symforce_wrapper.cpp` (C++) and `symforce_wrapper.h` (C/C++ bridge).
   - Uses `Eigen::Map` to bridge raw C float arrays to SymForce's Eigen-based math templates.

3. **Implement EKF Core & IMU Propagation:** (✅ Done)
   - `ekf_core.c`: Manages the 11-state nominal vector + 15x15 covariance memory.
   - `imu_predictor.c`: Implements the 250Hz nominal kinematics (quaternion integration + rotation) and calls the covariance bridge.

4. **Publish Outputs:** (⏳ Pending)
   - Publish `TOPIC_VEHICLE_STATE` (nominal state).
   - Publish `TOPIC_ESTIMATOR_SENSOR_BIAS`.

### Verification
Log `vehicle_state.quaternion` and `estimator_sensor_bias.gyro_bias_rads`. During the alignment phase, `q` should correctly initialize pitch/roll. During the predict loop, `q` should remain stable, and the Z-axis gyro bias should converge toward the simulated `0.01 rad/s` offset.

***

## Phase 4 — Rate Conditioner Module (🔄 Next)

**Goal:** Close the EKF bias feedback loop. Produce perfectly zeroed, filtered angular rates for the future rate controller.

### Steps
1. **Implement `src/modules/sensors/angvel.c`** at 250 Hz.
2. Subscribe to `TOPIC_SENSOR_IMU` and `TOPIC_ESTIMATOR_SENSOR_BIAS`.
3. Subtract the live bias: `corrected = raw_gyro - (bias.valid ? bias.gyro_bias : 0.0f)`.
4. Apply a 1st-order IIR low-pass filter (e.g., $f_{cutoff}$ = 30 Hz).
5. Publish to `TOPIC_VEHICLE_ANGULAR_VELOCITY`.

### Verification
Once the ESKF gyro bias converges in Phase 3, the DC offset in `vehicle_angular_velocity` should smoothly drop to zero.

***

## Phase 5 — GPS Plumbing + ESKF Measurement Update (⏳ Pending)

**Goal:** Real (or simulated) GPS position and velocity corrections using SymPy-generated Kalman updates.

### Steps
1. **Implement `src/modules/sensors/gps.c`** utilizing the `GPSNEOM9N` driver to publish `TOPIC_GPS` at 5 Hz.
2. **SymPy Code Generation:** Use the Python script to symbolically evaluate the Kalman Gain $K = P H^T (H P H^T + R)^{-1}$ and state update $\delta x = K y$.
3. **Implement ESKF Measurement Update in `ekf.c`**:
   - Compute Innovation $y$ and Innovation Covariance $S$.
   - Perform Innovation gate check ($\chi^2$).
   - Run generated Kalman Gain and Error-State update equations.
   - Inject error state into the nominal state ($p = p + \delta p$, $q = q \otimes \exp(\delta \theta/2)$, etc.)
   - Update covariance $P = (I - KH)P$.

### Verification
`vehicle_state.position_m` should hold tightly at `{0,0,0}` using simulated GPS data. Test-ratios should remain below 1.0.

***

## Phase 6 — Commander Arming Gate (⏳ Pending)

**Goal:** Prevent flight on invalid estimates (Mirrors PX4's `ARMING_STATE_INIT` → `STANDBY` logic).

### Steps
1. **Implement `src/modules/commander/commander.c`** (10 Hz state machine).
2. Gate the transition from `INIT` to `STANDBY` on `gyro_bias_converged == true` AND `pos_valid == true` from `TOPIC_ESTIMATOR_STATUS`.
3. Publish `TOPIC_VEHICLE_STATUS` changes.
4. **Update Tracking Radio:** Only transmit beacon if `arming_state >= STANDBY`.

### Verification
Commander holds in `INIT` for the ~10–60s bias convergence window, then transitions to `STANDBY` and tracking beacon begins transmitting valid EKF data.

***

## Design & Academic References

| Element | Reference / Rationale |
|---|---|
| **EKF formulation** | Error-State Kalman Filter (ESKF). Based on Joan Solà's *"Quaternion kinematics for the error-state Kalman filter"* (2017). |
| **Covariance Math** | Derived symbolically via SymPy and exported as unrolled C, matching the architecture of PX4 ECL. Guarantees real-time execution on Cortex-M33 single-precision FPU. |
| **Sensor redundancy** | Single IMU; single GPS. |
| **Yaw Observability** | Unobservable without magnetometer; accepted limitation. BMM350 integration is a modular future addition. |

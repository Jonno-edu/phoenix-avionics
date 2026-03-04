#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
File: generate_eskf_math.py
Description:
    Derivation of a 15-state error-state EKF based on PX4 ECL and Joan Sola's paper: 
    "Quaternion kinematics for the error-state Kalman filter".
    Generates pure unrolled C code for Cortex-M33 execution.
"""

import sympy as sp
import symforce
symforce.set_epsilon_to_symbol()

import symforce.symbolic as sf
from symforce import typing as T
from symforce.values import Values
from symforce.codegen import Codegen, CppConfig
import os

# 1. Define the 15-State Vector (Nominal State)
State = Values(
    quat_nominal = sf.Rot3(),
    vel = sf.V3(),
    pos = sf.V3(),
    gyro_bias = sf.V3(),
    accel_bias = sf.V3()
)

# 2. Define the 15-D Error State (Tangent Space)
class MTangent(sf.Matrix):
    SHAPE = (State.tangent_dim(), State.tangent_dim())

class VTangent(sf.Matrix):
    SHAPE = (State.tangent_dim(), 1)

def predict_covariance(
    q: sf.V4,       # [w, x, y, z]
    vel: sf.V3,
    pos: sf.V3,
    gyro_bias: sf.V3,
    accel_bias: sf.V3,
    P: MTangent,
    accel: sf.V3,
    accel_var: sf.V3,
    gyro: sf.V3,
    gyro_var: sf.Scalar,
    dt: sf.Scalar
) -> MTangent:
    """
    Symbolic function mapping inputs to the new Covariance matrix P_new
    """
    
    # Reconstruct the SymForce Rot3 object from the raw [w, x, y, z] input
    quat = sf.Rot3(sf.Quaternion(xyz=sf.V3(q[1], q[2], q[3]), w=q[0]))
    
    state = Values(
        quat_nominal=quat,
        vel=vel, pos=pos, gyro_bias=gyro_bias, accel_bias=accel_bias
    )
    
    g = sf.Symbol("g")

    state_error = Values(
        theta = sf.V3.symbolic("delta_theta"),
        vel = sf.V3.symbolic("delta_v"),
        pos = sf.V3.symbolic("delta_p"),
        gyro_bias = sf.V3.symbolic("delta_w_b"),
        accel_bias = sf.V3.symbolic("delta_a_b")
    )

    # True state kinematics
    state_t = Values()
    for key in state.keys():
        if key == "quat_nominal":
            # FIX: Right-Perturbation (Body frame error)
            state_t["quat_nominal"] = state["quat_nominal"] * sf.Rot3(sf.Quaternion(xyz=(state_error["theta"] / 2), w=1))
        else:
            state_t[key] = state[key] + state_error[key]

    noise = Values(
        accel = sf.V3.symbolic("a_n"),
        gyro = sf.V3.symbolic("w_n"),
    )

    input_t = Values(
        accel = accel - state_t["accel_bias"] - noise["accel"],
        gyro = gyro - state_t["gyro_bias"] - noise["gyro"]
    )

    R_t = state_t["quat_nominal"]
    state_t_pred = state_t.copy()
    state_t_pred["quat_nominal"] = state_t["quat_nominal"] * sf.Rot3(sf.Quaternion(xyz=(input_t["gyro"] * dt / 2), w=1))
    state_t_pred["vel"] = state_t["vel"] + (R_t * input_t["accel"] + sf.V3(0, 0, g)) * dt
    state_t_pred["pos"] = state_t["pos"] + state_t["vel"] * dt

    # Nominal state kinematics
    input = Values(
        accel = accel - state["accel_bias"],
        gyro = gyro - state["gyro_bias"]
    )

    R = state["quat_nominal"]
    state_pred = state.copy()
    state_pred["quat_nominal"] = state["quat_nominal"] * sf.Rot3(sf.Quaternion(xyz=(input["gyro"] * dt / 2), w=1))
    state_pred["vel"] = state["vel"] + (R * input["accel"] + sf.V3(0, 0, g)) * dt
    state_pred["pos"] = state["pos"] + state["vel"] * dt

    # Error state kinematics
    state_error_pred = Values()
    for key in state_error.keys():
        if key == "theta":
            # Extract Right-Perturbation (q_nom_inv * q_true)
            delta_q = sf.Quaternion.from_storage(state_pred["quat_nominal"].to_storage()).conj() * sf.Quaternion.from_storage(state_t_pred["quat_nominal"].to_storage())
            state_error_pred["theta"] = 2 * sf.V3(delta_q.x, delta_q.y, delta_q.z) 
        else:
            state_error_pred[key] = state_t_pred[key] - state_pred[key]

    # Simplify angular error state prediction
    for i in range(state_error_pred["theta"].storage_dim()):
        state_error_pred["theta"][i] = sp.expand(state_error_pred["theta"][i]).subs(dt**2, 0)
        q_est = sf.Quaternion.from_storage(state["quat_nominal"].to_storage())
        state_error_pred["theta"][i] = sp.factor(state_error_pred["theta"][i]).subs(q_est.w**2 + q_est.x**2 + q_est.y**2 + q_est.z**2, 1)

    zero_state_error = {state_error[key]: state_error[key].zero() for key in state_error.keys()}
    zero_noise = {noise[key]: noise[key].zero() for key in noise.keys()}

    # State propagation jacobian (F) and Noise Jacobian (G)
    A = VTangent(state_error_pred.to_storage()).jacobian(state_error).subs(zero_state_error).subs(zero_noise)
    G = VTangent(state_error_pred.to_storage()).jacobian(noise).subs(zero_state_error).subs(zero_noise)

    # Covariance propagation: F * P * F_T + G * Q * G_T
    var_u = sf.Matrix.diag([accel_var[0], accel_var[1], accel_var[2], gyro_var, gyro_var, gyro_var])
    P_new = A * P * A.T + G * var_u * G.T

    # Matrix is symmetric, only compute upper triangle
    for index in range(state.tangent_dim()):
        for j in range(state.tangent_dim()):
            if index > j:
                P_new[index,j] = 0

    return P_new

def update_stationary(
    q: sf.V4,       # [w, x, y, z]
    vel: sf.V3,
    pos: sf.V3,
    gyro_bias: sf.V3,
    accel_bias: sf.V3,
    P: MTangent,
    gyro_raw: sf.V3,
    vel_var: sf.V3,
    gyro_var: sf.V3,
    epsilon: sf.Scalar
) -> T.Tuple[sf.V4, sf.V3, sf.V3, sf.V3, sf.V3, MTangent]:
    """
    Zero-Velocity and Zero-Rotation Measurement Update.
    """
    quat = sf.Rot3(sf.Quaternion(xyz=sf.V3(q[1], q[2], q[3]), w=q[0]))

    # 1. Innovation (y = z_meas - z_pred)
    # The pad is stationary, so true velocity is 0 and true rotation is 0.
    # z_pred_vel = vel, so y_vel = 0 - vel
    # z_pred_gyro = gyro_raw - gyro_bias (since gyro_raw = true_gyro + bias + noise)
    y = sf.V6(
        0 - vel[0], 0 - vel[1], 0 - vel[2],
        gyro_raw[0] - gyro_bias[0], gyro_raw[1] - gyro_bias[1], gyro_raw[2] - gyro_bias[2]
    )

    # 2. Measurement Jacobian (H) [6x15 matrix]
    # Maps how changes in the 15D error state affect our 6D measurement prediction
    H = sf.Matrix.zeros(6, 15)
    H[0, 3] = 1; H[1, 4] = 1; H[2, 5] = 1   # velocity error impacts velocity measurement
    H[3, 9] = 1; H[4, 10] = 1; H[5, 11] = 1 # gyro bias error impacts gyro measurement

    # 3. Measurement Noise Matrix (R)
    R = sf.Matrix.diag([vel_var[0], vel_var[1], vel_var[2], gyro_var[0], gyro_var[1], gyro_var[2]])

    # 4. Kalman Gain (K = P * H^T * (H * P * H^T + R)^-1)
    S = H * P * H.T + R
    K_raw = P * H.T * S.inv()

    # FIX: Gate the accelerometer bias update.
    # While stationary, the accel bias is unobservable due to gravity entanglement.
    # We force the Kalman Gain for the accel_bias rows (12, 13, 14) to exactly 0.
    K = sf.Matrix.zeros(15, 6)
    for i in range(15):
        if i < 12: # Keep attitude, velocity, pos, and gyro_bias gains active
            for j in range(6):
                K[i, j] = K_raw[i, j]
        # Rows 12, 13, 14 remain 0, preventing accel_bias from updating

    # 5. Compute Error State (dx = K * y)
    dx = K * y

    # 6. Update Covariance (P_new = (I - K * H) * P)
    I = sf.Matrix.eye(15)
    P_new = (I - K * H) * P

    # 7. Inject Error State into Nominal State
    d_theta = sf.V3(dx[0], dx[1], dx[2])
    q_new_rot = quat * sf.Rot3.from_tangent(d_theta, epsilon=epsilon)
    q_new_storage = q_new_rot.to_storage()
    q_new = sf.V4(q_new_storage[3], q_new_storage[0], q_new_storage[1], q_new_storage[2])

    vel_new = vel + sf.V3(dx[3], dx[4], dx[5])
    pos_new = pos + sf.V3(dx[6], dx[7], dx[8])
    bg_new = gyro_bias + sf.V3(dx[9], dx[10], dx[11])
    ba_new = accel_bias + sf.V3(dx[12], dx[13], dx[14])

    return q_new, vel_new, pos_new, bg_new, ba_new, P_new


def update_baro(
    q: sf.V4, vel: sf.V3, pos: sf.V3,
    gyro_bias: sf.V3, accel_bias: sf.V3,
    P: MTangent,
    baro_alt: sf.Scalar, baro_var: sf.Scalar,
    epsilon: sf.Scalar
) -> T.Tuple[sf.V4, sf.V3, sf.V3, sf.V3, sf.V3, MTangent]:
    """
    Barometric altitude update: observes altitude = -pos[2] (NED, down-positive).
    Innovation: y = baro_alt - predicted_alt = baro_alt - (-pos[2])
    H is [1x15] with H[0,8] = -1  (d(-p_z)/d(delta_p_z) = -1)
    """
    quat = sf.Rot3(sf.Quaternion(xyz=sf.V3(q[1], q[2], q[3]), w=q[0]))

    # innovation: measured altitude minus predicted altitude (-pos_z)
    y = sf.V1(baro_alt - (-pos[2]))

    H = sf.Matrix.zeros(1, 15)
    H[0, 8] = -1  # altitude = -p_z  =>  d(alt)/d(delta_p_z) = -1

    R = sf.Matrix([[baro_var]])
    S = H * P * H.T + R
    K = P * H.T * S.inv()
    dx = K * y
    I = sf.Matrix.eye(15)
    P_new = (I - K * H) * P

    d_theta = sf.V3(dx[0], dx[1], dx[2])
    q_new_rot     = quat * sf.Rot3.from_tangent(d_theta, epsilon=epsilon)
    q_new_storage = q_new_rot.to_storage()
    q_new = sf.V4(q_new_storage[3], q_new_storage[0], q_new_storage[1], q_new_storage[2])

    vel_new = vel + sf.V3(dx[3], dx[4], dx[5])
    pos_new = pos + sf.V3(dx[6], dx[7], dx[8])
    bg_new  = gyro_bias  + sf.V3(dx[9],  dx[10], dx[11])
    ba_new  = accel_bias + sf.V3(dx[12], dx[13], dx[14])

    return q_new, vel_new, pos_new, bg_new, ba_new, P_new


def update_mag(
    q: sf.V4,       # [w, x, y, z] body-to-NED
    vel: sf.V3,
    pos: sf.V3,
    gyro_bias: sf.V3,
    accel_bias: sf.V3,
    P: MTangent,
    mag_body: sf.V3,      # measured body-frame field (Gauss)
    mag_ref_ned: sf.V3,   # reference NED field from WMM (Gauss)
    mag_var: sf.V3,       # per-axis measurement noise variance (Gauss²)
    epsilon: sf.Scalar
) -> T.Tuple[sf.V4, sf.V3, sf.V3, sf.V3, sf.V3, MTangent]:
    """
    3-axis magnetometer update.

    Observation model:
        h(q) = R_ned_to_body * mag_ref_ned = q.inverse() * mag_ref_ned

    Innovation:
        y = mag_body - h(q)

    H matrix is [3x15], non-zero only in the attitude columns (0-2):
        H_att = skew(h(q))

    This follows the right-perturbation convention consistent with the other
    update functions (dx injection uses quat * Rot3.from_tangent(dθ)).

    Physical interpretation:
        The magnetometer tells the EKF the direction of Earth's magnetic field
        as seen from the body. Comparing this to the known NED reference
        corrects the attitude, primarily yaw (heading), which is otherwise
        unobservable from GPS/Baro/IMU alone.
    """
    quat = sf.Rot3(sf.Quaternion(xyz=sf.V3(q[1], q[2], q[3]), w=q[0]))

    # Predicted measurement: rotate reference NED field into body frame
    mag_pred = quat.inverse() * mag_ref_ned

    # Innovation: measured - predicted
    y = mag_body - mag_pred

    # H matrix [3x15]:
    # H_att = d(h(q))/d(dθ) = skew(mag_pred)   (see derivation in roadmap)
    # All other columns are zero — mag does not directly observe vel, pos, or bias.
    def _skew(v):
        M = sf.Matrix.zeros(3, 3)
        M[0, 1] = -v[2]; M[0, 2] =  v[1]
        M[1, 0] =  v[2]; M[1, 2] = -v[0]
        M[2, 0] = -v[1]; M[2, 1] =  v[0]
        return M

    H = sf.Matrix.zeros(3, 15)
    H_att = _skew(mag_pred)
    for i in range(3):
        for j in range(3):
            H[i, j] = H_att[i, j]

    R = sf.Matrix.diag([mag_var[0], mag_var[1], mag_var[2]])
    S = H * P * H.T + R
    K = P * H.T * S.inv()
    dx = K * y

    I = sf.Matrix.eye(15)
    P_new = (I - K * H) * P

    d_theta = sf.V3(dx[0], dx[1], dx[2])
    q_new_rot     = quat * sf.Rot3.from_tangent(d_theta, epsilon=epsilon)
    q_new_storage = q_new_rot.to_storage()
    q_new = sf.V4(q_new_storage[3], q_new_storage[0], q_new_storage[1], q_new_storage[2])

    vel_new = vel + sf.V3(dx[3], dx[4], dx[5])
    pos_new = pos + sf.V3(dx[6], dx[7], dx[8])
    bg_new  = gyro_bias  + sf.V3(dx[9],  dx[10], dx[11])
    ba_new  = accel_bias + sf.V3(dx[12], dx[13], dx[14])

    return q_new, vel_new, pos_new, bg_new, ba_new, P_new


def update_gps(
    q: sf.V4, vel: sf.V3, pos: sf.V3,
    gyro_bias: sf.V3, accel_bias: sf.V3,
    P: MTangent,
    gps_pos: sf.V3, gps_vel: sf.V3,
    pos_var: sf.V3, vel_var: sf.V3,
    epsilon: sf.Scalar
) -> T.Tuple[sf.V4, sf.V3, sf.V3, sf.V3, sf.V3, MTangent]:
    """
    6-DOF GPS update: observes [pos_ned (3D), vel_ned (3D)].
    H is [6x15]:
      rows 0-2  →  pos_ned (state cols 6-8)
      rows 3-5  →  vel_ned (state cols 3-5)
    """
    quat = sf.Rot3(sf.Quaternion(xyz=sf.V3(q[1], q[2], q[3]), w=q[0]))

    y = sf.V6(
        gps_pos[0] - pos[0], gps_pos[1] - pos[1], gps_pos[2] - pos[2],
        gps_vel[0] - vel[0], gps_vel[1] - vel[1], gps_vel[2] - vel[2]
    )

    H = sf.Matrix.zeros(6, 15)
    # GPS pos observes p_ned (state indices 6-8)
    H[0, 6] = 1; H[1, 7] = 1; H[2, 8] = 1
    # GPS vel observes v_ned (state indices 3-5)
    H[3, 3] = 1; H[4, 4] = 1; H[5, 5] = 1

    R = sf.Matrix.diag([pos_var[0], pos_var[1], pos_var[2],
                        vel_var[0], vel_var[1], vel_var[2]])
    S = H * P * H.T + R
    K = P * H.T * S.inv()
    dx = K * y
    I = sf.Matrix.eye(15)
    P_new = (I - K * H) * P

    d_theta = sf.V3(dx[0], dx[1], dx[2])
    q_new_rot     = quat * sf.Rot3.from_tangent(d_theta, epsilon=epsilon)
    q_new_storage = q_new_rot.to_storage()
    q_new = sf.V4(q_new_storage[3], q_new_storage[0], q_new_storage[1], q_new_storage[2])

    vel_new = vel + sf.V3(dx[3], dx[4], dx[5])
    pos_new = pos + sf.V3(dx[6], dx[7], dx[8])
    bg_new  = gyro_bias  + sf.V3(dx[9],  dx[10], dx[11])
    ba_new  = accel_bias + sf.V3(dx[12], dx[13], dx[14])

    return q_new, vel_new, pos_new, bg_new, ba_new, P_new


if __name__ == "__main__":
    print("Deriving 15-state ESKF covariance equations...")

    # Output directory is always estimator/gen/ regardless of where the script
    # is invoked from (CMake, terminal, CI).
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.abspath(os.path.join(script_dir, "../gen"))

    config = CppConfig(
        use_eigen_types=True,
        explicit_template_instantiation_types=["float"]
    )

    codegen = Codegen.function(predict_covariance, config=config)
    codegen_data = codegen.generate_function(output_dir=output_dir)

    codegen_update = Codegen.function(update_stationary, config=config)
    codegen_update.generate_function(output_dir=output_dir)

    codegen_baro = Codegen.function(update_baro, config=config)
    codegen_baro.generate_function(output_dir=output_dir)

    codegen_gps = Codegen.function(update_gps, config=config)
    codegen_gps.generate_function(output_dir=output_dir)

    codegen_mag = Codegen.function(update_mag, config=config)
    codegen_mag.generate_function(output_dir=output_dir)

    print(f"Generated C++ equations in {codegen_data.generated_files[0]}")
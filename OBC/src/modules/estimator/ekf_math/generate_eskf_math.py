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
            state_t["quat_nominal"] = sf.Rot3(sf.Quaternion(xyz=(state_error["theta"] / 2), w=1)) * state["quat_nominal"]
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
            delta_q = sf.Quaternion.from_storage(state_t_pred["quat_nominal"].to_storage()) * sf.Quaternion.from_storage(state_pred["quat_nominal"].to_storage()).conj()
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

if __name__ == "__main__":
    print("Deriving 15-state ESKF covariance equations...")
    
    # Configure code generator to output pure scalar flat arrays, avoiding Eigen objects
    config = CppConfig(
        use_eigen_types=True,
        explicit_template_instantiation_types=["float"] 
    )
    
    codegen = Codegen.function(predict_covariance, config=config)
    codegen_data = codegen.generate_function(output_dir="gen")
    
    print(f"Generated C++ equations in {codegen_data.generated_files[0]}")
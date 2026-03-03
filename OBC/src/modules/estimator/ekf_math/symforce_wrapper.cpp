#include "symforce_wrapper.h"
#include <Eigen/Core>
#include "sym/predict_covariance.h"
#include "sym/update_stationary.h"
#include "sym/update_baro.h"
#include "sym/update_gps.h"

extern "C" {

void symforce_predict_covariance(
    float* P,
    const float* q,
    const float* vel,
    const float* pos,
    const float* gyro_bias,
    const float* accel_bias,
    const float* accel,
    const float* accel_var,
    const float* gyro,
    const float gyro_var,
    const float dt
) {
    // Wrap the raw pointers with Eigen::Map for zero-copy access
    Eigen::Map<Eigen::Matrix<float, 15, 15>> P_map(P);
    Eigen::Map<const Eigen::Matrix<float, 4, 1>> q_map(q);
    Eigen::Map<const Eigen::Matrix<float, 3, 1>> vel_map(vel);
    Eigen::Map<const Eigen::Matrix<float, 3, 1>> pos_map(pos);
    Eigen::Map<const Eigen::Matrix<float, 3, 1>> bg_map(gyro_bias);
    Eigen::Map<const Eigen::Matrix<float, 3, 1>> ba_map(accel_bias);
    Eigen::Map<const Eigen::Matrix<float, 3, 1>> accel_map(accel);
    Eigen::Map<const Eigen::Matrix<float, 3, 1>> accel_var_map(accel_var);
    Eigen::Map<const Eigen::Matrix<float, 3, 1>> gyro_map(gyro);

    // Call the generated SymForce template.
    // predict_covariance intentionally stores only the upper triangle for
    // computation efficiency.  Symmetrize immediately so that the update
    // functions always receive a full positive-definite P and the lower-
    // triangle cross-covariance terms (e.g. P[bias, vel]) are available
    // for GPS/Baro Kalman gain computation.
    Eigen::Matrix<float,15,15> P_pred = sym::PredictCovariance<float>(
        q_map, vel_map, pos_map, bg_map, ba_map, P_map,
        accel_map, accel_var_map, gyro_map, gyro_var, dt
    );
    P_map = P_pred.selfadjointView<Eigen::Upper>();
}

void symforce_update_stationary(
    float* P, float* q, float* vel, float* pos, float* gyro_bias, float* accel_bias,
    const float* gyro_raw, const float* vel_var, const float* gyro_var
) {
    Eigen::Map<Eigen::Matrix<float, 15, 15>> P_map(P);
    Eigen::Map<Eigen::Matrix<float, 4, 1>> q_map(q);
    Eigen::Map<Eigen::Matrix<float, 3, 1>> vel_map(vel);
    Eigen::Map<Eigen::Matrix<float, 3, 1>> pos_map(pos);
    Eigen::Map<Eigen::Matrix<float, 3, 1>> bg_map(gyro_bias);
    Eigen::Map<Eigen::Matrix<float, 3, 1>> ba_map(accel_bias);

    Eigen::Map<const Eigen::Matrix<float, 3, 1>> gyro_raw_map(gyro_raw);
    Eigen::Map<const Eigen::Matrix<float, 3, 1>> vel_var_map(vel_var);
    Eigen::Map<const Eigen::Matrix<float, 3, 1>> gyro_var_map(gyro_var);

    // predict_covariance symmetrizes P after each prediction step, so P is
    // always a full symmetric matrix when an update is called — no extra
    // symmetrization needed here.
    Eigen::Matrix<float, 4, 1>   q_out;
    Eigen::Matrix<float, 3, 1>   vel_out, pos_out, bg_out, ba_out;
    Eigen::Matrix<float, 15, 15> P_out;

    sym::UpdateStationary<float>(
        q_map, vel_map, pos_map, bg_map, ba_map, P_map,
        gyro_raw_map, vel_var_map, gyro_var_map, 1e-6f,
        &q_out, &vel_out, &pos_out, &bg_out, &ba_out, &P_out
    );

    q_map   = q_out;
    vel_map = vel_out;
    pos_map = pos_out;
    bg_map  = bg_out;
    ba_map  = ba_out;
    P_map   = P_out;
}

void symforce_update_baro(
    float* P, float* q, float* vel, float* pos, float* gyro_bias, float* accel_bias,
    float baro_alt, float baro_var, float epsilon
) {
    Eigen::Map<Eigen::Matrix<float,15,15>> P_map(P);
    Eigen::Map<Eigen::Matrix<float,4,1>>   q_map(q);
    Eigen::Map<Eigen::Matrix<float,3,1>>   vel_map(vel);
    Eigen::Map<Eigen::Matrix<float,3,1>>   pos_map(pos);
    Eigen::Map<Eigen::Matrix<float,3,1>>   bg_map(gyro_bias);
    Eigen::Map<Eigen::Matrix<float,3,1>>   ba_map(accel_bias);

    // P is always full-symmetric after predict_covariance — no symmetrize needed.
    Eigen::Matrix<float,4,1>   q_out;
    Eigen::Matrix<float,3,1>   vel_out, pos_out, bg_out, ba_out;
    Eigen::Matrix<float,15,15> P_out;

    sym::UpdateBaro<float>(
        q_map, vel_map, pos_map, bg_map, ba_map, P_map,
        baro_alt, baro_var, epsilon,
        &q_out, &vel_out, &pos_out, &bg_out, &ba_out, &P_out
    );

    q_map   = q_out;
    vel_map = vel_out;
    pos_map = pos_out;
    bg_map  = bg_out;
    ba_map  = ba_out;
    P_map   = P_out;
}

void symforce_update_gps(
    float* P, float* q, float* vel, float* pos, float* gyro_bias, float* accel_bias,
    const float* gps_pos, const float* gps_vel,
    const float* pos_var, const float* vel_var,
    float epsilon
) {
    Eigen::Map<Eigen::Matrix<float,15,15>> P_map(P);
    Eigen::Map<Eigen::Matrix<float,4,1>>   q_map(q);
    Eigen::Map<Eigen::Matrix<float,3,1>>   vel_map(vel);
    Eigen::Map<Eigen::Matrix<float,3,1>>   pos_map(pos);
    Eigen::Map<Eigen::Matrix<float,3,1>>   bg_map(gyro_bias);
    Eigen::Map<Eigen::Matrix<float,3,1>>   ba_map(accel_bias);

    Eigen::Map<const Eigen::Matrix<float,3,1>> gps_pos_map(gps_pos);
    Eigen::Map<const Eigen::Matrix<float,3,1>> gps_vel_map(gps_vel);
    Eigen::Map<const Eigen::Matrix<float,3,1>> pos_var_map(pos_var);
    Eigen::Map<const Eigen::Matrix<float,3,1>> vel_var_map(vel_var);

    // P is always full-symmetric after predict_covariance — no symmetrize needed.
    Eigen::Matrix<float,4,1>   q_out;
    Eigen::Matrix<float,3,1>   vel_out, pos_out, bg_out, ba_out;
    Eigen::Matrix<float,15,15> P_out;

    sym::UpdateGps<float>(
        q_map, vel_map, pos_map, bg_map, ba_map, P_map,
        gps_pos_map, gps_vel_map, pos_var_map, vel_var_map, epsilon,
        &q_out, &vel_out, &pos_out, &bg_out, &ba_out, &P_out
    );

    q_map   = q_out;
    vel_map = vel_out;
    pos_map = pos_out;
    bg_map  = bg_out;
    ba_map  = ba_out;
    P_map   = P_out;
}

} // extern "C"

#include "symforce_wrapper.h"
#include <Eigen/Core>
#include "sym/predict_covariance.h"

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

    // Call the generated SymForce template
    // Note: The template returns the new P matrix by value. 
    // SymForce generated code is highly optimized and unrolled.
    P_map = sym::PredictCovariance<float>(
        q_map,
        vel_map,
        pos_map,
        bg_map,
        ba_map,
        P_map,
        accel_map,
        accel_var_map,
        gyro_map,
        gyro_var,
        dt
    );
}

} // extern "C"

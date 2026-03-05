#ifndef IMU_PREDICTOR_H
#define IMU_PREDICTOR_H

#include "../ekf_core.h"

/**
 * @brief Run the heavy SymForce EKF prediction step on the delayed state.
 *
 * @param ekf   Pointer to the EKF core
 * @param imu   Pointer to the IMU history frame from the ring buffer
 */
void imu_predict(ekf_core_t* ekf, const imu_history_t* imu);

#endif // IMU_PREDICTOR_H

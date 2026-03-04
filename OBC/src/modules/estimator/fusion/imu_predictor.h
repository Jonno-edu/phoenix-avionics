#ifndef IMU_PREDICTOR_H
#define IMU_PREDICTOR_H

#include "../ekf_core.h"
#include "norb/topic_defs/vehicle_imu.h" 

/**
 * @brief Run the EKF prediction step
 * 
 * @param ekf   Pointer to the EKF core
 * @param imu   Pointer to the incoming IMU message
 */
void imu_predict(ekf_core_t* ekf, const vehicle_imu_t* imu);

#endif // IMU_PREDICTOR_H

#ifndef IMU_TOOL_H
#define IMU_TOOL_H

#include "hoja_includes.h"
#include "math.h"

void imu_access_safe(imu_data_s *out);
void imu_quat_access_safe(quaternion_s *out);

void imu_fifo_push(imu_data_s *imu_data);
imu_data_s* imu_fifo_last();
void imu_pack_quat(mode_2_s *out);

#endif
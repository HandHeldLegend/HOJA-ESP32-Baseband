#include "imu_tool.h"
#include "math.h"

#define IMU_FIFO_COUNT 3
#define IMU_FIFO_IDX_MAX (IMU_FIFO_COUNT-1)
int _imu_fifo_idx = 0;
imu_data_s _imu_fifo[IMU_FIFO_COUNT];

quaternion_s _imu_quat_state = {.w = 1};
#define GYRO_SENS (2000.0f / 32768.0f)

imu_data_s* imu_fifo_last()
{
  return &(_imu_fifo[_imu_fifo_idx]);
}


void imu_pack_quat(mode_2_s *out)
{
  out->mode = 2;

  // Determine maximum quat component
  uint8_t max_index = 0;
  for(uint8_t i = 1; i < 4; i++)
  {
    if(_imu_quat_state.raw[i] > fabsf(_imu_quat_state.raw[max_index]))
      max_index = i;
  }

  out->max_index = max_index;

  int quaternion_30bit_components[3];

  // Exclude the max_index component from the component list, invert sign of the remaining components if it was negative. Scales the final result to a 30 bit fixed precision format where 0x40000000 is 1.0
  for (int i = 0; i < 3; ++i) {
      quaternion_30bit_components[i] = _imu_quat_state.raw[(max_index + i + 1) & 3] * 0x40000000 * (_imu_quat_state.raw[max_index] < 0 ? -1 : 1);
  }

  // Insert into the last sample components, do bit operations to account for split data
  out->last_sample_0 = quaternion_30bit_components[0] >> 10;

  out->last_sample_1l = ((quaternion_30bit_components[1] >> 10) & 0x7F);
  out->last_sample_1h = ((quaternion_30bit_components[1] >> 10) & 0x1FFF80) >> 7;

  out->last_sample_2l = ((quaternion_30bit_components[2] >> 10) & 0x3);
  out->last_sample_2h = ((quaternion_30bit_components[2] >> 10) & 0x1FFFFC) >> 2;

  // We only store one sample, so all deltas are 0
  out->delta_last_first_0 = 0;
  out->delta_last_first_1 = 0;
  out->delta_last_first_2l = 0;
  out->delta_last_first_2h = 0;
  out->delta_mid_avg_0 = 0;
  out->delta_mid_avg_1 = 0;
  out->delta_mid_avg_2 = 0;

  // Timestamps handling is still a bit unclear, these are the values that motion_data in no drifting 
  out->timestamp_start_l = _imu_quat_state.timestamp & 0x1;
  out->timestamp_start_h = (_imu_quat_state.timestamp  >> 1) & 0x3FF;
  out->timestamp_count = 3;

  out->accel_0.x = _imu_quat_state.ax;
  out->accel_0.y = _imu_quat_state.ay;
  out->accel_0.z = _imu_quat_state.az;

  // Increment for the next cycle
  _imu_quat_state.timestamp += 8;
}

void _imu_rotate_quaternion(quaternion_s *first, quaternion_s *second) {
    float w = first->w * second->w - first->x * second->x - first->y * second->y - first->z * second->z;
    float x = first->w * second->x + first->x * second->w + first->y * second->z - first->z * second->y;
    float y = first->w * second->y - first->x * second->z + first->y * second->w + first->z * second->x;
    float z = first->w * second->z + first->x * second->y - first->y * second->x + first->z * second->w;
    
    first->w = w;
    first->x = x;
    first->y = y;
    first->z = z;
}

#define SCALE_FACTOR 2000.0f / INT16_MAX * M_PI / 180.0f / 1000000.0f

void _imu_quat_normalize(quaternion_s *data)
{
  float norm_inverse = 1.0f / sqrtf(data->x * data->x + data->y * data->y + data->z * data->z + data->w * data->w);
  data->x *= norm_inverse;
  data->y *= norm_inverse;
  data->z *= norm_inverse;
  data->w *= norm_inverse;
}

void _imu_update_quaternion(imu_data_s *imu_data, uint64_t timestamp) {
    // Previous timestamp (in microseconds)
    static uint64_t prev_timestamp = 0;

    float dt = (timestamp - prev_timestamp);

    // GZ is TURNING left/right (steering axis)
    // GX is TILTING up/down (aim up/down)
    // GY is TILTING left/right

    // Convert gyro readings to radians/second
    float angle_x = (float)imu_data->gy * SCALE_FACTOR * dt; // GY
    float angle_y = (float)imu_data->gx * SCALE_FACTOR * dt; // GX
    float angle_z = (float)imu_data->gz * SCALE_FACTOR * dt; // GZ

    // Euler to quaternion (in a custom Nintendo way)
    double norm_squared = angle_x * angle_x + angle_y * angle_y + angle_z * angle_z;
    double first_formula = norm_squared * norm_squared / 3840.0f - norm_squared / 48 + 0.5;
    double second_formula = norm_squared * norm_squared / 384.0f - norm_squared / 8 + 1;

    quaternion_s newstate = {
      .x = angle_x * first_formula,
      .y = angle_y * first_formula,
      .z = angle_z * first_formula,
      .w = second_formula
    };

    _imu_rotate_quaternion(&_imu_quat_state, &newstate);

    _imu_quat_normalize(&_imu_quat_state);

    _imu_quat_state.ax = imu_data->ax;
    _imu_quat_state.ay = imu_data->ay;
    _imu_quat_state.az = imu_data->az;

    // Update the previous timestamp
    prev_timestamp = timestamp;
}

// Add data to our FIFO
void imu_fifo_push(imu_data_s *imu_data)
{
    int _i = (_imu_fifo_idx+1) % IMU_FIFO_COUNT;

    _imu_fifo[_i].ax = imu_data->ax;
    _imu_fifo[_i].ay = imu_data->ay;
    _imu_fifo[_i].az = imu_data->az;

    _imu_fifo[_i].gx = imu_data->gx;
    _imu_fifo[_i].gy = imu_data->gy;
    _imu_fifo[_i].gz = imu_data->gz;

    _imu_fifo_idx = _i;

    int64_t t = esp_timer_get_time();

    _imu_update_quaternion(imu_data, t);
}

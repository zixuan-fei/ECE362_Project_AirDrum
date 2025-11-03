// imu_io.h
#ifndef IMU_IO_H
#define IMU_IO_H

#include <stdint.h>

// 供外部调用的“读原始数据”函数（去掉了 static）
void read_accel_raw(int16_t* ax, int16_t* ay, int16_t* az);
void read_gyro_raw (int16_t* gx, int16_t* gy, int16_t* gz);

// bias 校准函数（你在 calibrate_bias_fun.c 里实现）
void calibrate_bias(float acc_bias[3], float gyr_bias[3]);

#endif // IMU_IO_H

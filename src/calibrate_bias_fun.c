#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "pico/stdlib.h"
#include "imu_io.h"



void calibrate_bias(float acc_bias[3], float gyr_bias[3])
{
    const int samples = 400;       // 约2秒 @200Hz
    int64_t sum_ax=0, sum_ay=0, sum_az=0;
    int64_t sum_gx=0, sum_gy=0, sum_gz=0;

    printf("=== Calibrating IMU... keep still ===\n");
    sleep_ms(300); // 稍等片刻稳定

    for (int i = 0; i < samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        read_accel_raw(&ax, &ay, &az);
        read_gyro_raw (&gx, &gy, &gz);

        sum_ax += ax;
        sum_ay += ay;
        sum_az += az;

        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        sleep_ms(5);  // 200Hz
    }

    acc_bias[0] = (float)sum_ax / samples;
    acc_bias[1] = (float)sum_ay / samples;
    acc_bias[2] = (float)sum_az / samples;
    gyr_bias[0] = (float)sum_gx / samples;
    gyr_bias[1] = (float)sum_gy / samples;
    gyr_bias[2] = (float)sum_gz / samples;

    printf("Bias calibration done.\n");
    printf("ACC bias: %.1f %.1f %.1f | GYR bias: %.1f %.1f %.1f\n",
           acc_bias[0], acc_bias[1], acc_bias[2],
           gyr_bias[0], gyr_bias[1], gyr_bias[2]);
}
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "imu_io.h"

// ---- I2C 引脚/端口 ----
#define I2C_PORT      i2c0
#define I2C_SDA_PIN   40     // GPIO40 -> SDA0
#define I2C_SCL_PIN   41     // GPIO41 -> SCL0
#define I2C_BAUD      (400 * 1000)   // 400kHz

// ---- LSM6DSOX 地址与寄存器 ----
#define LSM6DSOX_ADDR           0x6A   // SA0=0 -> 0x6A; 若 SA0=1 请改为 0x6B
#define LSM6DSOX_REG_WHO_AM_I   0x0F
#define LSM6DSOX_WHOAMI_VAL     0x6C

#define REG_CTRL1_XL            0x10   // 加速度计控制
#define REG_CTRL2_G             0x11   // 陀螺仪控制
#define REG_CTRL3_C             0x12   // IF_INC/重置等
#define REG_OUTX_L_G            0x22   // Gx LSB 起始，6字节
#define REG_OUTX_L_A            0x28   // Ax LSB 起始，6字节

static inline int16_t u8pair_to_i16(uint8_t lo, uint8_t hi) {
    return (int16_t)((hi << 8) | lo);
}

static void i2c_bus_init(void) {
    i2c_init(I2C_PORT, I2C_BAUD);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

static int i2c_write_reg_addr(uint8_t reg, bool nostop) {
    return i2c_write_blocking(I2C_PORT, LSM6DSOX_ADDR, &reg, 1, nostop);
}

static int read_reg(uint8_t reg, uint8_t *val) {
    if (i2c_write_reg_addr(reg, true) != 1) return PICO_ERROR_GENERIC;
    return (i2c_read_blocking(I2C_PORT, LSM6DSOX_ADDR, val, 1, false) == 1) ? 0 : PICO_ERROR_GENERIC;
}

static int write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return (i2c_write_blocking(I2C_PORT, LSM6DSOX_ADDR, buf, 2, false) == 2) ? 0 : PICO_ERROR_GENERIC;
}

static bool probe_whoami(void) {
    uint8_t who = 0;
    if (read_reg(LSM6DSOX_REG_WHO_AM_I, &who) == 0 && who == LSM6DSOX_WHOAMI_VAL) return true;
    printf("WHO_AM_I=0x%02X (expect 0x6C). Check SA0/addr/wiring/pull-ups.\n", who);
    return false;
}

// 打开自动地址自增 + 208Hz ODR：ACC ±2g，GYR 250 dps
static void enable_odo_208hz(void) {
    // CTRL3_C: IF_INC=1(位2), BDU=1(位6可选，锁定数据更新以避免撕裂)
    write_reg(REG_CTRL3_C, 0b01000100);
    // CTRL1_XL: ODR_XL=208Hz(0110<<4), FS_XL=±2g(00), BW/LPF 默认
    write_reg(REG_CTRL1_XL, 0b01100000);
    // CTRL2_G : ODR_G =208Hz(0110<<4), FS_G =250 dps(00)
    write_reg(REG_CTRL2_G,  0b01100000);
    sleep_ms(20);
}

void read_accel_raw(int16_t* ax, int16_t* ay, int16_t* az) {
    uint8_t buf[6];
    i2c_write_reg_addr(REG_OUTX_L_A, true);
    i2c_read_blocking(I2C_PORT, LSM6DSOX_ADDR, buf, 6, false);
    if (ax) *ax = u8pair_to_i16(buf[0], buf[1]);
    if (ay) *ay = u8pair_to_i16(buf[2], buf[3]);
    if (az) *az = u8pair_to_i16(buf[4], buf[5]);
}

void read_gyro_raw(int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t buf[6];
    i2c_write_reg_addr(REG_OUTX_L_G, true);
    i2c_read_blocking(I2C_PORT, LSM6DSOX_ADDR, buf, 6, false);
    if (gx) *gx = u8pair_to_i16(buf[0], buf[1]);
    if (gy) *gy = u8pair_to_i16(buf[2], buf[3]);
    if (gz) *gz = u8pair_to_i16(buf[4], buf[5]);
}

int main() {
    stdio_init_all();
    sleep_ms(300);

    printf("Init I2C on GP%d/GP%d @ %u Hz...\n", I2C_SDA_PIN, I2C_SCL_PIN, I2C_BAUD);
    i2c_bus_init();

    if (!probe_whoami()) {
        while (true) { sleep_ms(1000); }
    }

    enable_odo_208hz();
    printf("LSM6DSOX streaming @208Hz (ACC ±2g, GYR 250 dps)\n");

    //calibrate bias校准函数
    float acc_bias[3], gyr_bias[3];
    calibrate_bias(acc_bias, gyr_bias);


    // 想要“实时输出”，就在循环里读 → 打印
    while (true) {
        int16_t ax, ay, az, gx, gy, gz;
        read_accel_raw(&ax, &ay, &az);
        read_gyro_raw (&gx, &gy, &gz);

        ax -= acc_bias[0];
        ay -= acc_bias[1];
        az -= acc_bias[2];
        gx -= gyr_bias[0];
        gy -= gyr_bias[1];
        gz -= gyr_bias[2];

        // 原始码打印（单位：LSB）
        printf("ACC(lsb): %6d %6d %6d | GYR(lsb): %6d %6d %6d\n", ax, ay, az, gx, gy, gz);

        // 如需换算为物理量，解除注释：
        // ACC: ±2g 对应 ~0.061 mg/LSB  ≈ 0.000061 g/LSB
        // GYR: 250 dps 对应 ~8.75 mdps/LSB ≈ 0.00875 dps/LSB
        // float ax_g = ax * 0.000061f, ay_g = ay * 0.000061f, az_g = az * 0.000061f;
        // float gx_dps = gx * 0.00875f, gy_dps = gy * 0.00875f, gz_dps = gz * 0.00875f;
        // printf("ACC(g): %.3f %.3f %.3f | GYR(dps): %.2f %.2f %.2f\n",
        //        ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);

        sleep_ms(20);  // ~50Hz 打印；想看更快可以减小这个延时
    }
}

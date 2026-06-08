#include "main.h"
#include "IMU.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define MPU6050_ADDR  (0x68 << 1)
#define PWR_MGMT_1    0x6B
#define ACCEL_XOUT_H  0x3B

static double pitch_rate_filt = 0.0;
static double roll_rate_filt  = 0.0;
static double yaw_rate_filt   = 0.0;

static double gx_bias = 0.0;
static double gy_bias = 0.0;
static double gz_bias = 0.0;

void MPU6050_Init(void)
{
    uint8_t data = 0x00;
    for (int i = 0; i < 3; i++)
    {
        if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, 100) == HAL_OK)
            break;
        HAL_Delay(10);
    }

    // Set DLPF — register 0x1A (CONFIG)
    // Value 0x03 = 44Hz gyro bandwidth, good starting point for a drone
    // Options: 0x00=260Hz, 0x01=184Hz, 0x02=94Hz, 0x03=44Hz, 0x04=21Hz
    uint8_t dlpf = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, 1, &dlpf, 1, 100);
}

void MPU6050_Read(int16_t *ax, int16_t *ay, int16_t *az,
                  int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14];
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, 1, buf, 14, 100) != HAL_OK)
    {
        *ax = *ay = *az = *gx = *gy = *gz = 0;
        return;
    }

    *ax = (int16_t)(buf[0]  << 8 | buf[1]);
    *ay = (int16_t)(buf[2]  << 8 | buf[3]);
    *az = (int16_t)(buf[4]  << 8 | buf[5]);
    // buf[6]/buf[7] are the MPU6050 temperature registers and are intentionally ignored here
    // we continue reading gyro data from buf[8]..
    *gx = (int16_t)(buf[8]  << 8 | buf[9]);
    *gy = (int16_t)(buf[10] << 8 | buf[11]);
    *gz = (int16_t)(buf[12] << 8 | buf[13]);
}

void IMU_Init(void)
{
    MPU6050_Init();
}

void IMU_Calibrate(int samples)
{
    int16_t ax, ay, az, gx, gy, gz;

    double gx_sum = 0.0;
    double gy_sum = 0.0;
    double gz_sum = 0.0;

    for (int i = 0; i < samples; i++)
    {
        MPU6050_Read(&ax, &ay, &az, &gx, &gy, &gz);

        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;

        HAL_Delay(2);
    }

    gx_bias = gx_sum / samples;
    gy_bias = gy_sum / samples;
    gz_bias = gz_sum / samples;
}

IMUData IMU_GetAngles(double dt)
{
    // dt is provided by caller (main loop). Clamp for safety here as well.
    if (dt <= 0.0) dt = 0.001;
    if (dt > 0.05) dt = 0.05;

    static double pitch = 0.0;
    static double roll  = 0.0;
    static double yaw   = 0.0;
    static int initialized = 0;

    int16_t ax, ay, az, gx, gy, gz;
    MPU6050_Read(&ax, &ay, &az, &gx, &gy, &gz);

    // =========================
    // GRAVITY-BASED ATTITUDE (decoupled 3-axis formulas)
    // =========================
    double ax_f = (double)ax;
    double ay_f = (double)ay;
    double az_f = (double)az;
    double accel_pitch = atan2(ax_f, sqrt(ay_f*ay_f + az_f*az_f)) * 180.0 / M_PI;
    double accel_roll  = atan2(ay_f,  sqrt(ax_f*ax_f + az_f*az_f)) * 180.0 / M_PI;

    // initialize gyro state from first accel reading ONLY
    if (!initialized)
    {
        pitch = accel_pitch;
        roll  = accel_roll;
        yaw   = 0.0;
        initialized = 1;
    }

    // =========================
    // GYRO RATES
    // =========================
    // subtract calibrated gyro biases and convert to deg/s (MPU6050 scale)
    double gyro_pitch_rate = -(gy - gy_bias) / 131.0;
    double gyro_roll_rate  = (gx - gx_bias) / 131.0;

    // integrate gyro
    pitch += gyro_pitch_rate * dt;
    roll  += gyro_roll_rate  * dt;

    // complementary filter (gravity correction)
    pitch = 0.98 * pitch + 0.02 * accel_pitch;
    roll  = 0.98 * roll  + 0.02 * accel_roll;

    // =========================
    // YAW (gyro only)
    // =========================
    double gz_rate = (gz - gz_bias) / 131.0;

    // deadband
    if (fabs(gz_rate) < 0.15)
        gz_rate = 0.0;

    yaw += gz_rate * dt;

    // optional wrap
    if (yaw > 180.0) yaw -= 360.0;
    if (yaw < -180.0) yaw += 360.0;

    // =========================
    // RATES OUTPUT
    // =========================

    #define GYRO_LPF_ALPHA 0.1

    pitch_rate_filt = GYRO_LPF_ALPHA * gyro_pitch_rate + (1.0 - GYRO_LPF_ALPHA) * pitch_rate_filt;
    roll_rate_filt  = GYRO_LPF_ALPHA * gyro_roll_rate  + (1.0 - GYRO_LPF_ALPHA) * roll_rate_filt;
    yaw_rate_filt   = GYRO_LPF_ALPHA * gz_rate         + (1.0 - GYRO_LPF_ALPHA) * yaw_rate_filt;


    IMUData data = {pitch, roll, yaw,
                    yaw_rate_filt, pitch_rate_filt, roll_rate_filt};

    return data;
}

void IMU_PrintTelemetry(IMUData data)
{
    char msg[128];
    int p = (int)round(data.pitch * 100.0);
    int r = (int)round(data.roll  * 100.0);
    int y = (int)round(data.yaw   * 100.0);
    int len = snprintf(msg, sizeof(msg),
        "IMU data: pitch:%d.%02d roll:%d.%02d yaw:%d.%02d\r\n",
        p/100, abs(p%100),
        r/100, abs(r%100),
        y/100, abs(y%100));
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, (uint16_t)len, 100);
}

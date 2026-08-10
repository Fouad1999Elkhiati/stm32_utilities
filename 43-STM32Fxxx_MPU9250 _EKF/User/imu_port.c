#include "imu_port.h"
#include "magcal.h"
#include "tm_stm32_mpu9250.h"
#include "tm_stm32_delay.h"

#define DEG2RAD 0.017453292f

static TM_MPU9250_t dev;
static magcal_t     cal;
static uint32_t     last_ms;

/* Read the raw magnetometer counts and apply the scale factor.
 *
 * Works around a defect in tm_stm32_mpu9250.c: TM_MPU9250_ReadMag
 * writes only Mx_Raw/My_Raw/Mz_Raw and returns. The line applying
 * MMult -- present in ReadAcce and ReadGyro -- is absent, so
 * dev.Mx/My/Mz are never written by the library at all.
 *
 * Units come out as 0.1 uT (MMult = 10 * 4912 / 32768). The absolute
 * unit is irrelevant downstream because magcal normalises. */
static void read_mag_raw(float out[3])
{
    out[0] = (float)dev.Mx_Raw * dev.MMult;
    out[1] = (float)dev.My_Raw * dev.MMult;
    out[2] = (float)dev.Mz_Raw * dev.MMult;
}

int imu_port_init(void)
{
    /* TM_MPU9250_Init calls TM_I2C_Init internally at 400 kHz on I2C1.
       Harmless after TM_SSD1306_Init has done the same on the same bus,
       but it means the sensor must come up after the display. */
    if (TM_MPU9250_Init(&dev, TM_MPU9250_Device_0) != TM_MPU9250_Result_Ok)
        return 0;

    magcal_reset(&cal);
    last_ms = TM_DELAY_Time();
    return 1;
}

int imu_port_read(imu_sample_t *s)
{
    uint32_t now = TM_DELAY_Time();
    float raw[3];

    /* These three return values are meaningless -- all three functions
       are declared TM_MPU9250_Result_t but have no return statement.
       Those are the -Wreturn-type warnings at lines 280, 293 and 310.
       Do not test them. */
    TM_MPU9250_ReadAcce(&dev);
    TM_MPU9250_ReadGyro(&dev);
    TM_MPU9250_ReadMag(&dev);

    /* Library scales gyro to deg/s and accel to g. The filter wants
       rad/s; accel in g is already what ekf_update_accel expects,
       since it normalises internally and gates on magnitude near 1. */
    s->gyro[0] = dev.Gx * DEG2RAD;
    s->gyro[1] = dev.Gy * DEG2RAD;
    s->gyro[2] = dev.Gz * DEG2RAD;

    s->acc[0] = dev.Ax;
    s->acc[1] = dev.Ay;
    s->acc[2] = dev.Az;

    read_mag_raw(raw);

    if (cal.valid) {
        magcal_apply(&cal, raw, s->mag);
        s->mag_valid = 1;
    } else {
        s->mag[0] = raw[0];
        s->mag[1] = raw[1];
        s->mag[2] = raw[2];
        s->mag_valid = 0;
    }

    s->dt   = (now - last_ms) * 0.001f;
    last_ms = now;

    /* Reject the first sample after init and any implausible gap. */
    return (s->dt > 0.0f && s->dt < 0.5f);
}

void imu_port_cal_begin(void)
{
    magcal_reset(&cal);
}

int imu_port_cal_feed(void)
{
    float raw[3];

    TM_MPU9250_ReadMag(&dev);
    read_mag_raw(raw);
    magcal_add(&cal, raw[0], raw[1], raw[2]);

    return (int)cal.count;
}

int imu_port_cal_finish(void)
{
    return magcal_solve(&cal);
}

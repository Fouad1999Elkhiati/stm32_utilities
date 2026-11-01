/**	
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen MAJERLE
 * |  
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software, 
 * | and to permit persons to whom the Software is furnished to do so, 
 * | subject to the following conditions:
 * | 
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * | 
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */
#include "imu_port.h"
#include "magcal.h"
#include "tm_stm32_mpu9250.h"
#include "tm_stm32_delay.h"

#define DEG2RAD 0.017453292f

static TM_MPU9250_t dev;
static magcal_t     cal;
static uint32_t     last_ms;
static uint8_t      fail_run;      /* consecutive failed reads */
static uint32_t     fail_total;    /* failed reads since init */
static int          bus_ok;

/* Read the raw magnetometer counts and apply the scale factor.
 *
 * Works from Mx_Raw/My_Raw/Mz_Raw rather than dev.Mx/My/Mz so the axis
 * remap below sits in one visible place.
 *
 * Units come out as 0.1 uT (MMult = 10 * 4912 / 32768). The absolute
 * unit is irrelevant downstream because magcal normalises. */

/* AK8963 axes differ from the accel/gyro axes inside the MPU9250:
      body X = mag Y, body Y = mag X, body Z = -mag Z. */
static void read_mag_raw(float out[3])
{
     out[0] =  (float)dev.My_Raw * dev.MMult;   /* swap X <-> Y */
     out[1] =  (float)dev.Mx_Raw * dev.MMult;
     out[2] = -(float)dev.Mz_Raw * dev.MMult;   /* negate Z */
}

int imu_port_init(void)
{
    /* TM_MPU9250_Init calls TM_I2C_Init internally at 400 kHz on I2C1.
       Harmless after TM_SSD1306_Init has done the same on the same bus,
       but it means the sensor must come up after the display. */
    if (TM_MPU9250_Init(&dev, TM_MPU9250_Device_0) != TM_MPU9250_Result_Ok)
        return 0;

    magcal_reset(&cal);
    last_ms    = TM_DELAY_Time();
    fail_run   = 0;
    fail_total = 0;
    bus_ok     = 1;
    return 1;
}

/* Count one failed read. After IMU_PORT_FAIL_LIMIT in a row, free the
   bus (a slave may be holding SDA low after an interrupted transfer)
   and bring the sensor back up. The magnetometer calibration is kept:
   it describes the board, not the sensor's register state. */
static void note_failure(void)
{
    bus_ok = 0;
    fail_total++;

    if (++fail_run < IMU_PORT_FAIL_LIMIT)
        return;

    fail_run = 0;
    TM_I2C_BusRecover(MPU9250_I2C, MPU9250_I2C_PP, MPU9250_I2C_CLOCK);
    TM_MPU9250_Init(&dev, TM_MPU9250_Device_0);
}

static void note_success(void)
{
    fail_run = 0;
    bus_ok   = 1;
}

int imu_port_read(imu_sample_t *s)
{
    uint32_t now = TM_DELAY_Time();
    TM_MPU9250_Result_t mag_res;
    float raw[3];

    /* Accel and gyro are both required for a usable sample. On failure
       leave *s untouched and do not advance last_ms, so the next good
       sample's dt spans the whole gap. */
    if (TM_MPU9250_ReadAcce(&dev) != TM_MPU9250_Result_Ok ||
        TM_MPU9250_ReadGyro(&dev) != TM_MPU9250_Result_Ok) {
        note_failure();
        return 0;
    }

    /* The magnetometer is optional per sample: NoNewData is the normal
       case between AK8963 updates. A bus error still counts as a fault,
       but accel and gyro from this sample are good and are used. */
    mag_res = TM_MPU9250_ReadMag(&dev);
    if (mag_res == TM_MPU9250_Result_Error)
        note_failure();
    else
        note_success();

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

    s->mag_new = (mag_res == TM_MPU9250_Result_Ok);

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

    /* Add only fresh samples. Feeding the same reading repeatedly
       between AK8963 updates would bias the ellipsoid fit. */
    if (TM_MPU9250_ReadMag(&dev) == TM_MPU9250_Result_Ok) {
        read_mag_raw(raw);
        magcal_add(&cal, raw[0], raw[1], raw[2]);
    }

    return (int)cal.count;
}

int imu_port_cal_finish(void)
{
    return magcal_solve(&cal);
}

int imu_port_bus_ok(void)
{
    return bus_ok;
}

uint32_t imu_port_error_count(void)
{
    return fail_total;
}

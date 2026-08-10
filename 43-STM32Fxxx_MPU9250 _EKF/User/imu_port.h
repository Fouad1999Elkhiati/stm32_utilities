/**
 * Hardware adaptation layer -- the only file in this project that
 * includes a tm_stm32 header.
 *
 * Turns MPU9250 register reads into calibrated SI quantities. Porting
 * to a different MCU or IMU means rewriting this file and nothing else.
 */

#ifndef IMU_PORT_H
#define IMU_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float gyro[3];    /* rad/s */
    float acc[3];     /* g */
    float mag[3];     /* calibrated unit vector, or raw if uncalibrated */
    float dt;         /* seconds since the previous sample */
    int   mag_valid;  /* 0 until a calibration has been solved */
} imu_sample_t;

/* Brings up the MPU9250. Call after TM_DELAY_Init and after
   TM_SSD1306_Init -- both devices share I2C1, and the display's init
   is what brings the bus up. Returns 1 on success. */
int imu_port_init(void);

/* Reads all three sensors and fills s. Returns 0 if dt was implausible,
   in which case the sample should be skipped rather than fed forward. */
int imu_port_read(imu_sample_t *s);

/* --- Magnetometer calibration lifecycle --- */

/* Clear the accumulators and begin collecting. */
void imu_port_cal_begin(void);

/* Fold one magnetometer sample in. Returns the running sample count.
   Rotate the board through as many orientations as possible while
   this is being called. */
int imu_port_cal_feed(void);

/* Solve the fit and start applying it. Returns 1 on success; on
   failure the calibration stays inactive and mag_valid remains 0. */
int imu_port_cal_finish(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_PORT_H */

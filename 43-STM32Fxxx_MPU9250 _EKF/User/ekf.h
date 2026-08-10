/**
 * Hardware-agnostic attitude EKF (multiplicative formulation).
 *
 * Six error states: 3 attitude errors + 3 gyro biases.
 * The quaternion is carried outside the state vector and corrected
 * multiplicatively, so its norm constraint is never violated.
 *
 * This file has no platform dependencies. It compiles unchanged on a
 * Cortex-M, on a PC for offline testing, or anywhere else with a C99
 * compiler and math.h.
 *
 * Frame convention: q rotates body -> navigation. Navigation frame is
 * ENU-like, with gravity read as +1 g on Z when the board is level --
 * matching what an MPU9250 reports lying flat.
 */

#ifndef EKF_H
#define EKF_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float gyro_noise;   /* gyro white noise,        rad/s/sqrt(Hz) */
    float bias_noise;   /* bias random walk,       rad/s^2/sqrt(Hz) */
    float accel_noise;  /* accel measurement noise, normalised units */
    float mag_noise;    /* mag measurement noise,   normalised units */
    float mag_ref[3];   /* unit magnetic reference vector, nav frame */
} ekf_cfg_t;

typedef struct {
    float q[4];         /* w x y z, body -> nav, unit norm */
    float bias[3];      /* estimated gyro bias, rad/s */
    float P[36];        /* 6x6 error covariance, row-major */
    ekf_cfg_t cfg;
} ekf_t;

/* Sets a level attitude, zero bias, and a diagonal initial covariance. */
void ekf_init(ekf_t *f, const ekf_cfg_t *cfg);

/* Propagate with a gyro sample. gyro in rad/s, dt in seconds. */
void ekf_predict(ekf_t *f, const float gyro[3], float dt);

/* Correct roll and pitch from the gravity vector.
   acc may be in any consistent unit -- it is normalised internally.
   Returns 0 and does nothing if the sample is not close to 1 g, which
   rejects updates during linear acceleration. */
int ekf_update_accel(ekf_t *f, const float acc[3]);

/* Correct heading from the magnetic field vector.
   mag must already be hard/soft-iron calibrated. Returns 0 on a
   degenerate sample. */
int ekf_update_mag(ekf_t *f, const float mag[3]);

/* Extract Euler angles in radians. Any pointer may be NULL. */
void ekf_euler(const ekf_t *f, float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif

#endif /* EKF_H */

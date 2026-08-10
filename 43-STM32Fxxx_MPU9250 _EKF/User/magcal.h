/**
 * Magnetometer hard/soft-iron calibration by ellipsoid fitting.
 *
 * Raw magnetometer readings trace an off-centre, stretched ellipsoid
 * rather than a sphere: hard-iron effects (nearby ferrous material)
 * shift the centre, soft-iron effects scale the axes. Uncalibrated,
 * the heading derived from them is unusable.
 *
 * This fits the axis-aligned ellipsoid
 *
 *     x^2 + B y^2 + C z^2 + P x + Q y + R z + D = 0
 *
 * by least squares. Each sample contributes one row to the design
 * matrix; the normal equations M'M v = M'w form a 6x6 dense system,
 * solved once with Gaussian elimination.
 *
 * The accumulators are fixed size, so memory does not grow with the
 * number of samples -- samples are folded in and discarded.
 *
 * Usage:
 *     magcal_t c;
 *     magcal_reset(&c);
 *     while (collecting)  magcal_add(&c, mx, my, mz);   // rotate in all axes
 *     if (magcal_solve(&c)) magcal_apply(&c, raw, out);
 *
 * No platform dependencies beyond Gauss_elimination.
 */

#ifndef MAGCAL_H
#define MAGCAL_H

#ifdef __cplusplus
extern "C" {
#endif

#define MAGCAL_MIN_SAMPLES  100

typedef struct {
    double  ata[36];      /* 6x6 normal matrix, accumulated */
    double  atb[6];       /* 6-vector right-hand side */
    unsigned count;

    /* Results, valid after magcal_solve returns 1 */
    float offset[3];      /* hard-iron centre */
    float scale[3];       /* per-axis normalisation */
    int   valid;
} magcal_t;

/* Clear all accumulators. Call before collecting samples. */
void magcal_reset(magcal_t *c);

/* Fold one raw sample into the normal equations.
   Rotate the sensor through as many orientations as possible --
   a fit from samples clustered in one attitude is ill-conditioned. */
void magcal_add(magcal_t *c, float mx, float my, float mz);

/* Solve the 6x6 system and derive offset/scale.
   Returns 1 on success, 0 if too few samples or the fit is degenerate. */
int magcal_solve(magcal_t *c);

/* Apply a solved calibration. raw and out may alias.
   Output is normalised to roughly unit length. */
void magcal_apply(const magcal_t *c, const float raw[3], float out[3]);

#ifdef __cplusplus
}
#endif

#endif /* MAGCAL_H */

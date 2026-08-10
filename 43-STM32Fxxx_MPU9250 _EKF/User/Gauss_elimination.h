#ifndef GAUSS_ELIMINATION_H_
#define GAUSS_ELIMINATION_H_

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

/* Swaps the values of two floats through pointers. */
void swap(float *a, float *b);

/* Swaps rows i and j of an n-column matrix stored row-major. */
void ligne_swap(float* arr, int i, int j, int n);

/* Partial pivoting: searches rows [pivot, n) for the largest-magnitude entry
   in column `pivot` and swaps that row into the pivot position. Searching only
   downward leaves already-eliminated rows untouched; choosing the largest
   magnitude keeps the elimination ratios small and limits rounding error.
   Increments *swap_cnt when a swap occurs, so the determinant sign can be
   corrected afterwards. */
void partial_pivot(int pivot, float* a, float* b, float* tb, int n, int* swap_cnt);

/* Zeroes the column below the pivot, applying the same row operations to the
   augmented matrix b and the right-hand side tb. */
void matrix_elimination(int pivot, float* a, float* b, float* tb, int n);

/* Reduces a to upper-triangular form and returns its determinant, or 0 if the
   matrix is singular. Destroys a, and applies the same operations to b and tb.
   Must be called before inversion(). */
float determinant(float* a, float* b, float* tb, int n);

/* Back substitution. On return b holds the inverse of the original matrix and
   tb holds the solution of Ax = b. Only valid if determinant() was non-zero. */
void inversion(float* a, float* b, float* tb, int n);

/*------------------------------------------*/
#endif

/**
 * @author  Fouad El khiati
 * @email   tilen@majerle.eu
 * @website http://stm32f4-discovery.net
 * @link    http://stm32f4-discovery.net/2015/10/hal-library-29-analog-to-digital-converter-for-stm32fxxx/
 * @version v1.0
 * @ide     Keil uVision
 * @license MIT
 * @brief   Gaussian elimination library STM32Fxxx
 *	
@verbatim
   ----------------------------------------------------------------------
    Copyright (c) 2016 Tilen MAJERLE

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software, 
    and to permit persons to whom the Software is furnished to do so, 
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
@endverbatim
 */
#ifndef GAUSS_ELIMINATION_H_
#define GAUSS_ELIMINATION_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup TM_STM32Fxxx_HAL_Libraries
 * @{
 */

/**
 * @defgroup Gauss_elimination
 * @brief    
 * @{
 *
 * This library allows you to operate Matrix inversion and linear system solving.
 * It works on STM32F0xx, STM32F4xx and STM32F7xx devices
 *
 * \par How it works
 * 
 * it implements standard gauss method on systems of any size n.
 *
 * \par 
 *
 * 
\verbatim

\endverbatim
 *
 * \par Reading Battery Voltage
 *
 * Circuit for battery is integrated inside STM32 device.
 * It can measure a battery, connected to VBAT pin, meant for RTC and backup purpose only.
 *
 * \par Changelog
 *
@verbatim
 Version 1.0
  - First release
@endverbatim
 *
 * \par Dependencies
 *
@verbatim
 - stdio.h
 - math.h
 - stdlib.h
 - time.h
@endverbatim
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

/**
 * @defgroup GAUSS_elimination_macros
 * @brief    Library defines
 * @{ (here)
 */

/**
 * @defgroup GAUSS_elimination_functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Swaps the values of two floats through pointers.
 * @param  *a: first float value
 * @param  *b: second float value
 * @retval None
 */
void swap(float *a, float *b);

/**
 * @brief  Swaps rows i and j of an n-column matrix stored row-major.
 * @param  *arr: Pointer to matrix
 * @param  i: index of row i
 * @param  j: index of row j
 * @param  n: number of elements to swap between raws
 * @retval None
 */
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

#ifdef __cplusplus
}
#endif

/*------------------------------------------*/
#endif

#include "Gauss_elimination.h"

/* Magnitude below which a pivot is treated as zero. */
const float GAUSS_EPS = 1e-6f;

void swap(float *a, float *b) {

    float temp = *a;
    *a = *b;
    *b = temp;

}

void ligne_swap(float* arr, int i, int j, int n){

    int k;

    for(k = 0; k < n; k++){
        swap(arr + i*n + k, arr + j*n + k);
    }
}

void partial_pivot(int pivot, float* a, float* b, float* tb, int n, int* swap_cnt){

    int j;
    int max_row = pivot;
    float max_val = fabsf(*(a + pivot*n + pivot));
    float val;

    /* Search downward only: rows above the pivot are already eliminated and
       swapping them here would undo that work. */
    for(j = pivot + 1; j < n; j++){

        val = fabsf(*(a + j*n + pivot));

        if(val > max_val){
            max_val = val;
            max_row = j;
        }
    }

    if(max_row != pivot){

        ligne_swap(a, pivot, max_row, n);
        ligne_swap(b, pivot, max_row, n);
        swap(tb + pivot, tb + max_row);
        (*swap_cnt)++;

    }
}

void matrix_elimination(int pivot, float* a, float* b, float* tb, int n){

    int j, k;
    float ratio;

    for(j = pivot + 1; j < n; j++){

        ratio = (*(a + j*n + pivot) / *(a + pivot*n + pivot));

        for(k = pivot; k < n; k++){
            *(a + j*n + k) -= (ratio*(*(a + pivot*n + k)));
        }

        for(k = 0; k < n; k++){
            *(b + j*n + k) -= (ratio*(*(b + pivot*n + k)));
        }

        *(tb + j) -= ratio*(*(tb + pivot));
    }
}

float determinant(float* a, float* b, float* tb, int n){

    float det = 1.0f;
    int i;
    int s = 0;

    /* Pivot and eliminate column by column. Pivoting must happen inside this
       loop, not in a separate pass beforehand: which row makes the best pivot
       depends on the eliminations already applied. */
    for(i = 0; i < n - 1; i++){

        partial_pivot(i, a, b, tb, n, &s);

        /* If the largest candidate in this column is still ~0, every entry
           below is ~0 too and the matrix is singular. */
        if(fabsf(*(a + i*n + i)) < GAUSS_EPS){
            return 0.0f;
        }

        matrix_elimination(i, a, b, tb, n);
    }

    /* The final pivot never gets a column eliminated below it, so check it
       separately. */
    if(fabsf(*(a + (n-1)*n + (n-1))) < GAUSS_EPS){
        return 0.0f;
    }

    for(i = 0; i < n; i++){
        det *= *(a + i*n + i);
    }

    /* Each row swap flips the sign. */
    if(s % 2 != 0){
        det = -det;
    }

    return det;
}

/*------------------------------------------*/
void inversion(float* a, float* b, float* tb, int n){

    int i, j, k;
    float coeff;

    for(i = n - 1; i >= 0; --i){

        coeff = *(a + i*n + i);

        for(j = n - 1; j > i - 1; --j){
            *(a + i*n + j) /= coeff;
        }

        for(j = n - 1; j >= 0; --j){
            *(b + i*n + j) /= coeff;
        }

        *(tb + i) /= coeff;

        for(k = i - 1; k >= 0; --k){
            if(*(a + k*n + i) != 0){

                coeff = *(a + k*n + i);

                for(j = n - 1; j > k; --j){
                    *(a + k*n + j) -= coeff * *(a + i*n + j);
                }

                for(j = n - 1; j >= 0; --j){
                    *(b + k*n + j) -= coeff * *(b + i*n + j);
                }

                *(tb + k) -= coeff * *(tb + i);
            }
        }
    }
}
/*------------------------------------------*/

#ifndef __ARM_MATH_H
#define __ARM_MATH_H

/*
 * Minimal subset of CMSIS-DSP arm_math.h needed by HARDWARE/kalman/kalman_filter.*
 *
 * This project currently does not vendor CMSIS-DSP. The Kalman implementation
 * only relies on basic float32 matrix operations, so we provide a lightweight
 * compatible API surface here.
 *
 * If you later add official CMSIS-DSP, you can remove this file and update the
 * include paths accordingly.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef float float32_t;

typedef enum
{
    ARM_MATH_SUCCESS = 0,
    ARM_MATH_ARGUMENT_ERROR = -1,
    ARM_MATH_LENGTH_ERROR = -2,
    ARM_MATH_SIZE_MISMATCH = -3,
    ARM_MATH_SINGULAR = -4
} arm_status;

typedef struct
{
    uint16_t numRows;
    uint16_t numCols;
    float32_t *pData; /* row-major: [r*numCols + c] */
} arm_matrix_instance_f32;

void arm_mat_init_f32(arm_matrix_instance_f32 *S, uint16_t nRows, uint16_t nCols, float32_t *pData);
arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst);
arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst);
arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA,
                            const arm_matrix_instance_f32 *pSrcB,
                            arm_matrix_instance_f32 *pDst);
arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc,
                             arm_matrix_instance_f32 *pDst);
arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc,
                               arm_matrix_instance_f32 *pDst);

#ifdef __cplusplus
}
#endif

#endif /* __ARM_MATH_H */

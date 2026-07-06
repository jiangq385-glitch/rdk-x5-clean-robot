#include "arm_math.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef ARM_MATH_EPS
#define ARM_MATH_EPS 1e-12f
#endif

static void arm_mat_set_identity_f32(arm_matrix_instance_f32 *pDst)
{
    uint16_t nRows = pDst->numRows;
    uint16_t nCols = pDst->numCols;
    memset(pDst->pData, 0, (size_t)nRows * (size_t)nCols * sizeof(float32_t));
    uint16_t n = (nRows < nCols) ? nRows : nCols;
    for (uint16_t i = 0; i < n; ++i)
    {
        pDst->pData[(size_t)i * nCols + i] = 1.0f;
    }
}

void arm_mat_init_f32(arm_matrix_instance_f32 *S, uint16_t nRows, uint16_t nCols, float32_t *pData)
{
    if (S == NULL)
        return;
    S->numRows = nRows;
    S->numCols = nCols;
    S->pData = pData;
}

arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst)
{
    if (!pSrcA || !pSrcB || !pDst)
        return ARM_MATH_ARGUMENT_ERROR;

    if (pSrcA->numRows != pSrcB->numRows || pSrcA->numCols != pSrcB->numCols)
        return ARM_MATH_SIZE_MISMATCH;

    if (pDst->numRows != pSrcA->numRows || pDst->numCols != pSrcA->numCols)
        return ARM_MATH_SIZE_MISMATCH;

    size_t count = (size_t)pSrcA->numRows * (size_t)pSrcA->numCols;
    for (size_t i = 0; i < count; ++i)
        pDst->pData[i] = pSrcA->pData[i] + pSrcB->pData[i];

    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst)
{
    if (!pSrcA || !pSrcB || !pDst)
        return ARM_MATH_ARGUMENT_ERROR;

    if (pSrcA->numRows != pSrcB->numRows || pSrcA->numCols != pSrcB->numCols)
        return ARM_MATH_SIZE_MISMATCH;

    if (pDst->numRows != pSrcA->numRows || pDst->numCols != pSrcA->numCols)
        return ARM_MATH_SIZE_MISMATCH;

    size_t count = (size_t)pSrcA->numRows * (size_t)pSrcA->numCols;
    for (size_t i = 0; i < count; ++i)
        pDst->pData[i] = pSrcA->pData[i] - pSrcB->pData[i];

    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA,
                            const arm_matrix_instance_f32 *pSrcB,
                            arm_matrix_instance_f32 *pDst)
{
    if (!pSrcA || !pSrcB || !pDst)
        return ARM_MATH_ARGUMENT_ERROR;

    if (pSrcA->numCols != pSrcB->numRows)
        return ARM_MATH_SIZE_MISMATCH;

    if (pDst->numRows != pSrcA->numRows || pDst->numCols != pSrcB->numCols)
        return ARM_MATH_SIZE_MISMATCH;

    uint16_t m = pSrcA->numRows;
    uint16_t n = pSrcA->numCols;
    uint16_t p = pSrcB->numCols;

    for (uint16_t i = 0; i < m; ++i)
    {
        for (uint16_t j = 0; j < p; ++j)
        {
            float32_t sum = 0.0f;
            const float32_t *aRow = &pSrcA->pData[(size_t)i * n];
            for (uint16_t k = 0; k < n; ++k)
                sum += aRow[k] * pSrcB->pData[(size_t)k * p + j];
            pDst->pData[(size_t)i * p + j] = sum;
        }
    }

    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc,
                             arm_matrix_instance_f32 *pDst)
{
    if (!pSrc || !pDst)
        return ARM_MATH_ARGUMENT_ERROR;

    if (pDst->numRows != pSrc->numCols || pDst->numCols != pSrc->numRows)
        return ARM_MATH_SIZE_MISMATCH;

    uint16_t r = pSrc->numRows;
    uint16_t c = pSrc->numCols;

    for (uint16_t i = 0; i < r; ++i)
        for (uint16_t j = 0; j < c; ++j)
            pDst->pData[(size_t)j * pDst->numCols + i] = pSrc->pData[(size_t)i * c + j];

    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc,
                               arm_matrix_instance_f32 *pDst)
{
    if (!pSrc || !pDst)
        return ARM_MATH_ARGUMENT_ERROR;

    if (pSrc->numRows != pSrc->numCols)
        return ARM_MATH_SIZE_MISMATCH;

    if (pDst->numRows != pSrc->numRows || pDst->numCols != pSrc->numCols)
        return ARM_MATH_SIZE_MISMATCH;

    uint16_t n = pSrc->numRows;

    float32_t *a = (float32_t *)malloc((size_t)n * (size_t)n * sizeof(float32_t));
    if (!a)
        return ARM_MATH_ARGUMENT_ERROR;

    memcpy(a, pSrc->pData, (size_t)n * (size_t)n * sizeof(float32_t));
    arm_mat_set_identity_f32(pDst);

    for (uint16_t col = 0; col < n; ++col)
    {
        uint16_t pivotRow = col;
        float32_t pivotAbs = fabsf(a[(size_t)col * n + col]);
        for (uint16_t r = col + 1; r < n; ++r)
        {
            float32_t v = fabsf(a[(size_t)r * n + col]);
            if (v > pivotAbs)
            {
                pivotAbs = v;
                pivotRow = r;
            }
        }

        if (pivotAbs < ARM_MATH_EPS)
        {
            free(a);
            return ARM_MATH_SINGULAR;
        }

        if (pivotRow != col)
        {
            for (uint16_t j = 0; j < n; ++j)
            {
                float32_t tmp = a[(size_t)col * n + j];
                a[(size_t)col * n + j] = a[(size_t)pivotRow * n + j];
                a[(size_t)pivotRow * n + j] = tmp;

                tmp = pDst->pData[(size_t)col * n + j];
                pDst->pData[(size_t)col * n + j] = pDst->pData[(size_t)pivotRow * n + j];
                pDst->pData[(size_t)pivotRow * n + j] = tmp;
            }
        }

        float32_t pivot = a[(size_t)col * n + col];
        float32_t invPivot = 1.0f / pivot;
        for (uint16_t j = 0; j < n; ++j)
        {
            a[(size_t)col * n + j] *= invPivot;
            pDst->pData[(size_t)col * n + j] *= invPivot;
        }

        for (uint16_t r = 0; r < n; ++r)
        {
            if (r == col)
                continue;

            float32_t factor = a[(size_t)r * n + col];
            if (fabsf(factor) < ARM_MATH_EPS)
                continue;

            for (uint16_t j = 0; j < n; ++j)
            {
                a[(size_t)r * n + j] -= factor * a[(size_t)col * n + j];
                pDst->pData[(size_t)r * n + j] -= factor * pDst->pData[(size_t)col * n + j];
            }
        }
    }

    free(a);
    return ARM_MATH_SUCCESS;
}

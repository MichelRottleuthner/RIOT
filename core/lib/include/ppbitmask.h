/*
 * Copyright (C) 2022 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     core_util
 * @{
 *
 * @file
 * @brief       Helper functions for static preprocessor bitmasking
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */

#ifndef PPBITMASK_H
#define PPBITMASK_H

#ifdef __cplusplus
extern "C" {
#endif

//! @cond Doxygen_Suppress
#define BITMASK_BIT_0_SET(VAL)  ((((VAL & (1 <<  0))) != 0) ? 1 : 0)
#define BITMASK_BIT_1_SET(VAL)  ((((VAL & (1 <<  1))) != 0) ? 1 : 0)
#define BITMASK_BIT_2_SET(VAL)  ((((VAL & (1 <<  2))) != 0) ? 1 : 0)
#define BITMASK_BIT_3_SET(VAL)  ((((VAL & (1 <<  3))) != 0) ? 1 : 0)
#define BITMASK_BIT_4_SET(VAL)  ((((VAL & (1 <<  4))) != 0) ? 1 : 0)
#define BITMASK_BIT_5_SET(VAL)  ((((VAL & (1 <<  5))) != 0) ? 1 : 0)
#define BITMASK_BIT_6_SET(VAL)  ((((VAL & (1 <<  6))) != 0) ? 1 : 0)
#define BITMASK_BIT_7_SET(VAL)  ((((VAL & (1 <<  7))) != 0) ? 1 : 0)
#define BITMASK_BIT_8_SET(VAL)  ((((VAL & (1 <<  8))) != 0) ? 1 : 0)
#define BITMASK_BIT_9_SET(VAL)  ((((VAL & (1 <<  9))) != 0) ? 1 : 0)
#define BITMASK_BIT_10_SET(VAL) ((((VAL & (1 << 10))) != 0) ? 1 : 0)
#define BITMASK_BIT_11_SET(VAL) ((((VAL & (1 << 11))) != 0) ? 1 : 0)
#define BITMASK_BIT_12_SET(VAL) ((((VAL & (1 << 12))) != 0) ? 1 : 0)
#define BITMASK_BIT_13_SET(VAL) ((((VAL & (1 << 13))) != 0) ? 1 : 0)
#define BITMASK_BIT_14_SET(VAL) ((((VAL & (1 << 14))) != 0) ? 1 : 0)
#define BITMASK_BIT_15_SET(VAL) ((((VAL & (1 << 15))) != 0) ? 1 : 0)
#define BITMASK_BIT_16_SET(VAL) ((((VAL & (1 << 16))) != 0) ? 1 : 0)
#define BITMASK_BIT_17_SET(VAL) ((((VAL & (1 << 17))) != 0) ? 1 : 0)
#define BITMASK_BIT_18_SET(VAL) ((((VAL & (1 << 18))) != 0) ? 1 : 0)
#define BITMASK_BIT_19_SET(VAL) ((((VAL & (1 << 19))) != 0) ? 1 : 0)
#define BITMASK_BIT_20_SET(VAL) ((((VAL & (1 << 20))) != 0) ? 1 : 0)
#define BITMASK_BIT_21_SET(VAL) ((((VAL & (1 << 21))) != 0) ? 1 : 0)
#define BITMASK_BIT_22_SET(VAL) ((((VAL & (1 << 22))) != 0) ? 1 : 0)
#define BITMASK_BIT_23_SET(VAL) ((((VAL & (1 << 23))) != 0) ? 1 : 0)
#define BITMASK_BIT_24_SET(VAL) ((((VAL & (1 << 24))) != 0) ? 1 : 0)
#define BITMASK_BIT_25_SET(VAL) ((((VAL & (1 << 25))) != 0) ? 1 : 0)
#define BITMASK_BIT_26_SET(VAL) ((((VAL & (1 << 26))) != 0) ? 1 : 0)
#define BITMASK_BIT_27_SET(VAL) ((((VAL & (1 << 27))) != 0) ? 1 : 0)
#define BITMASK_BIT_28_SET(VAL) ((((VAL & (1 << 28))) != 0) ? 1 : 0)
#define BITMASK_BIT_29_SET(VAL) ((((VAL & (1 << 29))) != 0) ? 1 : 0)
#define BITMASK_BIT_30_SET(VAL) ((((VAL & (1 << 30))) != 0) ? 1 : 0)
#define BITMASK_BIT_31_SET(VAL) ((((VAL & (1 << 31))) != 0) ? 1 : 0)
//! @endcond

/**
 * @brief Macro to get the MSB index of a value from the preprocessor.
 */
#define STATIC_BITMASK_MSB(X) (BITMASK_BIT_31_SET(X) ? 31 : (\
                               BITMASK_BIT_30_SET(X) ? 30 : (\
                               BITMASK_BIT_29_SET(X) ? 29 : (\
                               BITMASK_BIT_28_SET(X) ? 28 : (\
                               BITMASK_BIT_27_SET(X) ? 27 : (\
                               BITMASK_BIT_26_SET(X) ? 26 : (\
                               BITMASK_BIT_25_SET(X) ? 25 : (\
                               BITMASK_BIT_24_SET(X) ? 24 : (\
                               BITMASK_BIT_23_SET(X) ? 23 : (\
                               BITMASK_BIT_22_SET(X) ? 22 : (\
                               BITMASK_BIT_21_SET(X) ? 21 : (\
                               BITMASK_BIT_20_SET(X) ? 20 : (\
                               BITMASK_BIT_19_SET(X) ? 19 : (\
                               BITMASK_BIT_18_SET(X) ? 18 : (\
                               BITMASK_BIT_17_SET(X) ? 17 : (\
                               BITMASK_BIT_16_SET(X) ? 16 : (\
                               BITMASK_BIT_15_SET(X) ? 15 : (\
                               BITMASK_BIT_14_SET(X) ? 14 : (\
                               BITMASK_BIT_13_SET(X) ? 13 : (\
                               BITMASK_BIT_12_SET(X) ? 12 : (\
                               BITMASK_BIT_11_SET(X) ? 11 : (\
                               BITMASK_BIT_10_SET(X) ? 10 : (\
                               BITMASK_BIT_9_SET(X) ?   9 : (\
                               BITMASK_BIT_8_SET(X) ?   8 : (\
                               BITMASK_BIT_7_SET(X) ?   7 : (\
                               BITMASK_BIT_6_SET(X) ?   6 : (\
                               BITMASK_BIT_5_SET(X) ?   5 : (\
                               BITMASK_BIT_4_SET(X) ?   4 : (\
                               BITMASK_BIT_3_SET(X) ?   3 : (\
                               BITMASK_BIT_2_SET(X) ?   2 : (\
                               BITMASK_BIT_1_SET(X) ?   1 : (\
                               BITMASK_BIT_0_SET(X) ?   0 : 0\
                               ))))))))))))))))))))))))))))))))

/**
 * @brief Macro to get the LSB index of a value from the preprocessor.
 */
#define STATIC_BITMASK_LSB(X) (BITMASK_BIT_0_SET(X) ?   0 : (\
                               BITMASK_BIT_1_SET(X) ?   1 : (\
                               BITMASK_BIT_2_SET(X) ?   2 : (\
                               BITMASK_BIT_3_SET(X) ?   3 : (\
                               BITMASK_BIT_4_SET(X) ?   4 : (\
                               BITMASK_BIT_5_SET(X) ?   5 : (\
                               BITMASK_BIT_6_SET(X) ?   6 : (\
                               BITMASK_BIT_7_SET(X) ?   7 : (\
                               BITMASK_BIT_8_SET(X) ?   8 : (\
                               BITMASK_BIT_9_SET(X) ?   9 : (\
                               BITMASK_BIT_10_SET(X) ? 10 : (\
                               BITMASK_BIT_11_SET(X) ? 11 : (\
                               BITMASK_BIT_12_SET(X) ? 12 : (\
                               BITMASK_BIT_13_SET(X) ? 13 : (\
                               BITMASK_BIT_14_SET(X) ? 14 : (\
                               BITMASK_BIT_15_SET(X) ? 15 : (\
                               BITMASK_BIT_16_SET(X) ? 16 : (\
                               BITMASK_BIT_17_SET(X) ? 17 : (\
                               BITMASK_BIT_18_SET(X) ? 18 : (\
                               BITMASK_BIT_19_SET(X) ? 19 : (\
                               BITMASK_BIT_20_SET(X) ? 20 : (\
                               BITMASK_BIT_21_SET(X) ? 21 : (\
                               BITMASK_BIT_22_SET(X) ? 22 : (\
                               BITMASK_BIT_23_SET(X) ? 23 : (\
                               BITMASK_BIT_24_SET(X) ? 24 : (\
                               BITMASK_BIT_25_SET(X) ? 25 : (\
                               BITMASK_BIT_26_SET(X) ? 26 : (\
                               BITMASK_BIT_27_SET(X) ? 27 : (\
                               BITMASK_BIT_28_SET(X) ? 28 : (\
                               BITMASK_BIT_29_SET(X) ? 29 : (\
                               BITMASK_BIT_30_SET(X) ? 30 : (\
                               BITMASK_BIT_31_SET(X) ? 31 : 0\
                               ))))))))))))))))))))))))))))))))

#ifdef __cplusplus
}
#endif

#endif /* PPBITMASK_H */
/** @} */

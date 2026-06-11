/*
*   by dl8mcg Jan. 2025 to June 2026
*/

#pragma once

#ifndef DEMODINT_H
#define DEMODINT_H

#include <stdint.h>

typedef enum
{
    FSK_RTTY_45_BAUD_170Hz,
    FSK_RTTY_50_BAUD_85Hz,
    FSK_RTTY_50_BAUD_450Hz,
    FSK_EFR_200_BAUD_340Hz,
    FSK_ASCII_300_BAUD_850Hz,
    FSK_AX25_1200_BAUD_1000Hz,
    FSK_AX25_9600_BAUD
} FskMode;

typedef struct
{
    float amp1;
    float amp2;
} FskAmplitudes;


void init_fsk_demod_int(FskMode mode);

extern FskAmplitudes(*smDemod_int)(int32_t);


#endif // DEMODINT_H

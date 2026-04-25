/*
*   by dl8mcg Jan. 2025 to April 2026
*/

#pragma once

#ifndef DEMOD_H
#define DEMOD_H

typedef enum 
{
    FSK_RTTY_45_BAUD_170Hz,
    FSK_RTTY_50_BAUD_85Hz,
    FSK_RTTY_50_BAUD_450Hz,
    FSK_EFR_200_BAUD_340Hz,
    FSK_ASCII_300_BAUD_850Hz,
    FSK_AX25_1200_BAUD_1000Hz
} FskMode;

typedef struct
{
    float amp1;
    float amp2;
} FskAmplitudes;

extern volatile int demod_bit;

void init_fsk_demod(FskMode mode);
//void process_fsk_demodulation(float sample);
FskAmplitudes process_fsk_demod_center_nco(float sample);

#endif // DEMOD_H
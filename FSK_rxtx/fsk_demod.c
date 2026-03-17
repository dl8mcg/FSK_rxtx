/*
*   by dl8mcg Jan. 2025 to März 2026       FSK-demodulator
*/

#include <stdint.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include "config.h"
#include "fsk_demod.h"
#include "fsk_decode_ascii.h"
#include "fsk_decode_rtty.h"
#include "fsk_decode_ax25.h"
#include "fsk_decode_efr.h"

static void (*smMode)(uint8_t) = process_rtty;    

// ------------------------------------------------------------
// NCO
// ------------------------------------------------------------

static float nco_phase = 0.0f;
static float nco_step = 0.0f;

// ------------------------------------------------------------
// I/Q LPF
// ------------------------------------------------------------

static float lpf_alpha = 0.05f;

static float i1 = 0, i2 = 0, i3 = 0, i4 = 0;
static float q1 = 0, q2 = 0, q3 = 0, q4 = 0;

// ------------------------------------------------------------
// discriminator
// ------------------------------------------------------------

static float prev_i = 0;
static float prev_q = 0;

static float freq;
static float freq_filt = 0;
static float freq_alpha = 0.2f;

static float freq_cnt = 0;


static float prev_freq_filt = 0;

// ------------------------------------------------------------
// bit clock
// ------------------------------------------------------------

static float baud_rate;

static float samples_per_bit;
static float samples_per_half_bit;

static float bit_timer = 0;

// ------------------------------------------------------------

static int bit_value = 0;
static int previous_bit_value = 0;

volatile bool inverse_fsk = false; 


// ------------------------------------------------------------
// main processing
// ------------------------------------------------------------

void process_fsk_demodulation(float sample)
{
    // -----------------------------
    // NCO
    // -----------------------------

    nco_phase += nco_step;

    if (nco_phase > 2 * M_PI)
        nco_phase -= 2 * M_PI;

    float lo_i = cosf(nco_phase);
    float lo_q = sinf(nco_phase);

    // -----------------------------
    // mixer
    // -----------------------------

    float mix_i = sample * lo_i;
    float mix_q = sample * lo_q;

    // -----------------------------
    // I/Q lowpass (4 pole IIR)
    // -----------------------------

    i1 += lpf_alpha * (mix_i - i1);
    q1 += lpf_alpha * (mix_q - q1);

    i2 += lpf_alpha * (i1 - i2);
    q2 += lpf_alpha * (q1 - q2);

    i3 += lpf_alpha * (i2 - i3);
    q3 += lpf_alpha * (q2 - q3);

    i4 += lpf_alpha * (i3 - i4);
    q4 += lpf_alpha * (q3 - q4);

    // -----------------------------
    // frequency discriminator
    // -----------------------------

    freq = i4 * prev_q - q4 * prev_i;

    prev_i = i4;
    prev_q = q4;

    // ------------
    // smoothing
    // ------------

    freq_filt += freq_alpha * (freq - freq_filt);

    // ------------------------------------------------------------------------------
	// sum up low (negative) and high (positive) frequencies to stable bit decision
    // ------------------------------------------------------------------------------

    if (freq_filt < 0)
    {
        freq_cnt--;
    }
    else
    {
        freq_cnt++;
    }
        
    // -------------
    // bit decision
    // -------------

    bit_value = (freq_filt > 0);

    //-----------------------------------------------
    // edge detector , mögliche Bitwechsel reduziert
    //-----------------------------------------------

    if (bit_value != previous_bit_value)
    {
        if (bit_timer < samples_per_bit * 0.75f)
        {
            bit_timer = samples_per_half_bit;
            freq_cnt = 0;
            previous_bit_value = bit_value;   
        }
    }

    // -----------------------------
    // bit sampler
    // -----------------------------

    if (--bit_timer <= 0)
    {
        bit_timer = samples_per_bit;

        if (inverse_fsk)
        {

            if (freq_cnt < 0)
                smMode(1);
            else
                smMode(0);
        }
        else
        {
            if (freq_cnt > 0)
                smMode(1);
            else
                smMode(0);
        }
        freq_cnt = 0;
    }
}

// -----------------------------
// Init mode specific parameters
// -----------------------------

void init_fsk_demod(FskMode mode)
{
    float flow;
    float fhigh;

    switch (mode)
    {
        case FSK_RTTY_45_BAUD_170Hz:
            baud_rate = 45.454545f;
            lpf_alpha = 0.04f;
            flow = 2125.0f;
            fhigh = 2295.0f;
            inverse_fsk = false; // Standard FSK (mark = high frequency, space = low frequency)
            smMode = process_rtty_uos;
            wprintf(L"\n\nModus FSK_RTTY_45_BAUD  %g Hz / %g Hz  set rx to usb\n\n\n\n", flow, fhigh);
            break;

        case FSK_RTTY_50_BAUD_85Hz:
            baud_rate = 50.0f;
            lpf_alpha = 0.021f;
            flow = 1957.5f;
            fhigh = 2042.5f;
            inverse_fsk = true; // Inverse FSK (mark = low frequency, space = high frequency)
            smMode = process_rtty;
            wprintf(L"\n\nModus FSK_RTTY_50_BAUD  %g Hz / %g Hz   f = 147,3 kHz   set rx to f - 2kHz usb\n\n", flow, fhigh);
            break;

        case FSK_RTTY_50_BAUD_450Hz:
            baud_rate = 50.0f;
            lpf_alpha = 0.06f;
            flow = 1775.0f;
            fhigh = 2225.0f;
            inverse_fsk = true; // Inverse FSK (mark = low frequency, space = high frequency)
            smMode = process_rtty;
            wprintf(L"\n\nModus FSK_RTTY_50_BAUD  %g Hz / %g Hz   f = 4583 kHz, 7646 kHz, 10100.8 kHz, 11039 kHz, 14467.3 kHz  set rx to f - 2kHz usb\n\n", flow, fhigh);
            break;

        case FSK_EFR_200_BAUD_340Hz:
            baud_rate = 200.0f;
            lpf_alpha = 0.04f;
            flow = 1830.0f;
            fhigh = 2170.0f;
            inverse_fsk = true; // Inverse FSK (mark = low frequency, space = high frequency)
            smMode = process_efr;
            wprintf(L"\n\nModus FSK_EFR_200_BAUD  %g Hz / %g Hz   129,1 kHz  139 kHz  135,6 kHz\n\n", flow, fhigh);
            break;

        case FSK_ASCII_300_BAUD_850Hz:
            baud_rate = 300.0f;
            lpf_alpha = 0.064f;
            flow = 1275.0f;
            fhigh = 2125.0f;
            inverse_fsk = false; // Standard FSK (mark = high frequency, space = low frequency)
            smMode = process_ascii;
            wprintf(L"\n\nModus FSK_ASCII_300_BAUD  %g Hz / %g Hz\n\n", flow, fhigh);
            break;

        case FSK_AX25_1200_BAUD_1000Hz:
            baud_rate = 1200.0f;
            lpf_alpha = 0.1075f;
            flow = 1200.0f;
            fhigh = 2200.0f;
            inverse_fsk = false; // Standard FSK (mark = high frequency, space = low frequency)
            smMode = process_ax25;
            wprintf(L"\n\nModus FSK_AX25_1200_BAUD  %g Hz / %g Hz\n\n", flow, fhigh);
            break;

        default:
            printf("Ungültiger FSK-Modus.\n");
            return;
    }

    nco_step = 2 * (float)M_PI * (flow + (fhigh - flow) / 2) / SAMPLING_RATE;
    samples_per_bit = SAMPLING_RATE / baud_rate;
    samples_per_half_bit = samples_per_bit / 2;

    freq_alpha = lpf_alpha;

}




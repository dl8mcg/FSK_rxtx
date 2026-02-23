/*
*   by dl8mcg Jan. 2025 .. Feb. 2026
*   DPLL Bitclock Version
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

static void (*smMode)(uint8_t) = process_ascii;      // Initialzustand

// -------------------------------------------------
// Tone NCO
// -------------------------------------------------
volatile double nco_phase_low = 0.0f;
volatile double nco_phase_high = 0.0f;
double nco_step_low = 0.0f;
double nco_step_high = 0.0f;

double baud_rate = 300.0f;

// -------------------------------------------------
// Bit Clock DPLL (NEU)
// -------------------------------------------------
volatile double bit_phase = 0.0f;     // 0 … 1
volatile double bit_freq = 0.0f;
double bit_freq_nom = 0.0f;

double pll_alpha = 0.02f;   // proportional
double pll_beta = 0.0005f;  // integral

volatile double pll_integrator = 0.0f;

// -------------------------------------------------
// Moving Average Filter (Matched Filter)
// -------------------------------------------------
#define MA_MAX_LEN 2048 // Reicht locker für 45 Baud @ 44.1kHz (ca. 970 Samples)

static double i_low_buf[MA_MAX_LEN] = { 0 };
static double q_low_buf[MA_MAX_LEN] = { 0 };
static double i_high_buf[MA_MAX_LEN] = { 0 };
static double q_high_buf[MA_MAX_LEN] = { 0 };

static double i_low_sum = 0.0f;
static double q_low_sum = 0.0f;
static double i_high_sum = 0.0f;
static double q_high_sum = 0.0f;

static int ma_idx = 0;
static int ma_len = 1; // Wird in der Init-Routine berechnet

// -------------------------------------------------
// Synchronisation
// -------------------------------------------------
volatile double amplitude_low = 0;
volatile double amplitude_high = 0;

volatile int bit_value = -1;
volatile int previous_bit_value = -1;
volatile int demod_bit = 0;

// =================================================
// Haupt-Demodulationsroutine
// =================================================
void process_fsk_demodulation(float sample)
{
    // 1. NCO & Mischen 
    nco_phase_low += nco_step_low;
    if (nco_phase_low > 2 * (float)M_PI) nco_phase_low -= 2 * (float)M_PI;
    nco_phase_high += nco_step_high;
    if (nco_phase_high > 2 * (float)M_PI) nco_phase_high -= 2 * (float)M_PI;

    double i_sample_low = sample * cosf(nco_phase_low);
    double q_sample_low = sample * sinf(nco_phase_low);
    double i_sample_high = sample * cosf(nco_phase_high);
    double q_sample_high = sample * sinf(nco_phase_high);

    // 2. Moving Average 
    i_low_sum -= i_low_buf[ma_idx];
    q_low_sum -= q_low_buf[ma_idx];
    i_high_sum -= i_high_buf[ma_idx];
    q_high_sum -= q_high_buf[ma_idx];

    i_low_buf[ma_idx] = i_sample_low;
    q_low_buf[ma_idx] = q_sample_low;
    i_high_buf[ma_idx] = i_sample_high;
    q_high_buf[ma_idx] = q_sample_high;

    i_low_sum += i_sample_low;
    q_low_sum += q_sample_low;
    i_high_sum += i_sample_high;
    q_high_sum += q_sample_high;

    if (++ma_idx >= ma_len) ma_idx = 0;

    double i_low_filt = i_low_sum / ma_len;
    double q_low_filt = q_low_sum / ma_len;
    double i_high_filt = i_high_sum / ma_len;
    double q_high_filt = q_high_sum / ma_len;

    amplitude_low = sqrtf(i_low_filt * i_low_filt + q_low_filt * q_low_filt);
    amplitude_high = sqrtf(i_high_filt * i_high_filt + q_high_filt * q_high_filt);

    double baseband = amplitude_high - amplitude_low;
    double hysteresis = 0.05f * (amplitude_high + amplitude_low);

    if (baseband > hysteresis) bit_value = 1;
    else if (baseband < -hysteresis) bit_value = 0;

    // --- 1. Bit Clock NCO ---
    bit_phase += bit_freq;

    if (bit_phase >= 1.0f)
    {
        bit_phase -= 1.0f;
        demod_bit = bit_value;
        smMode(demod_bit);
    }

    // --- 2. DPLL Logik  ---
    if (previous_bit_value == -1)
    {
        previous_bit_value = bit_value;
        return;
    }

    if (bit_value != previous_bit_value)
    {
        // Regeln auf 0.5 (Bit-Mitte)!
        // Da das MA-Filter 0.5 Bits Verzögerung hat, ist die Flanke 
        // im Basisband exakt der ideale Zeitpunkt für die Phasen-Mitte.
        double phase_error = bit_phase - 0.5f;
        
        // Wrap-around ist hier wichtig
        if (phase_error > 0.5f) phase_error -= 1.0f;
        if (phase_error < -0.5f) phase_error += 1.0f;

        double error_limit = 0.10f;
        double clamped_error = (phase_error > error_limit) ? error_limit : (phase_error < -error_limit) ? -error_limit : phase_error;

        // Korrektur
        bit_phase -= pll_alpha * clamped_error;
        pll_integrator -= pll_beta * clamped_error;
        
        // Sicherheits-Limit (sehr wichtig gegen Davonlaufen!)
        double max_dev = bit_freq_nom * 0.01f;
        if (pll_integrator > max_dev) pll_integrator = max_dev;
        if (pll_integrator < -max_dev) pll_integrator = -max_dev;

        bit_freq = bit_freq_nom + pll_integrator;

        previous_bit_value = bit_value;
    }
}

// =================================================
// Initialisierung
// =================================================
void init_fsk_demod(FskMode mode)
{
    double flow;
    double fhigh;

    switch (mode)
    {
    case FSK_RTTY_45_BAUD_170Hz:
        baud_rate = 45.454545f;
        pll_alpha = 0.015f;
        pll_beta = 0.0003f;
        flow = 2125.0f;
        fhigh = 2295.0f;
        smMode = process_rtty;
        wprintf(L"\n\nModus FSK_RTTY_45_BAUD  %g Hz / %g Hz\n\n", flow, fhigh);
        break;

    case FSK_RTTY_50_BAUD_85Hz:
        baud_rate = 50.0f;
        pll_alpha = 0.015f;
        pll_beta = 0.0003f;
        flow = 1957.5f;
        fhigh = 2042.5f;
        smMode = process_rtty;
        wprintf(L"\n\nModus FSK_RTTY_50_BAUD  %g Hz / %g Hz   147,3 kHz\n\n", flow, fhigh);
        break;

    case FSK_RTTY_50_BAUD_450Hz:
        baud_rate = 50.0f;
        pll_alpha = 0.015f;
        pll_beta = 0.0003f;
        flow = 1775.0f;
        fhigh = 2225.0f;
        smMode = process_rtty;
        wprintf(L"\n\nModus FSK_RTTY_50_BAUD  %g Hz / %g Hz   4583 kHz  7646 kHz  10100,8 kHz  11039 kHz  14467,3 kHz\n\n", flow, fhigh);
        break;

    case FSK_EFR_200_BAUD_340Hz:
        baud_rate = 200.0f;
        pll_alpha = 0.02f;
        pll_beta = 0.0005f;
        flow = 1830.0f;
        fhigh = 2170.0f;
        smMode = process_efr;
        wprintf(L"\n\nModus FSK_EFR_200_BAUD  %g Hz / %g Hz   129,1 kHz  139 kHz  135,6 kHz\n\n", flow, fhigh);
        break;

    case FSK_ASCII_300_BAUD_850Hz:
        baud_rate = 300.0f;
        pll_alpha = 0.02f;
        pll_beta = 0.0005f;
        flow = 1275.0f;
        fhigh = 2125.0f;
        smMode = process_ascii;
        wprintf(L"\n\nModus FSK_ASCII_300_BAUD  %g Hz / %g Hz\n\n", flow, fhigh);
        break;

    case FSK_AX25_1200_BAUD_1000Hz:
        baud_rate = 1200.0f;
        pll_alpha = 0.03f;
        pll_beta = 0.0008f;
        flow = 1200.0f;
        fhigh = 2200.0f;
        smMode = process_ax25;
        wprintf(L"\n\nModus FSK_AX25_1200_BAUD  %g Hz / %g Hz\n\n", flow, fhigh);
        break;

    default:
        printf("Ungültiger FSK-Modus.\n");
        return;
    }

    nco_step_low = 2 * (double)M_PI * flow / SAMPLING_RATE;
    nco_step_high = 2 * (double)M_PI * fhigh / SAMPLING_RATE;

    bit_freq_nom = baud_rate / SAMPLING_RATE;
    bit_freq = bit_freq_nom;
    bit_phase = 0.0f;
    pll_integrator = 0.0f;
    previous_bit_value = -1;

    // ---------------------------------------------
    // Moving Average Filter initialisieren
    // ---------------------------------------------
    // Die Länge entspricht genau der Dauer eines Bits in Samples
    ma_len = (int)(SAMPLING_RATE / baud_rate);

    // Sicherheit-Check
    if (ma_len > MA_MAX_LEN) ma_len = MA_MAX_LEN;
    if (ma_len < 1) ma_len = 1;

    ma_idx = 0;
    i_low_sum = 0.0f; q_low_sum = 0.0f;
    i_high_sum = 0.0f; q_high_sum = 0.0f;

    for (int i = 0; i < MA_MAX_LEN; i++) 
    {
        i_low_buf[i] = 0.0f;
        q_low_buf[i] = 0.0f;
        i_high_buf[i] = 0.0f;
        q_high_buf[i] = 0.0f;
    }
}
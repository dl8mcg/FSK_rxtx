/*
*   by dl8mcg Jan. 2025 to May 2026       FSK-demodulator
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

static void (*smDecoding)(uint8_t) = process_rtty;

// ------------------------------------------------------------
// FSK settings
// ------------------------------------------------------------

static float baud_rate;
volatile bool inverse_fsk = false;
static float samples_per_bit = 0.0f;
static float samples_per_half_bit = 0.0f;

// ------------------------------------------------------------
// NCO - center
// ------------------------------------------------------------

static float nco_phase = 0.0f;
static float nco_step = 0.0f;

// ------------------------------------------------------------
// I/Q LPF
// ------------------------------------------------------------

static float lpf_alpha = 0;
static float i1 = 0, i2 = 0, i3 = 0, i4 = 0;
static float q1 = 0, q2 = 0, q3 = 0, q4 = 0;

// ------------------------------------------------------------
// frequency discriminator
// ------------------------------------------------------------

static float prev_i = 0;
static float prev_q = 0;
static float disc_filt = 0;
static float disc_alpha = 0;
static float disc_acc = 0.0f;
static float disc_acc_dbg = 0.0f;

// ------------------------------------------------------------
// bit clock
// ------------------------------------------------------------

static float bit_timer = 0.0f;
static int bit_avg = 0;
static bit_avg_threshold = 4;
static bool toggle_strobe_dbg = false;




static int disc_acc_last = 0;


static void BitTransSimple(float disc);
static void BitTransAvg_0(float disc);
static void BitTransAvg_1(float disc);
static void BitTransBuf(float disc);
static void (*smBitClock)(float) = BitTransAvg_0;

static void init_trans_ringbuffer();


FskAmplitudes process_fsk_demod_center_nco(float sample)
{
    // NCO
    nco_phase += nco_step;
    if (nco_phase > 2 * M_PI)
        nco_phase -= 2 * M_PI;

    // Mixer
    float mix_i = sample * cosf(nco_phase);
    float mix_q = sample * sinf(nco_phase);

    // I/Q Lowpass (4-pol IIR)
    i1 += lpf_alpha * (mix_i - i1);  q1 += lpf_alpha * (mix_q - q1);
    i2 += lpf_alpha * (i1 - i2);     q2 += lpf_alpha * (q1 - q2);
    i3 += lpf_alpha * (i2 - i3);     q3 += lpf_alpha * (q2 - q3);
    i4 += lpf_alpha * (i3 - i4);     q4 += lpf_alpha * (q3 - q4);

    // frequency discriminator
    float disc = i4 * prev_q - q4 * prev_i;
    prev_i = i4;
    prev_q = q4;
    disc_filt += disc_alpha * (disc - disc_filt);

	// data bit accumulation
    disc_acc += disc_filt;
    disc_acc_dbg += disc_filt;

	// bit clock
    smBitClock(disc_filt);

    // bit detektion
    bit_timer += 1.0f;
    if (bit_timer >= samples_per_bit)
    {
        bit_timer = 0;
        smDecoding(inverse_fsk ? (disc_acc < disc_acc_last ? 1 : 0) : (disc_acc > disc_acc_last ? 1 : 0));
		disc_acc_last = disc_acc;
        disc_acc = 0.0f;
        disc_acc_dbg = 0.0f;
        toggle_strobe_dbg = !toggle_strobe_dbg;
    }

    FskAmplitudes amps;
    amps.amp1 = toggle_strobe_dbg * 0.1f;   // Zum Debuggen: Zeigt die Detektion der Bitkante an
	amps.amp2 = disc_acc_dbg * 100;         // Zum Debuggen: Zeigt den akkumulierten Diskriminatorwert an
    return amps;
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
            smBitClock = BitTransBuf; 
            inverse_fsk = false; // Standard FSK (mark = high frequency, space = low frequency)
            smDecoding = process_rtty_uos;
            wprintf(L"\n\nModus FSK_RTTY_45_BAUD  %g Hz / %g Hz  set rx to usb   or  f = 438.450 MHz, 438.550 MHz  FM\n\n\n\n", flow, fhigh);
            break;

        case FSK_RTTY_50_BAUD_85Hz:
            baud_rate = 50.0f;
            lpf_alpha = 0.02f;
            flow = 1957.5f;
            fhigh = 2042.5f;
            smBitClock = BitTransBuf;
            inverse_fsk = true; // Inverse FSK (mark = low frequency, space = high frequency)
            smDecoding = process_rtty;
            wprintf(L"\n\nModus FSK_RTTY_50_BAUD  %g Hz / %g Hz   f = 147.3 kHz   set rx to f - 2kHz USB\n\n", flow, fhigh);
            break;

        case FSK_RTTY_50_BAUD_450Hz:
            baud_rate = 50.0f;
            lpf_alpha = 0.06f;
            flow = 1775.0f;
            fhigh = 2225.0f;
            smBitClock = BitTransBuf;
            inverse_fsk = true; // Inverse FSK (mark = low frequency, space = high frequency)
            smDecoding = process_rtty;
            wprintf(L"\n\nModus FSK_RTTY_50_BAUD  %g Hz / %g Hz   f = 4583 kHz, 7646 kHz, 10100.8 kHz, 11039 kHz, 14467.3 kHz  set rx to f - 2kHz USB\n\n", flow, fhigh);
            break;

        case FSK_EFR_200_BAUD_340Hz:
            baud_rate = 200.0f;
            lpf_alpha = 0.06f;
            flow = 1830.0f;
            fhigh = 2170.0f;
            smBitClock = BitTransBuf;
            inverse_fsk = true; // Inverse FSK (mark = low frequency, space = high frequency)
            smDecoding = process_efr;
            wprintf(L"\n\nModus FSK_EFR_200_BAUD  %g Hz / %g Hz   f = 129.1 kHz  139 kHz  135.6 kHz   set rx to f - 2kHz USB\n\n", flow, fhigh);
            break;

        case FSK_ASCII_300_BAUD_850Hz:
            baud_rate = 300.0f;
            lpf_alpha = 0.07f;
            flow = 1275.0f;
            fhigh = 2125.0f;
            smBitClock = BitTransBuf; 
            inverse_fsk = false; // Standard FSK (mark = high frequency, space = low frequency)
            smDecoding = process_ascii;
            wprintf(L"\n\nModus FSK_ASCII_300_BAUD  %g Hz / %g Hz    f = 438.450 MHz, 438.550 MHz  FM\n\n", flow, fhigh);
            break;

        case FSK_AX25_1200_BAUD_1000Hz:
            baud_rate = 1200.0f;
            lpf_alpha = 0.10f;
            flow = 1200.0f;
            fhigh = 2200.0f;
            smBitClock = BitTransAvg_0;
            inverse_fsk = false; // Standard FSK (mark = high frequency, space = low frequency)
            smDecoding = process_ax25;
            wprintf(L"\n\nModus FSK_AX25_1200_BAUD  %g Hz / %g Hz   f = 438.450 MHz, 438.550 MHz, 144.800 MHz  FM\n\n", flow, fhigh);
            break;

        default:
        printf("Ungültiger FSK-Modus.\n");
        return;
    }

    nco_step = 2 * (float)M_PI * (flow + (fhigh - flow) / 2) / SAMPLING_RATE;
    disc_alpha = lpf_alpha;   
    samples_per_bit = SAMPLING_RATE / baud_rate;
    samples_per_half_bit = samples_per_bit * 0.5f;
    bit_timer = samples_per_bit;
    disc_acc = 0.0f;
	disc_acc_dbg = 0.0f;    
	bit_avg_threshold = samples_per_bit * 0.12f;   // 12% der Bitdauer

	init_trans_ringbuffer();
}

// -------------------------------------------------------------------------------------- 

static void BitTransSimple(float disc)
{
    static int bit = 0;
    static int prev_bit = 0;
    bit = (disc_filt >= 0) ? 1 : 0;
    if (bit != prev_bit)
    {
        bit_timer = (bit_timer + samples_per_half_bit) / 2;
        prev_bit = bit;
        disc_acc = 0.0f;
    }
}

// -------------------------------------------------------------------------------------- 

static void BitTransAvg_0(float disc)
{
    if (disc > 0)
    {
        bit_avg++;
        if (bit_avg > bit_avg_threshold)
        {
            bit_timer = (bit_timer + samples_per_half_bit + bit_avg_threshold) / 2;
            bit_avg = 0;
            disc_acc = 0.0f;
            smBitClock = BitTransAvg_1;
            return;
        }
        return;
    }
    bit_avg = 0;
}

static void BitTransAvg_1(float disc)
{
    if (disc < 0)
    {
        bit_avg++;
        if (bit_avg > bit_avg_threshold)
        {
            bit_timer = (bit_timer + samples_per_half_bit + bit_avg_threshold) / 2;
            bit_avg = 0;
            disc_acc = 0.0f;
            smBitClock = BitTransAvg_0;
            return;
        }
        return;
    }
    bit_avg = 0;
}

// -------------------------------------------------------------------------------------- 

#define SAMPLE_RINGBUFFER_MAX 256
#define SAMPLE_RINGBUFFER_FACTOR 0.2f
#define SAMPLE_RINGBUFFER_ZERO_WINDOW_RATIO 0.2f

static float sample_ringbuffer[SAMPLE_RINGBUFFER_MAX];
static float* sample_ringbuffer_write_ptr = sample_ringbuffer;
static float* sample_ringbuffer_end_ptr = sample_ringbuffer;
static int sample_ringbuffer_size = 0;
static float sample_ringbuffer_sum = 0.0f;
static float sample_ringbuffer_abs_sum = 0.0f;
static int sample_ringbuffer_rearm_count = 0;

static void init_trans_ringbuffer()
{
    sample_ringbuffer_size = (int)(samples_per_bit * SAMPLE_RINGBUFFER_FACTOR + 0.5f);
    if (sample_ringbuffer_size < 1)
    {
        sample_ringbuffer_size = 1;
    }
    if (sample_ringbuffer_size > SAMPLE_RINGBUFFER_MAX)
    {
        sample_ringbuffer_size = SAMPLE_RINGBUFFER_MAX;
    }

    sample_ringbuffer_write_ptr = sample_ringbuffer;
    sample_ringbuffer_end_ptr = sample_ringbuffer + sample_ringbuffer_size;
    sample_ringbuffer_sum = 0.0f;
    sample_ringbuffer_abs_sum = 0.0f;
    sample_ringbuffer_rearm_count = sample_ringbuffer_size;

    for (int i = 0; i < sample_ringbuffer_size; ++i)
    {
        sample_ringbuffer[i] = 0.0f;
    }
}

static void push_sample_to_ringbuffer(float sample)
{
    float sum = 0.0f;
    float abs_sum = 0.0f;
    float* sample_ptr;

    if (sample_ringbuffer_size <= 0)
    {
        sample_ringbuffer_sum = 0.0f;
        sample_ringbuffer_abs_sum = 0.0f;
        return;
    }

    *sample_ringbuffer_write_ptr = sample;
    sample_ringbuffer_write_ptr++;

    if (sample_ringbuffer_write_ptr >= sample_ringbuffer_end_ptr)
    {
        sample_ringbuffer_write_ptr = sample_ringbuffer;
    }

    for (sample_ptr = sample_ringbuffer; sample_ptr < sample_ringbuffer_end_ptr; ++sample_ptr)
    {
        sum += *sample_ptr;
        abs_sum += fabsf(*sample_ptr);
    }

    sample_ringbuffer_sum = sum;
    sample_ringbuffer_abs_sum = abs_sum;
}

static bool sample_ringbuffer_sum_is_in_zero_window(void)
{
    if (sample_ringbuffer_abs_sum <= 0.0f)
    {
        return false;
    }

    return fabsf(sample_ringbuffer_sum) <= (sample_ringbuffer_abs_sum * SAMPLE_RINGBUFFER_ZERO_WINDOW_RATIO);
}

static void BitTransBuf(float disc)
{
    push_sample_to_ringbuffer(disc_filt * 100);

    if (sample_ringbuffer_rearm_count > 0)
    {
        sample_ringbuffer_rearm_count--;
    }

    if ((sample_ringbuffer_rearm_count == 0) && sample_ringbuffer_sum_is_in_zero_window())
    {
        float target_bit_timer = samples_per_half_bit + (float)sample_ringbuffer_size * 0.5f;
        bit_timer = (bit_timer + target_bit_timer) / 2.0f;
        disc_acc = 0.0f;
        sample_ringbuffer_rearm_count = sample_ringbuffer_size;
    }

    return;
}
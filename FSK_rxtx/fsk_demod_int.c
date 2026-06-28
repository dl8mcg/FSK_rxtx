/*
*   by dl8mcg Jan. 2025 to June 2026       FSK-demodulator-detector-decoder fixed-point implementation
*/

#include <stdint.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include "config.h"
#include "fsk_demod_int.h"
#include "fsk_decode_ascii.h"
#include "fsk_decode_rtty.h"
#include "fsk_decode_ax25.h"
#include "fsk_decode_efr.h"
#include "fsk_decode_ax25_g3ruh.h"
#include "fsk_decode_sitorb.h"

FskAmplitudes process_fsk_demod_baseband_int(int32_t sample_q31);
FskAmplitudes process_fsk_demod_center_nco_int(int32_t sample_q31);
FskAmplitudes(*smDemod_int)(int32_t) = process_fsk_demod_baseband_int;

static void (*smDecoding)(uint8_t) = process_rtty;

// ------------------------------------------------------------
// Fixed-point format
// ------------------------------------------------------------

#define Q31_SHIFT          31
#define Q30_SHIFT          30
#define Q31_SCALE          2147483648.0
#define Q30_SCALE          1073741824.0
#define Q31_DEBUG_TENTH    ((int32_t)214748365)

static inline int64_t sat_add_i64(int64_t a, int64_t b)
{
    if ((b > 0) && (a > INT64_MAX - b)) return INT64_MAX;
    if ((b < 0) && (a < INT64_MIN - b)) return INT64_MIN;
    return a + b;
}

static inline int64_t sat_sub_i64(int64_t a, int64_t b)
{
    if ((b > 0) && (a < INT64_MIN + b)) return INT64_MIN;
    if ((b < 0) && (a > INT64_MAX + b)) return INT64_MAX;
    return a - b;
}

static inline int32_t sat_i32_from_i64(int64_t v)
{
    if (v > (int64_t)INT32_MAX) return INT32_MAX;
    if (v < (int64_t)INT32_MIN) return INT32_MIN;
    return (int32_t)v;
}

static inline int32_t round_shift_i64_to_i32(int64_t v, unsigned shift)
{
    if (shift == 0)
        return sat_i32_from_i64(v);

    const int64_t half = (int64_t)1 << (shift - 1);

    if (v >= 0)
        return sat_i32_from_i64(sat_add_i64(v, half) >> shift);

    if (v == INT64_MIN)
        return INT32_MIN;

    const int64_t rounded = sat_add_i64(-v, half) >> shift;
    return sat_i32_from_i64(-rounded);
}

static inline int32_t q31_mul_q31(int32_t a_q31, int32_t b_q31)
{
    return round_shift_i64_to_i32((int64_t)a_q31 * (int64_t)b_q31, Q31_SHIFT);
}

static inline int32_t q31_from_double(double x)
{
    if (!isfinite(x)) return 0;
    if (x >= 1.0) return INT32_MAX;
    if (x <= -1.0) return INT32_MIN;
    return sat_i32_from_i64((int64_t)llround(x * Q31_SCALE));
}

static inline int32_t q30_from_double(double x)
{
    if (!isfinite(x)) return 0;
    return sat_i32_from_i64((int64_t)llround(x * Q30_SCALE));
}

static inline float q31_to_float(int32_t x_q31)
{
    return (float)((double)x_q31 / Q31_SCALE);
}

// ------------------------------------------------------------
// FSK settings
// ------------------------------------------------------------

static float baud_rate;
static volatile bool inverse_fsk = false;

// ------------------------------------------------------------
// NCO - center
// ------------------------------------------------------------

#define NCO_TABLE_SIZE 1024
static int32_t sin_table[NCO_TABLE_SIZE];

static uint32_t nco_phase_acc = 0;
static uint32_t nco_step_acc = 0;

// ------------------------------------------------------------
// frequency discriminator
// ------------------------------------------------------------

static int32_t prev_i = 0;
static int32_t prev_q = 0;
static int32_t disc_filt = 0;

// ------------------------------------------------------------
// bit clock
// ------------------------------------------------------------

#define BIT_TIMER_SCALE  256

static int32_t bit_timer = 0;
static int32_t samples_per_bit = 0;
static int32_t samples_per_half_bit = 0;
static int32_t samples_per_half_bit_plus_tol = 0;
static int32_t samples_per_half_bit_minus_tol = 0;

static bool toggle_strobe_dbg = false;

static void BitClockSlowTrack0(int);
static void BitClockSlowTrack1(int);
static void (*smBitClockSlowTrack)(int) = BitClockSlowTrack0;

static void BitClockFastTrack0(int);
static void BitClockFastTrack1(int);
static void (*smBitClockFastTrack)(int) = BitClockFastTrack0;

typedef struct
{
    int32_t b0, b1, b2;
    int32_t a1, a2;
    int64_t z1;
    int64_t z2;
} biquad_t;

static biquad_t lp1, lp1i, lp2i, lp3i, lp1q, lp2q, lp3q, lpdisc;

static void biquad_clear(biquad_t* bq)
{
    bq->z1 = 0;
    bq->z2 = 0;
}

static void biquad_init_lowpass(biquad_t* bq, double fs, double fc, double Q)
{
    const double w0 = 2.0 * M_PI * (fc / fs);
    const double cw0 = cos(w0);
    const double sw0 = sin(w0);
    const double alpha = sw0 / (2.0 * Q);

    const double b0 = (1.0 - cw0) * 0.5;
    const double b1 = 1.0 - cw0;
    const double b2 = (1.0 - cw0) * 0.5;
    const double a0 = 1.0 + alpha;
    const double a1 = -2.0 * cw0;
    const double a2 = 1.0 - alpha;

    bq->b0 = q30_from_double(b0 / a0);
    bq->b1 = q30_from_double(b1 / a0);
    bq->b2 = q30_from_double(b2 / a0);
    bq->a1 = q30_from_double(a1 / a0);
    bq->a2 = q30_from_double(a2 / a0);

    biquad_clear(bq);
}

static inline int32_t biquad_process_sample(biquad_t* bq, int32_t x_q31)
{
    const int64_t b0x = (int64_t)bq->b0 * (int64_t)x_q31;
    const int64_t acc = sat_add_i64(b0x, bq->z1);
    const int32_t y_q31 = round_shift_i64_to_i32(acc, Q30_SHIFT);

    const int64_t b1x = (int64_t)bq->b1 * (int64_t)x_q31;
    const int64_t a1y = (int64_t)bq->a1 * (int64_t)y_q31;
    const int64_t b2x = (int64_t)bq->b2 * (int64_t)x_q31;
    const int64_t a2y = (int64_t)bq->a2 * (int64_t)y_q31;

    bq->z1 = sat_add_i64(sat_sub_i64(b1x, a1y), bq->z2);
    bq->z2 = sat_sub_i64(b2x, a2y);

    return y_q31;
}

static void init_filter_45bd_170sh(void)
{
    const double fs = SAMPLING_RATE;
    const double fc = 400.0;
    const double Q = 0.5;

    const double fc_disc = 45.0;
    const double Q_disc = 0.6;

    biquad_init_lowpass(&lp1i, fs, fc, Q);
    biquad_init_lowpass(&lp2i, fs, fc, Q);
    biquad_init_lowpass(&lp3i, fs, fc, Q);
    biquad_init_lowpass(&lp1q, fs, fc, Q);
    biquad_init_lowpass(&lp2q, fs, fc, Q);
    biquad_init_lowpass(&lp3q, fs, fc, Q);
    biquad_init_lowpass(&lpdisc, fs, fc_disc, Q_disc);
}

static void init_filter_50bd_85sh(void)
{
    const double fs = SAMPLING_RATE;
    const double fc = 300.0;
    const double Q = 0.3;

    const double fc_disc = 50.0;
	const double Q_disc = 0.6;

    biquad_init_lowpass(&lp1i, fs, fc, Q);
    biquad_init_lowpass(&lp2i, fs, fc, Q);
    biquad_init_lowpass(&lp3i, fs, fc, Q);
    biquad_init_lowpass(&lp1q, fs, fc, Q);
    biquad_init_lowpass(&lp2q, fs, fc, Q);
    biquad_init_lowpass(&lp3q, fs, fc, Q);
    biquad_init_lowpass(&lpdisc, fs, fc_disc, Q_disc);
}

static void init_filter_50bd_450sh(void)
{
    const double fs = SAMPLING_RATE;
    const double fc = 600.0;
    const double Q = 0.5;

	const double fc_disc = 50.0;
	const double Q_disc = 0.6;

    biquad_init_lowpass(&lp1i, fs, fc, Q);
    biquad_init_lowpass(&lp2i, fs, fc, Q);
    biquad_init_lowpass(&lp3i, fs, fc, Q);
    biquad_init_lowpass(&lp1q, fs, fc, Q);
    biquad_init_lowpass(&lp2q, fs, fc, Q);
    biquad_init_lowpass(&lp3q, fs, fc, Q);
    biquad_init_lowpass(&lpdisc, fs, fc_disc, Q_disc);
}

static void init_filter_100bd_170sh(void)
{
    const double fs = SAMPLING_RATE;
    const double fc = 500.0;
    const double Q = 0.6;

    const double fc_disc = 100.0;
    const double Q_disc = 0.6;

    biquad_init_lowpass(&lp1i, fs, fc, Q);
    biquad_init_lowpass(&lp2i, fs, fc, Q);
    biquad_init_lowpass(&lp3i, fs, fc, Q);
    biquad_init_lowpass(&lp1q, fs, fc, Q);
    biquad_init_lowpass(&lp2q, fs, fc, Q);
    biquad_init_lowpass(&lp3q, fs, fc, Q);
    biquad_init_lowpass(&lpdisc, fs, fc_disc, Q_disc);
}

static void init_filter_200bd_340sh(void)
{
    const double fs = SAMPLING_RATE;
    const double fc = 800.0;
    const double Q = 0.6;

    const double fc_disc = 200.0;
    const double Q_disc = 0.6;

    biquad_init_lowpass(&lp1i, fs, fc, Q);
    biquad_init_lowpass(&lp2i, fs, fc, Q);
    biquad_init_lowpass(&lp3i, fs, fc, Q);
    biquad_init_lowpass(&lp1q, fs, fc, Q);
    biquad_init_lowpass(&lp2q, fs, fc, Q);
    biquad_init_lowpass(&lp3q, fs, fc, Q);
    biquad_init_lowpass(&lpdisc, fs, fc_disc, Q_disc);
}

static void init_filter_300bd_850sh(void)
{
    const double fs = SAMPLING_RATE;
    const double fc = 1250.0;
    const double Q = 0.6;

    const double fc_disc = 300.0;
    const double Q_disc = 0.6;

    biquad_init_lowpass(&lp1i, fs, fc, Q);
    biquad_init_lowpass(&lp2i, fs, fc, Q);
    biquad_init_lowpass(&lp3i, fs, fc, Q);
    biquad_init_lowpass(&lp1q, fs, fc, Q);
    biquad_init_lowpass(&lp2q, fs, fc, Q);
    biquad_init_lowpass(&lp3q, fs, fc, Q);
    biquad_init_lowpass(&lpdisc, fs, fc_disc, Q_disc);
}

static void init_filter_1k2_1000sh(void)
{
    const double fs = SAMPLING_RATE;
    const double fc = 1250.0;
    const double Q = 0.6;

    const double fc_disc = 1200.0;
    const double Q_disc = 0.6;

    biquad_init_lowpass(&lp1i, fs, fc, Q);
    biquad_init_lowpass(&lp2i, fs, fc, Q);
    biquad_init_lowpass(&lp3i, fs, fc, Q);
    biquad_init_lowpass(&lp1q, fs, fc, Q);
    biquad_init_lowpass(&lp2q, fs, fc, Q);
    biquad_init_lowpass(&lp3q, fs, fc, Q);
    biquad_init_lowpass(&lpdisc, fs, fc_disc, Q_disc);
}

static void init_filter_9k6(void)
{
    const double fs = SAMPLING_RATE;
    const double fc = 9600.0;
    const double Q = 0.707;

    biquad_init_lowpass(&lp1, fs, fc, Q);
}

// ---------------------------------------------------------------
// 9600bps baseband - IIR Lowpassfilter - Demodulate Detect Decode
// ---------------------------------------------------------------

static void BitClockSlowTrack0(int bit)
{
    bit_timer += BIT_TIMER_SCALE;

    if (bit_timer >= samples_per_bit)
    {
        bit_timer -= samples_per_bit;
        smDecoding(0);
        toggle_strobe_dbg ^= 1;
        return;
    }

    if (bit == 1)
    {
        smBitClockSlowTrack = BitClockSlowTrack1;
		if (bit_timer > samples_per_half_bit_plus_tol) 
        { 
            bit_timer -= samples_per_bit / 32;
            return;
        }
        if (bit_timer < samples_per_half_bit_minus_tol)
        {
            bit_timer += samples_per_bit / 32;
            return;
        }
    }
}

static void BitClockSlowTrack1(int bit)
{
    bit_timer += BIT_TIMER_SCALE;

    if (bit_timer >= samples_per_bit)
    {
        bit_timer -= samples_per_bit;
        smDecoding(1);
        toggle_strobe_dbg ^= 1;
        return;
    }

    if (bit == 0)
    {
        smBitClockSlowTrack = BitClockSlowTrack0;
		if (bit_timer > samples_per_half_bit_plus_tol) 
        {
            bit_timer -= samples_per_bit / 32;
			return;
        }
        
		if (bit_timer < samples_per_half_bit_minus_tol) 
        {
            bit_timer += samples_per_bit / 32;
            return;
        }
    }
}

static FskAmplitudes process_fsk_demod_baseband_int(int32_t sample_q31)
{
    static int bit = 0;

    const int32_t sample_lpf_q31 = biquad_process_sample(&lp1, sample_q31);


    bit = (sample_lpf_q31 >= 0) ? 1 : 0;
    smBitClockSlowTrack(bit);

    FskAmplitudes amps;
    amps.amp2 = q31_to_float(toggle_strobe_dbg ? Q31_DEBUG_TENTH : -Q31_DEBUG_TENTH);
	amps.amp1 = q31_to_float(sample_lpf_q31);

    return amps;
}

// --------------------------------------------------------------------
// Mix down - center NCO - IIR Lowpassfilter - Demodulate Detect Decode
// --------------------------------------------------------------------

static void BitClockFastTrack0(int bit)
{
    bit_timer += BIT_TIMER_SCALE;

    if (bit_timer >= samples_per_bit)
    {
        bit_timer -= samples_per_bit;
        smDecoding(0);
        toggle_strobe_dbg ^= 1;
        return;
    }

    if (bit == 1)
    {
        smBitClockFastTrack = BitClockFastTrack1;

        if (bit_timer > samples_per_half_bit_plus_tol)
        {
            bit_timer -= samples_per_bit / 128;
            return;
        }

        if (bit_timer < samples_per_half_bit_minus_tol)
        {
            bit_timer += samples_per_bit / 128;
            return;
        }
    }
}

static void BitClockFastTrack1(int bit)
{
    bit_timer += BIT_TIMER_SCALE;

    if (bit_timer >= samples_per_bit)
    {
        bit_timer -= samples_per_bit;
        smDecoding(1);
        toggle_strobe_dbg ^= 1;
        return;
    }

    if (bit == 0)
    {
        smBitClockFastTrack = BitClockFastTrack0;

        if (bit_timer > samples_per_half_bit_plus_tol)
        {
            bit_timer -= samples_per_bit / 128;
            return;
        }

        if (bit_timer < samples_per_half_bit_minus_tol)
        {
            bit_timer += samples_per_bit / 128;
            return;
        }

    }
}

static FskAmplitudes process_fsk_demod_center_nco_int(int32_t sample_q31)
{
    static int bit = 0;

    nco_phase_acc += nco_step_acc;

    const uint32_t idx = nco_phase_acc >> 22;
    const int32_t sin_q31 = sin_table[idx];
    const int32_t cos_q31 = sin_table[(idx + 256U) & 0x3FFU];

    int32_t mix_i_q31 = q31_mul_q31(sample_q31, cos_q31);
    int32_t mix_q_q31 = q31_mul_q31(sample_q31, sin_q31);

    mix_i_q31 = biquad_process_sample(&lp1i, mix_i_q31);
    mix_i_q31 = biquad_process_sample(&lp2i, mix_i_q31);
    mix_i_q31 = biquad_process_sample(&lp3i, mix_i_q31);

    mix_q_q31 = biquad_process_sample(&lp1q, mix_q_q31);
    mix_q_q31 = biquad_process_sample(&lp2q, mix_q_q31);
    mix_q_q31 = biquad_process_sample(&lp3q, mix_q_q31);

    const int64_t iq = (int64_t)mix_i_q31 * (int64_t)prev_q;
    const int64_t qi = (int64_t)mix_q_q31 * (int64_t)prev_i;
    const int32_t disc_q31 = round_shift_i64_to_i32(sat_sub_i64(iq, qi), Q31_SHIFT);

    prev_i = mix_i_q31;
    prev_q = mix_q_q31;

    disc_filt = biquad_process_sample(&lpdisc, disc_q31);

    bit = inverse_fsk ? (disc_filt < 0 ? 1 : 0) : (disc_filt > 0 ? 1 : 0);

    smBitClockFastTrack(bit);

    FskAmplitudes amps;
    amps.amp2 = q31_to_float(toggle_strobe_dbg ? -Q31_DEBUG_TENTH : Q31_DEBUG_TENTH);
    amps.amp1 = q31_to_float(disc_filt);
    return amps;
}

// -----------------------------
// Init mode specific parameters
// -----------------------------

static void reset_demod_runtime_state(void)
{
    prev_i = 0;
    prev_q = 0;
    disc_filt = 0;
    bit_timer = 0;
    nco_phase_acc = 0;
    toggle_strobe_dbg = false;
    smBitClockSlowTrack = BitClockSlowTrack0;
    smBitClockFastTrack = BitClockFastTrack0;
}

void init_fsk_demod_int(FskMode mode)
{
    double flow = 1.0;
    double fhigh = 2.0;

    switch (mode)
    {
    case FSK_RTTY_45_BAUD_170Hz:
        init_filter_45bd_170sh();
        smDemod_int = process_fsk_demod_center_nco_int;
        baud_rate = 2.0f * 45.454545f;      // doppelte Baudrate, wegen 1.5 Stopitlänge
        flow = 2125.0;
        fhigh = 2295.0;
        inverse_fsk = false;
        smDecoding = process_rtty_uos;
        wprintf(L"\n\nModus FSK_RTTY_45_BAUD  %g Hz / %g Hz  set rx to usb   or  f = 438.450 MHz, 438.550 MHz  FM\n\n", flow, fhigh);
        break;

    case FSK_RTTY_50_BAUD_85Hz:
        init_filter_50bd_85sh();
        smDemod_int = process_fsk_demod_center_nco_int;
        baud_rate = 2.0f * 50.0f;           // doppelte Baudrate, wegen 1.5 Stopitlänge
        flow = 1957.5;
        fhigh = 2042.5;
        inverse_fsk = true;
        smDecoding = process_rtty;
        wprintf(L"\n\nModus FSK_RTTY_50_BAUD  %g Hz / %g Hz   DDH47 : 147.3 kHz   set rx to f - 2kHz USB\n\n", flow, fhigh);
        break;

    case FSK_RTTY_50_BAUD_450Hz:
        init_filter_50bd_450sh();
        smDemod_int = process_fsk_demod_center_nco_int;
        baud_rate = 2.0f * 50.0f;           // doppelte Baudrate, wegen 1.5 Stopitlänge     
        flow = 1775.0;
        fhigh = 2225.0;
        inverse_fsk = true;
        smDecoding = process_rtty;
        wprintf(L"\n\nModus FSK_RTTY_50_BAUD  %g Hz / %g Hz    DDK2 : 4583 kHz   DDH7 : 7646 kHz   DDK9 : 10100.8 kHz   DDH9 : 11039 kHz   DDH8 : 14467.3 kHz  set rx to f - 2kHz USB\n\n", flow, fhigh);
        break;

    case FSK_EFR_200_BAUD_340Hz:
        init_filter_200bd_340sh();
        smDemod_int = process_fsk_demod_center_nco_int;
        baud_rate = 200.0f;
        flow = 1830.0;
        fhigh = 2170.0;
        inverse_fsk = true;
        smDecoding = process_efr;
        wprintf(L"\n\nModus FSK_EFR_200_BAUD  %g Hz / %g Hz    DCF49 : 129.1 kHz    HGA22 : 135.6 kHz    DCF39 : 139 kHz    set rx to f - 2kHz USB\n\n", flow, fhigh);
        break;

    case FSK_ASCII_300_BAUD_850Hz:
        init_filter_300bd_850sh();
        smDemod_int = process_fsk_demod_center_nco_int;
        baud_rate = 300.0f;
        flow = 1275.0;
        fhigh = 2125.0;
        inverse_fsk = false;
        smDecoding = process_ascii;
        wprintf(L"\n\nModus FSK_ASCII_300_BAUD  %g Hz / %g Hz    f = 438.450 MHz, 438.550 MHz  FM\n\n", flow, fhigh);
        break;

    case FSK_AX25_1200_BAUD_1000Hz:
        init_filter_1k2_1000sh();
        smDemod_int = process_fsk_demod_center_nco_int;
        baud_rate = 1200.0f;
        flow = 1200.0;
        fhigh = 2200.0;
        inverse_fsk = false;
        smDecoding = process_ax25;
        wprintf(L"\n\nModus FSK_AX25_1200_BAUD  %g Hz / %g Hz   f = 438.450 MHz, 438.550 MHz, 144.800 MHz  FM\n\n", flow, fhigh);
        break;

    case FSK_AX25_9600_BAUD:
        init_filter_9k6();
        smDemod_int = process_fsk_demod_baseband_int;
        baud_rate = 9600.0f;
        smDecoding = process_ax25_g3ruh;
        wprintf(L"\n\nModus FSK_AX25_9600_BAUD    baseband   f = 439.850 MHz  \n\n");
        break;

    case FSK_SITORB_100_BAUD:
        init_filter_100bd_170sh();
        smDemod_int = process_fsk_demod_center_nco_int;
        baud_rate = 100.0f;
        flow = 900 - 170.0/2;
        fhigh = 900 + 170.0/2;
        inverse_fsk = false;
        sitorb_reset();
        smDecoding = process_sitorb;
        wprintf(L"\n\nModus FSK_SITORB_100_BAUD  %g Hz / %g Hz   f = 490 kHz, 518 kHz, 4209.5 kHz, 12579 kHz    set rx to f - 900 Hz USB\n\n", flow, fhigh);
        break;

    default:
        printf("Ungueltiger FSK-Modus.\n");
        return;
    }

    for (int i = 0; i < NCO_TABLE_SIZE; i++)
		sin_table[i] = q31_from_double(sin(2.0 * M_PI * (double)i / (double)NCO_TABLE_SIZE));   // sinus table for NCO

    const double fcenter = (flow + fhigh) * 0.5;
    nco_step_acc = (uint32_t)llround((fcenter / (double)SAMPLING_RATE) * 4294967296.0);

    samples_per_bit = (int32_t)llround(((double)SAMPLING_RATE / (double)baud_rate) * (double)BIT_TIMER_SCALE);
    samples_per_half_bit = (int32_t)llround(((double)SAMPLING_RATE / (double)baud_rate) * (double)BIT_TIMER_SCALE / (double)2.0);
    samples_per_half_bit_plus_tol = (int32_t)llround(((double)SAMPLING_RATE / (double)baud_rate) * (double)BIT_TIMER_SCALE / (double)2.0 * 1.01);
    samples_per_half_bit_minus_tol = (int32_t)llround(((double)SAMPLING_RATE / (double)baud_rate) * (double)BIT_TIMER_SCALE / (double)2.0 * 0.99);

    reset_demod_runtime_state();
}

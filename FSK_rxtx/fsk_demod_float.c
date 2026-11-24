/*
*   by dl8mcg Jan. 2025 to June 2026       FSK-demodulator-detector-decoder
*/

#include <stdint.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include "config.h"
#include "fsk_demod_float.h"            // Version mit Fließkomma-Berechnung
#include "fsk_decode_ascii.h"
#include "fsk_decode_rtty.h"
#include "fsk_decode_ax25.h"
#include "fsk_decode_efr.h"
#include "fsk_decode_ax25_g3ruh.h"
#include "fsk_decode_sitorb.h"

 FskAmplitudes process_fsk_demod_baseband_float(float sample);
 FskAmplitudes process_fsk_demod_center_nco_float(float sample);
 FskAmplitudes(*smDemod_float)(float) = process_fsk_demod_center_nco_float;      // State Machine Demodulate and Detect

static void (*smDecoding)(uint8_t) = process_rtty;                  // State Machine Decode

// ------------------------------------------------------------
// FSK settings
// ------------------------------------------------------------

static float baud_rate;
volatile bool inverse_fsk = false;

// ------------------------------------------------------------
// NCO - center
// ------------------------------------------------------------

static float nco_phase_acc = 0.0f;
static float nco_step_acc = 0.0f;

// ------------------------------------------------------------
// frequency discriminator
// ------------------------------------------------------------

static float prev_i = 0;
static float prev_q = 0;
static float disc_filt = 0;

// ------------------------------------------------------------
// bit clock
// ------------------------------------------------------------

#define BIT_TIMER_SCALE  256                // Fixkomma-Skalierung: 8 Bit Nachkommanteil

static int32_t bit_timer = 0;
static int32_t samples_per_bit = 0;         // enthält den skalierten Wert
static int32_t samples_per_half_bit = 0;
static int32_t samples_per_half_bit_plus_tol = 0;
static int32_t samples_per_half_bit_minus_tol = 0;
static int32_t delta_bit_timer = 0;
static int32_t fraction_bit_clock = 128;                 // Skalierung für die Bit-Clock Recovery

static bool toggle_strobe_dbg = false;

static void BitClockTrack0(int);
static void BitClockTrack1(int);
static void (*smBitClockTrack)(int) = BitClockTrack0;

typedef struct                                                                  // IIR Biquad-Filter-Struktur
{
    float b0, b1, b2;
    float a1, a2;
    float z1, z2; 
} biquad_t;

biquad_t lp1, lp1i, lp2i, lp3i,  lp1q, lp2q, lp3q,  lpdisc;                     // IIR Tiefpassfilter - Variablen

static void biquad_init_lowpass(biquad_t* bq, float fs, float fc, float Q)      // IIR - Tiefpassfilter Koeffizientenberechnung
{
    float w0 = 2.0f * (float)M_PI * (fc / fs);
    float cw0 = cosf(w0);
    float sw0 = sinf(w0);
    float alpha = sw0 / (2.0f * Q);

    float b0 = (1.0f - cw0) * 0.5f;
    float b1 = 1.0f - cw0;
    float b2 = (1.0f - cw0) * 0.5f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cw0;
    float a2 = 1.0f - alpha;

    bq->b0 = b0 / a0;
    bq->b1 = b1 / a0;
    bq->b2 = b2 / a0;
    bq->a1 = a1 / a0;
    bq->a2 = a2 / a0;

    bq->z1 = 0.0f;
    bq->z2 = 0.0f;
}

static float biquad_process_sample(biquad_t* bq, float x)
{
    float y = bq->b0 * x + bq->z1;
    bq->z1 = bq->b1 * x - bq->a1 * y + bq->z2;
    bq->z2 = bq->b2 * x - bq->a2 * y;
    return y;
}

static void init_filter_45bd_170sh(void)
{
    float fs = SAMPLING_RATE;
    float fc = 400.0f;      // Filtereckfrequenz : 2 x shift + datenrate
    float Q = 0.5f;         // leicht gedämpfter Filterverlauf 

    const float fc_disc = 45.0;
    const float Q_disc = 0.6;

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
    const float fs = SAMPLING_RATE;
    const float fc = 300.0f;        // Filtereckfrequenz : 2 x shift + datenrate
    float Q = 0.3f;                 // leicht gedämpfter Filterverlauf 

    const float fc_disc = 50.0;
    const float Q_disc = 0.6;

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
    const float fs = SAMPLING_RATE;
    const float fc = 600.0f;      // Filtereckfrequenz etwas kleiner als 2 x shift + datenrate
    const float Q = 0.5f;         // leicht gedämpfter Filterverlauf

    const float fc_disc = 50.0;
    const float Q_disc = 0.6;

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
    float fs = SAMPLING_RATE;
    float fc = 800.0f;      // Filtereckfrequenz etwas kleiner als 2 x shift + datenrate
    float Q = 0.6f;         // leit gedämpfter Filterverlauf

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
    float fs = SAMPLING_RATE;
    const float fc = 1250.0f;     // Filtereckfrequenz etwas kleiner als 2 x shift + datenrate, weil die Audio-FSK-Frequenzen recht dicht dran liegen
	const float Q = 0.6f;         // leit gedämpfter Filterverlauf

    const float fc_disc = 300.0f;
    const float Q_disc = 0.6f;

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
    const float fs = SAMPLING_RATE;
    const float fc = 1250.0f;           // Filtereckfrequenz etwas kleiner als 2 x shift + datenrate, weil die Audio-FSK-Frequenzen recht dicht dran liegen
    const float Q = 0.6f;               // leit gedämpfter Filterverlauf

    const float fc_disc = 1200.0f;
    const float Q_disc = 0.6f;

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
    float fs = SAMPLING_RATE;
    float fc = 9600.f;      // Filtereckfrequenz : datenrate
	float Q = 0.707f;       // Filterverlauf ohne Überschwingen

    biquad_init_lowpass(&lp1, fs, fc, Q);
}


static void BitClockTrack0(int bit)
{
    bit_timer += BIT_TIMER_SCALE;                       // Bit-Timer hochzählen, skaliert mit BIT_TIMER_SCALE für die Fixkomma-Statemachine
    if (bit_timer >= samples_per_bit)                   // Abtastung der Mitte des Bits erreicht
    {
        bit_timer -= samples_per_bit;                   // Zähler um eine Bitdauer zurücksetzen
        smDecoding(0);                                  // Detektion eines "0"-Bits
        toggle_strobe_dbg ^= 1;                         // Debug: Zeigt die Bit-Abtastung an 
        return;
    }
    if (bit == 1)                                       // Bitzustand wechselt von "0" auf "1"
    {
        smBitClockTrack = BitClockTrack1;

        if (bit_timer > samples_per_half_bit_plus_tol)
        {
            bit_timer -= delta_bit_timer;
            return;
        }

        if (bit_timer < samples_per_half_bit_minus_tol)
        {
            bit_timer += delta_bit_timer;
            return;
        }
    }
}

static void BitClockTrack1(int bit)                        // Zustand der Basisband-Demodulator-Statemachine: "1"-Bit
{
    bit_timer += BIT_TIMER_SCALE;
    if (bit_timer >= samples_per_bit)                   // Abtastung der Mitte des Bits erreicht
    {
        bit_timer -= samples_per_bit;                   // Zähler um eine Bitdauer zurücksetzen
        smDecoding(1);                                  // Detektion eines "1"-Bits
        toggle_strobe_dbg ^= 1;                         // Debug: Zeigt die Bit-Abtastung an
        return;
    }
    if (bit == 0)                                       // Bitzustand wechselt von "1" auf "0"
    {
        smBitClockTrack = BitClockTrack0;

        if (bit_timer > samples_per_half_bit_plus_tol)
        {
            bit_timer -= delta_bit_timer;
            return;
        }

        if (bit_timer < samples_per_half_bit_minus_tol)
        {
            bit_timer += delta_bit_timer;
            return;
        }
    }
}


// ---------------------------------------------------------------
// 9600bps baseband - IIR Lowpassfilter - Demodulate Detect Decode
// ---------------------------------------------------------------

static FskAmplitudes process_fsk_demod_baseband_float(float sample)
{
    static int bit = 0;
	static float sample_lpf = 0.0f;

	// IIR Lowpassfilterung des Basisbandsignals
    sample_lpf = biquad_process_sample(&lp1, sample);

    // Bit-Decision
    bit = (sample_lpf  >= 0) ? 1 : 0;                       // hard decision 

	// Bit-Clock Recovery
    smBitClockTrack(bit);                                   // Bit-Clock State Machine

    FskAmplitudes amps;
    amps.amp2 = toggle_strobe_dbg > 0 ? 0.1f : -0.1f;   // Zum Debuggen: Zeigt die Detektion der Bitkante an
	amps.amp1 = sample_lpf;                             // Zum Debuggen: Zeigt den gefilterten Basisbandsample an
    return amps;
}


// --------------------------------------------------------------------
// Mix down - center NCO - IIR Lowpassfilter - Demodulate Detect Decode 
// --------------------------------------------------------------------

static FskAmplitudes process_fsk_demod_center_nco_float(float sample)
{
    sample *= 5.0;                                    // mehr Verstärkung bei Betrieb mit Mikrofon

    static int bit = 0;

    // NCO
    nco_phase_acc += nco_step_acc;
    if (nco_phase_acc > 2 * M_PI)
        nco_phase_acc -= 2 * M_PI;

    // Mixer
    float mix_i = sample * cosf(nco_phase_acc);
    float mix_q = sample * sinf(nco_phase_acc);

    // IIR I/Q Lowpass (jeweils 3 mal Biquads, also insgesamt 6. Ordnung)
    mix_i = biquad_process_sample(&lp1i, mix_i);
    mix_i = biquad_process_sample(&lp2i, mix_i);
    mix_i = biquad_process_sample(&lp3i, mix_i);

    mix_q = biquad_process_sample(&lp1q, mix_q);
    mix_q = biquad_process_sample(&lp2q, mix_q);
    mix_q = biquad_process_sample(&lp3q, mix_q);

    // Frequency Discriminator
    float disc = mix_i * prev_q - mix_q * prev_i;
    prev_i = mix_i;
    prev_q = mix_q;

    // Lowpass-Filterung des Diskriminatorsignals
    disc_filt = biquad_process_sample(&lpdisc, disc);

    // Bit-Decision 
    bit = inverse_fsk ? (disc_filt <= 0 ? 1 : 0) : (disc_filt >= 0 ? 1 : 0);        // hard decision 

	// Bit-Clock Recovery
    smBitClockTrack(bit);                                       // Bit-Clock State Machine

    FskAmplitudes amps;
    amps.amp2 = toggle_strobe_dbg > 0 ? -0.1f : 0.1f;           // Zum Debuggen: Zeigt die Detektion der Bitkante an
    amps.amp1 = disc_filt;                                      // Zum Debuggen: Zeigt den gefilterten Diskriminatorwert an
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
    smBitClockTrack = BitClockTrack0;
}

void init_fsk_demod_float(FskMode mode)
{
    float flow = 1.0f;
    float fhigh = 2.0f;

    inverse_fsk = false;

    switch (mode)
    {
        case FSK_RTTY_45_BAUD_170Hz:
			init_filter_45bd_170sh();
			smDemod_float = process_fsk_demod_center_nco_float;
            baud_rate = 2.0f * 45.454545f;
            flow = 2125.0f;
            fhigh = 2295.0f;
            //flow = 1000 - 170.0 / 2;
            //fhigh = 1000 + 170.0 / 2;
            inverse_fsk = false;                // Standard FSK (mark = high frequency, space = low frequency)
            smDecoding = process_rtty_uos;
			fraction_bit_clock = 128;                 // Skalierung für die Bit-Clock Recovery
            wprintf(L"\n\nModus FSK_RTTY_45_BAUD  %g Hz / %g Hz  set rx to usb   or  f = 438.450 MHz, 438.550 MHz  FM\n\n\n\n", flow, fhigh);
            break;

        case FSK_RTTY_50_BAUD_85Hz:
            init_filter_50bd_85sh();
            smDemod_float = process_fsk_demod_center_nco_float;
            baud_rate = 2.0f * 50.0f;
            //flow = 1957.5f;
            //fhigh = 2042.5f;
            flow = 1000 - 85.0 / 2;
            fhigh = 1000 + 85.0 / 2;
            inverse_fsk = true;                 // Inverse FSK (mark = low frequency, space = high frequency)
            smDecoding = process_rtty;
			fraction_bit_clock = 128;                 // Skalierung für die Bit-Clock Recovery
            wprintf(L"\n\nModus FSK_RTTY_50_BAUD  %g Hz / %g Hz   f = 147.3 kHz   set rx to f - 1kHz USB\n\n", flow, fhigh);
            break;

        case FSK_RTTY_50_BAUD_450Hz:
            init_filter_50bd_450sh();
            smDemod_float = process_fsk_demod_center_nco_float;
            baud_rate = 2.0f * 50.0f;
            //flow = 1775.0f;
            //fhigh = 2225.0f;
            flow = 1000 - 450.0 / 2;
            fhigh = 1000 + 450.0 / 2;
            inverse_fsk = true;                 // Inverse FSK (mark = low frequency, space = high frequency)
            smDecoding = process_rtty;
			fraction_bit_clock = 128;                 // Skalierung für die Bit-Clock Recovery
            wprintf(L"\n\nModus FSK_RTTY_50_BAUD  %g Hz / %g Hz   f = 4583 kHz, 7646 kHz, 10100.8 kHz, 11039 kHz, 14467.3 kHz  set rx to f - 1kHz USB\n\n", flow, fhigh);
            break;

        case FSK_EFR_200_BAUD_340Hz:
			init_filter_200bd_340sh();
            smDemod_float = process_fsk_demod_center_nco_float;
            baud_rate = 200.0f;
            //flow = 1830.0f;
            //fhigh = 2170.0f;
            flow = 1000 - 340.0 / 2;
            fhigh = 1000 + 340.0 / 2;
            inverse_fsk = true;                 // Inverse FSK (mark = low frequency, space = high frequency)
            smDecoding = process_efr;
			fraction_bit_clock = 128;                 // Skalierung für die Bit-Clock Recovery
            wprintf(L"\n\nModus FSK_EFR_200_BAUD  %g Hz / %g Hz   f = 129.1 kHz  135.6 kHz  139 kHz    set rx to f - 1kHz USB\n\n", flow, fhigh);
            break;

        case FSK_ASCII_300_BAUD_850Hz:
            init_filter_300bd_850sh();
            smDemod_float = process_fsk_demod_center_nco_float;
            baud_rate = 300.0f;
            flow = 1275.0f;
            fhigh = 2125.0f;
            inverse_fsk = false;                // Standard FSK (mark = high frequency, space = low frequency)
            smDecoding = process_ascii;
			fraction_bit_clock = 128;                 // Skalierung für die Bit-Clock Recovery
            wprintf(L"\n\nModus FSK_ASCII_300_BAUD  %g Hz / %g Hz    f = 438.450 MHz, 438.550 MHz  FM\n\n", flow, fhigh);
            break;

        case FSK_AX25_1200_BAUD_1000Hz:
			init_filter_1k2_1000sh();
            smDemod_float = process_fsk_demod_center_nco_float;
            baud_rate = 1200.0f;
            flow = 1200.0f;
            fhigh = 2200.0f;
            inverse_fsk = false;                // Standard FSK (mark = high frequency, space = low frequency), bei AX25 egal
            smDecoding = process_ax25;
			fraction_bit_clock = 128;                 // Skalierung für die Bit-Clock Recovery
            wprintf(L"\n\nModus FSK_AX25_1200_BAUD  %g Hz / %g Hz   f = 438.450 MHz, 438.550 MHz, 144.800 MHz  FM\n\n", flow, fhigh);
            break;

		case FSK_AX25_9600_BAUD:
			init_filter_9k6();
            smDemod_float = process_fsk_demod_baseband_float;
            baud_rate = 9600.0f;
            smDecoding = process_ax25_g3ruh;
			fraction_bit_clock = 32;                 // Skalierung für die Bit-Clock Recovery
            wprintf(L"\n\nModus FSK_AX25_9600_BAUD    baseband   f = 439.850 MHz  \n\n");
			break;  

        case FSK_SITORB_100_BAUD:
            init_filter_100bd_170sh();
            smDemod_float = process_fsk_demod_center_nco_float;
            baud_rate = 100.0f;
            flow = 1000 - 170.0 / 2;
            fhigh = 1000 + 170.0 / 2;
            inverse_fsk = false;
            sitorb_reset();
            smDecoding = process_sitorb;
			fraction_bit_clock = 128;                 // Skalierung für die Bit-Clock Recovery
            wprintf(L"\n\nModus FSK_SITORB_100_BAUD  %g Hz / %g Hz   f = 490 kHz, 518 kHz, 4209.5 kHz, 12579 kHz    set rx to f - 1kHz USB\n\n", flow, fhigh);
            break;

        default:
        printf("Ungültiger FSK-Modus.\n");
        return;
    }

	nco_step_acc = 2 * (float)M_PI * (flow + (fhigh - flow) / 2) / SAMPLING_RATE;           // NCO-Schrittweite für die Mitte zwischen den beiden FSK-Frequenzen

    samples_per_bit = (int32_t)llround(((double)SAMPLING_RATE / (double)baud_rate) * (double)BIT_TIMER_SCALE);
    samples_per_half_bit = (int32_t)llround(((double)SAMPLING_RATE / (double)baud_rate) * (double)BIT_TIMER_SCALE / (double)2.0);
    samples_per_half_bit_plus_tol = (int32_t)llround(((double)SAMPLING_RATE / (double)baud_rate) * (double)BIT_TIMER_SCALE / (double)2.0 * 1.01);
    samples_per_half_bit_minus_tol = (int32_t)llround(((double)SAMPLING_RATE / (double)baud_rate) * (double)BIT_TIMER_SCALE / (double)2.0 * 0.99);
    delta_bit_timer = samples_per_bit / fraction_bit_clock;

    reset_demod_runtime_state();
}


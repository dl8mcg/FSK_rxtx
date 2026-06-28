/*
*   by dl8mcg Jan. 2025 to June 2026       2FSK - SITORB / NAVTEX - Decoder
*/

/*
 * First-pass SITOR-B / NAVTEX decoder core.  <-- Codex
 *
 * ITU-R M.476-5 NBDP (Narrow-Band Direct Printing) Telex over Radio
 * Reference: https://www.itu.int/rec/R-REC-M.476-5-199510-I/en
 *
 * NBDP Multiplexierung (Single-Frequency, Time-Division):
 *   RX  = Erste Datenfolge (von Remote-Station A gesendet)
 *   DX  = Zweite Datenfolge (von Remote-Station B oder Redundanz von A)
 *   Alle Daten auf GLEICHER Frequenz, zeitlich verschachtelt (5-Slot-Muster).
 *   Dieser Decoder empfängt BEIDE Datenfolgen gleichzeitig und nutzt sie zur FEC.
 *   Der Begriff "Kanal" ist hier misnomer — besser: "Datenspuren" oder "Sequenzen".
 *
 * Input contract:
 *   process_sitorb(bit) is called once per recovered 100 Bd bit.
 *   bit == 1 should mean the higher FSK tone (ITU "B", mark).
 *
 * Operation:
 *   1. Auto-detects 4 variants: (normal/reversed bit order) × (normal/inverted polarity).
 *   2. Syncs on Mode-B phasing pair:
 *      - CODE_PHASING_1 (0x78): marks RX spur, next slot is DX.
 *      - CODE_PHASING_2 (0x33): marks DX spur, next slot is RX.
 *   3. Fast-Sync fallback: locks when valid characters repeat every 5 slots (FEC property).
 *      Uses parity-based scoring (fec_score[0/1]) per variant/phase combination.
 *   4. After lock: collects RX and DX characters, emits FEC pairs (error correction):
 *      - Both valid & equal → output character, high confidence (quality=2).
 *      - One valid → output that character, medium confidence (quality=1).
 *      - Both invalid but recoverable (Hamming-1) → attempt error correction.
 *      - Unrecoverable → output '*', no confidence (quality=0).
 *   5. Re-sync on bad_pairs ≥ SITORB_BAD_PAIRS_TO_RESYNC (quality=0 accumulates).
 *
 * Slot structure (after lock, same frequency, time-multiplexed):
 *   Slot 0 (mod 5): character 0, RX or DX per phasing code
 *   Slot 1 (mod 5): character 0 repeat (opposite of slot 0)
 *   Slot 2 (mod 5): character 1, same polarity as slot 0
 *   Slot 3 (mod 5): character 1 repeat
 *   Slot 4 (mod 5): character 2, same polarity as slot 0
 *   (pattern repeats, 5-slot FEC window)
 *
 * Error handling:
 *   - Hamming-1 correction: single bit errors corrected via table lookup.
 *   - Quality tracking: maintains bad_pairs counter (incremented on mismatches).
 *   - Drop-sync: if bad_pairs ≥ threshold, resync automatically.
 *
 * State machine (internal):
 *   searching → (phasing detected or fast-sync) → locked → (degraded quality) → searching
 *
 * Main functions:
 *   process_sitorb(bit) - entry point, called per decoded bit.
 *   sitorb_reset()      - reset all state.
 *   Other functions are internal.
 */


/* 
 * NBDP‑Telex‑Terminologie (NBDP = Narrow‑Band Direct Printing) :
 * RX : Receive Copy
 * DX : Duplex Copy / Second Copy
 */

#include <stdint.h>
#include <string.h>
#include "buffer.h"

#ifndef SITORB_PUTC
#define SITORB_PUTC(ch) writebuf((uint8_t)(ch))
#endif

#define SITORB_SYNC_CHARS 4u
#define SITORB_BAD_PAIRS_TO_RESYNC 12u
#define SITORB_FASTSYNC_SCORE_TO_LOCK 8u        // erforderliche Lock-Schwelle
#define SITORB_FASTSYNC_SCORE_MAX 32u           // Maximalwert für die die Wahrscheinlichkeit eines Fast-Locks 

 /* Codes are packed in transmitted order: first received bit is bit 6. */
#define CODE_PHASING_1 0x78u /* BBBBYYY, RX position */
#define CODE_PHASING_2 0x33u /* YBBYYBB, DX position */
#define CODE_IDLE_BETA 0x66u /* BBYYBBY */
#define CODE_LTRS      0x2du /* YBYBBYB */
#define CODE_FIGS      0x36u /* YBBYBBY */
#define CODE_NULL      0x2bu /* YBYBYBB */

typedef struct
{
    uint8_t code;
    char letters;
    char figures;
} ccir476_entry_t;

static const ccir476_entry_t ccir476_table[] =   // ccir476-Zeichentabelle
{
    { 0x71, 'A', '-'  }, { 0x27, 'B', '?'  },
    { 0x5c, 'C', ':'  }, { 0x65, 'D', '?'  },
    { 0x35, 'E', '3'  }, { 0x6c, 'F', '?'  },
    { 0x56, 'G', '?'  }, { 0x4b, 'H', '?'  },
    { 0x59, 'I', '8'  }, { 0x74, 'J', '\a' },
    { 0x3c, 'K', '('  }, { 0x53, 'L', ')'  },
    { 0x4e, 'M', '.'  }, { 0x4d, 'N', ','  },
    { 0x47, 'O', '9'  }, { 0x5a, 'P', '0'  },
    { 0x3a, 'Q', '1'  }, { 0x55, 'R', '4'  },
    { 0x69, 'S', '\'' }, { 0x17, 'T', '5'  },
    { 0x39, 'U', '7'  }, { 0x1e, 'V', '='  },
    { 0x72, 'W', '2'  }, { 0x2e, 'X', '/'  },
    { 0x6a, 'Y', '6'  }, { 0x63, 'Z', '+'  },
    { 0x0f, '\r', '\r' }, { 0x1b, '\n', '\n' },
    { CODE_LTRS, 0, 0 }, { CODE_FIGS, 0, 0 },
    { 0x1d, ' ', ' '  }, { CODE_NULL, 0, 0 }
};

typedef struct
{
    uint8_t active;
    uint8_t count;
    uint8_t last_code;
    uint8_t bits_until_next;
} sync_trial_t;

typedef struct
{
    uint8_t have;
    uint8_t valid;
    uint8_t code;
} pending_char_t;

typedef struct
{
    uint8_t slot;
    uint8_t have;
    uint8_t code[5];
    uint8_t valid[5];
    uint8_t fec_score[2];    // Zwei separate Scores für die beiden Phasenpositionen (DX/RX)
} fast_sync_trial_t;

typedef struct
{
    uint8_t locked;
    uint8_t variant;
    uint8_t raw_shift;
    uint8_t raw_bits_seen;
    uint8_t search_bit_phase;
    uint8_t current_raw;
    uint8_t current_bits;
    uint8_t next_is_dx;
    uint8_t letters_shift;
    uint8_t bad_pairs;
    uint32_t slot;
    sync_trial_t trial[4][7];
    fast_sync_trial_t fast_trial[4][7];
    pending_char_t pending[5];
} sitorb_state_t;

static sitorb_state_t sitorb;

/* ------------------------
   Hilfsfunktionen zur Rettung verlorener Zeichen (Hamming‑1)   <-- coilot
   ------------------------ */

/* Zähle 1-Bits in 7-bit XOR (einfacher popcount) */
static int hamming7(uint8_t a, uint8_t b)
{
    uint8_t x = (uint8_t)((a ^ b) & 0x7Fu);
    int cnt = 0;
    while (x)
    {
        cnt += (x & 1u);
        x >>= 1;
    }
    return cnt;
}

/* Suche einen Tabellen‑Code innerhalb maximaler Hamming‑Distanz.
   Liefert den Tabellen‑code oder -1 wenn keiner gefunden. */
static int find_closest_ccir_code(uint8_t raw, int maxdist)
{
    int best_idx = -1;
    int best_d = 8;
    uint8_t r = raw & 0x7Fu;
    for (unsigned i = 0; i < sizeof(ccir476_table) / sizeof(ccir476_table[0]); ++i)
    {
        uint8_t tc = (uint8_t)(ccir476_table[i].code & 0x7Fu);
        int d = hamming7(r, tc);
        if (d < best_d)
        {
            best_d = d;
            best_idx = (int)i;
            if (best_d == 0) break;
        }
    }
    if (best_idx >= 0 && best_d <= maxdist)
        return (int)(ccir476_table[best_idx].code & 0x7Fu);
    return -1;
}

/* Wenn ein Code als ungültig erkannt wurde, versuche eine Korrektur
   auf einen Tabellen‑Code mit Hamming‑Distanz ≤ 1. */
static uint8_t correct_if_close(uint8_t code, uint8_t* valid)
{
    if (valid && *valid) return code;
    int c = find_closest_ccir_code(code, 1);
    if (c >= 0)
    {
        if (valid) *valid = 1u;
        return (uint8_t)c;
    }
    return code;
}

/* ------------------------
   Ende Hilfsfunktionen
   ------------------------ */


static uint8_t reverse7(uint8_t x)
{
    uint8_t y = 0u;
    uint8_t i;

    x &= 0x7fu;
    for (i = 0u; i < 7u; ++i)
    {
        y = (uint8_t)((y << 1) | (x & 1u));
        x >>= 1;
    }
    return y;
}

static uint8_t transform_code(uint8_t raw, uint8_t variant)
{
    uint8_t code = raw & 0x7fu;

    if (variant & 1u)
    {
        code = reverse7(code);
    }

    if (variant & 2u)
    {
        code ^= 0x7fu;
    }

    return code;
}

static int is_phasing(uint8_t code)
{
    return code == CODE_PHASING_1 || code == CODE_PHASING_2;
}

static int has_four_one_bits(uint8_t x)
{
    x &= 0x7fu;
    x = (uint8_t)(x - ((x >> 1) & 0x55u));
    x = (uint8_t)((x & 0x33u) + ((x >> 2) & 0x33u));
    x = (uint8_t)((x + (x >> 4)) & 0x0fu);
    return x == 4u;
}

static int table_index(uint8_t code)
{
    unsigned i;

    for (i = 0; i < sizeof(ccir476_table) / sizeof(ccir476_table[0]); ++i)
    {
        if (ccir476_table[i].code == code)
        {
            return (int)i;
        }
    }
    return -1;
}

static int is_traffic_code(uint8_t code)
{
    return table_index(code) >= 0;
}

static int is_sitor_code(uint8_t code)
{
    return has_four_one_bits(code);
}

static void clear_pending(void)
{
    memset(sitorb.pending, 0, sizeof(sitorb.pending));
}

static void drop_sync(void)
{
    uint8_t raw_shift = sitorb.raw_shift;
    uint8_t raw_bits_seen = sitorb.raw_bits_seen;
    uint8_t search_bit_phase = sitorb.search_bit_phase;

    memset(&sitorb, 0, sizeof(sitorb));
    sitorb.raw_shift = raw_shift;
    sitorb.raw_bits_seen = raw_bits_seen;
    sitorb.search_bit_phase = search_bit_phase;
    sitorb.letters_shift = 1u;
}

void sitorb_reset(void)
{
    memset(&sitorb, 0, sizeof(sitorb));
    sitorb.letters_shift = 1u;
}

static void emit_code(uint8_t code)
{
    int idx = table_index(code);
    char ch;

    if (idx < 0)
    {
        SITORB_PUTC('*');
        return;
    }

    if (code == CODE_LTRS)
    {
        sitorb.letters_shift = 1u;
        return;
    }

    if (code == CODE_FIGS)
    {
        sitorb.letters_shift = 0u;
        return;
    }

    if (code == CODE_NULL)
    {
        return;
    }

    ch = sitorb.letters_shift ? ccir476_table[idx].letters : ccir476_table[idx].figures;
    if (ch != 0)
    {
        SITORB_PUTC(ch);
    }
}

static uint8_t emit_fec_pair(uint8_t dx_valid, uint8_t dx_code, uint8_t rx_valid, uint8_t rx_code)
{
    /* Versuche Korrektur auf Hamming‑Distanz 1 falls eine Seite ungültig ist. */
    dx_code = correct_if_close(dx_code, &dx_valid);
    rx_code = correct_if_close(rx_code, &rx_valid);

    if (dx_valid && rx_valid)
    {
        if (dx_code == rx_code)
        {
            emit_code(dx_code);
            return 2u;
        }
        else
        {
            /* beide gültig, aber unterschiedlich -> keine sichere Korrektur */
            SITORB_PUTC('*');
            return 0u;
        }
    }
    else if (dx_valid)
    {
        emit_code(dx_code);
        return 1u;
    }
    else if (rx_valid)
    {
        emit_code(rx_code);
        return 1u;
    }
    else
    {
        /* Beide ungültig: probiere, ob einer durch Hamming‑1 gerettet werden kann */
        int cdx = find_closest_ccir_code(dx_code, 1);
        int crx = find_closest_ccir_code(rx_code, 1);
        if (cdx >= 0)
        {
            emit_code((uint8_t)cdx);
            return 1u;
        }
        if (crx >= 0)
        {
            emit_code((uint8_t)crx);
            return 1u;
        }
        SITORB_PUTC('*');
        return 0u;
    }
}

static void note_fec_quality(uint8_t quality)
{
    if (quality == 2u)
    {
        sitorb.bad_pairs = 0u;
        return;
    }

    if (quality == 1u)
    {
        return;
    }

    if (sitorb.bad_pairs < 254u)
    {
        sitorb.bad_pairs = (uint8_t)(sitorb.bad_pairs + 2u);
    }
    else
    {
        sitorb.bad_pairs = 255u;
    }
    if (sitorb.bad_pairs >= SITORB_BAD_PAIRS_TO_RESYNC)
    {
        drop_sync();
    }
}

static void handle_character(uint8_t code)
{
    uint8_t is_dx = sitorb.next_is_dx;
    uint8_t idx = (uint8_t)(sitorb.slot % 5u);
    uint8_t valid = (uint8_t)is_traffic_code(code);

    sitorb.next_is_dx ^= 1u;
    sitorb.slot++;

    if (is_phasing(code))
    {
        clear_pending();
        sitorb.bad_pairs = 0u;
        sitorb.slot = 0u;
        sitorb.next_is_dx = (code == CODE_PHASING_1) ? 1u : 0u;
        return;
    }

    if (code == CODE_IDLE_BETA)
    {
        clear_pending();
        return;
    }

    if (is_dx)
    {
        sitorb.pending[idx].have = 1u;
        sitorb.pending[idx].valid = valid;
        sitorb.pending[idx].code = code;
    }
    else
    {
        if (sitorb.pending[idx].have)
        {
            uint8_t quality = emit_fec_pair(sitorb.pending[idx].valid,
                sitorb.pending[idx].code,
                valid,
                code);
            sitorb.pending[idx].have = 0u;
            note_fec_quality(quality);
        }
        else
        {
            /* Keine pending-DX vorhanden: wenn code gültig ausgeben,
               ansonsten versuchen wir eine Hamming‑1‑Korrektur und
               geben das korrigierte Zeichen aus, falls möglich. */
            if (valid)
            {
                emit_code(code);
            }
            else
            {
                int c = find_closest_ccir_code(code, 1);
                if (c >= 0)
                {
                    emit_code((uint8_t)c);
                }
            }
        }
    }
}

static void acquire_sync(uint8_t variant, uint8_t last_phasing_code)
{
    sitorb.locked = 1u;
    sitorb.variant = variant;
    sitorb.current_raw = 0u;
    sitorb.current_bits = 0u;
    sitorb.slot = 0u;
    sitorb.letters_shift = 1u;
    sitorb.bad_pairs = 0u;
    clear_pending();

    /*
     * ITU-R M.476: phasing signal 2 determines DX, signal 1 determines RX.
     * Therefore the next character slot is the opposite position.
     */
    sitorb.next_is_dx = (last_phasing_code == CODE_PHASING_1) ? 1u : 0u;
}

static void acquire_fast_sync(uint8_t variant)
{
    sitorb.locked = 1u;
    sitorb.variant = variant;
    sitorb.current_raw = 0u;
    sitorb.current_bits = 0u;
    sitorb.slot = 0u;
    sitorb.letters_shift = 1u;
    sitorb.bad_pairs = 0u;
    clear_pending();

    // Reset der Fast‑Sync Scores für alle Varianten und Phasen,
    // damit keine verbliebenen Scores sofort wieder ein Lock auslösen.
    for (int v = 0; v < 4; ++v)   // <-- copilot
    {
        for (int phase = 0; phase < 7; ++phase)
        {
            sitorb.fast_trial[v][phase].fec_score[0] = 0u;
            sitorb.fast_trial[v][phase].fec_score[1] = 0u;
        }
    }

    /*
     * A +5 FEC match means the current character was most probably the RX
     * repeat of an earlier DX character.  The next character slot is DX.
     */
    sitorb.next_is_dx = 1u;
}

static void start_trial(sync_trial_t* trial, uint8_t code)
{
    trial->active = 1u;
    trial->count = 1u;
    trial->last_code = code;
    trial->bits_until_next = 1u;
}

static void score_up(uint8_t* score, uint8_t amount)        // Wahrscheinlichkeit für 5-Slot-Vergleich erhöhen
{
    if (*score <= (uint8_t)(SITORB_FASTSYNC_SCORE_MAX - amount))
    {
        *score = (uint8_t)(*score + amount);
    }
    else
    {
        *score = SITORB_FASTSYNC_SCORE_MAX;
    }
}

static void score_down(uint8_t* score)          // Wahrscheinlichkeit von 5-Slot-Vergleich reduzieren
{
    if (*score > 0u)
    {
        (*score)--;
    }
}

static int update_fast_sync_trial(fast_sync_trial_t* trial, uint8_t code)
{
    uint8_t valid = (uint8_t)is_sitor_code(code);
    uint8_t idx = (uint8_t)(trial->slot % 5u);
    uint8_t parity = (uint8_t)(trial->slot & 1u);
    uint8_t lock = 0u;

    if (trial->have >= 5u)
    {
        if (valid && trial->valid[idx] && trial->code[idx] == code)
        {
            score_up(&trial->fec_score[parity], 2u);
            score_down(&trial->fec_score[parity ^ 1u]);
        }
        else
        {
            score_down(&trial->fec_score[parity]);
            if (!valid || !trial->valid[idx])
            {
                score_down(&trial->fec_score[parity ^ 1u]);
            }
        }

        if (trial->fec_score[parity] >= SITORB_FASTSYNC_SCORE_TO_LOCK)
        {
            lock = 1u;
        }
    }

    trial->code[idx] = code;
    trial->valid[idx] = valid;
    if (trial->have < 5u)
    {
        trial->have++;
    }
    trial->slot++;

    return lock;
}

static void search_sync(uint8_t raw_window, uint8_t phase)
{
    uint8_t variant;

    for (variant = 0u; variant < 4u; ++variant)
    {
        uint8_t code = transform_code(raw_window, variant);
        sync_trial_t* trial = &sitorb.trial[variant][phase];

        if (!trial->active)
        {
            if (is_phasing(code))
            {
                start_trial(trial, code);
            }
        }
        else
        {
            if (--trial->bits_until_next == 0u)
            {
                if (is_phasing(code) && code != trial->last_code)
                {
                    trial->count++;
                    trial->last_code = code;
                    trial->bits_until_next = 1u;

                    if (trial->count >= SITORB_SYNC_CHARS)
                    {
                        acquire_sync(variant, code);
                        return;
                    }
                }
                else if (is_phasing(code))
                {
                    start_trial(trial, code);
                }
                else
                {
                    memset(trial, 0, sizeof(*trial));
                }
            }
        }

        if (update_fast_sync_trial(&sitorb.fast_trial[variant][phase], code))
        {
            acquire_fast_sync(variant);
            return;
        }
    }
}

void process_sitorb(uint8_t bit)
{
    uint8_t code;

    bit &= 1u;

    sitorb.raw_shift = (uint8_t)(((sitorb.raw_shift << 1) | bit) & 0x7fu);

    if (sitorb.raw_bits_seen < 7u)
    {
        sitorb.raw_bits_seen++;
    }

    sitorb.search_bit_phase++;

    if (sitorb.search_bit_phase >= 7u)
    {
        sitorb.search_bit_phase = 0u;
    }

    if (!sitorb.locked)
    {
        if (sitorb.raw_bits_seen >= 7u)
        {
            search_sync(sitorb.raw_shift, sitorb.search_bit_phase);
        }
        return;
    }

    sitorb.current_raw = (uint8_t)(((sitorb.current_raw << 1) | bit) & 0x7fu);

    if (++sitorb.current_bits < 7u)
    {
        return;
    }

    code = transform_code(sitorb.current_raw, sitorb.variant);
    sitorb.current_bits = 0u;
    sitorb.current_raw = 0u;
    handle_character(code);
}


/*
*   by dl8mcg Jan. 2025
*/

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "buffer.h"

// baudot - tables
const char letters_table[32] =
{
    '\0', 'E', '\n', 'A', ' ', 'S', 'I', 'U',
    '\r', 'D', 'R', 'J', 'N', 'F', 'C', 'K',
    'T', 'Z', 'L', 'W', 'H', 'Y', 'P', 'Q',
    'O', 'B', 'G', ' ', 'M', 'X', 'V', '\0'
};

const char figures_table[32] =
{
    '\0', '3', '\n', '-', ' ', '\'', '8', '7',
    '\r', '$', '4', '\'', ',', '!', ':', '(',
    '5', '+', ')', '2', '#', '6', '0', '1',
    '9', '?', '&', ' ', '.', '/', '=', '\0'
};

enum RTTY_MODE { LETTERS, FIGURES };

// some variables
static enum RTTY_MODE current_mode = LETTERS;

static uint8_t rxbit;
static uint8_t rxbyte = 0;                      // shift-in register
static uint8_t bit_count = 0;
static uint8_t bit_buffer = 0;

// Funktionszeiger für Zustandsmaschine
static void state1();
static void state2();
static void state3();
static void state4();

static void (*smRtty)() = state1;              // Initialzustand

const char* table = letters_table;

void process_rtty(uint8_t bit)
{
    rxbyte = (rxbyte << 1) | bit;               // shift in new bit to LSB
    rxbit = bit;                                // Eingehendes Bit speichern
    smRtty();
}

static void state1()                            // Startbit-Suche
{
    if ((rxbyte & 0b111) == 0b110)                // Stopbit, Startbit erkannt
    {
        bit_count = 0;
        bit_buffer = 0;
        smRtty = state2;
    }
}

static void state2()                            // Datenerfassung
{
    bit_buffer = (bit_buffer >> 1) | (rxbit << 4);  // Bits sammeln (LSB zuerst)
    bit_count++;

    if (bit_count == 5)                         // Alle Datenbits gesammelt
    {
        smRtty = state3;
    }
}

static void state3()                            // Prüfung des ersten Stopp-Bits
{
    if (rxbit == 1)
    {
        smRtty = state4;
        return;
    }
    smRtty = state1;
}


static void state4()                    // Prüfung des zweiten Stopp-Bits
{
    if (rxbit == 1)                     // Zweites Stopp-Bit korrekt
    {
        if ((bit_buffer & 0x1F) == 0b11111)
        {
            table = letters_table;
            smRtty = state1;
            return;
        }

        if ((bit_buffer & 0x1F) == 0b11011)
        {
            table = figures_table;
            smRtty = state1;
            return;
        }

        if( (bit_buffer & 0x1F) == 0b00100)  // space
            table = letters_table;

        writebuf(table[bit_buffer & 0x1F]);
    }
    smRtty = state1;                   // Zurücksetzen für nächstes Zeichen
}

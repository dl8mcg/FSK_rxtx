/*
*   by dl8mcg Jan. 2025 .. März 2026       2FSK - ASCII - Decoder
*/

#include <stdint.h>
#include <stdio.h>
#include "config.h"
#include "buffer.h"

static uint8_t rxbit;
static uint8_t rxbyte = 0;              // shift-in register
static uint8_t bit_count = 0;
static uint8_t bit_buffer = 0;

// Funktionszeiger für Zustandsmaschine
static void state1();
static void state2();
static void state3();
static void state4();

static void (*smAscii)() = state1;      // Initialzustand

void process_ascii(uint8_t bit)
{
    rxbyte = (rxbyte << 1) | bit;       // shift in new bit to LSB
    rxbit = bit;                        // eingehendes Bit speichern
    smAscii();
}

static void state1()                    // Startbit-Suche
{
    if ((rxbyte & 0b111) == 0b110)      // Stopbit, Stopbit, Startbit erkannt
    {
        bit_count = 0;
        bit_buffer = 0;
        smAscii = state2;
    }
}

static void state2()                    // Datenerfassung
{
    bit_buffer = (bit_buffer >> 1) | (rxbit << 7);  // Bits sammeln (LSB zuerst)
    bit_count++;

    if (bit_count == 8)                 // Alle Datenbits gesammelt
    {
        smAscii = state3;
    }
}

static void state3()                    // Prüfung des ersten Stopp-Bits
{
    if (rxbit == 1)                     // Erstes Stopp-Bit korrekt
    {
        smAscii = state4;
    }
    else                                // Fehler, zurücksetzen
    {
        smAscii = state1;
    }
}

static void state4()                    // Prüfung des zweiten Stopp-Bits
{
    if (rxbit == 1)                     // Zweites Stopp-Bit korrekt
    {
        writebuf(bit_buffer);
    }
    smAscii = state1;                   // Zurücksetzen für nächstes Zeichen
}


/*
*   by dl8mcg Jan. 2025 to June 2026       2FSK - RTTY - Decoder
*/

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "buffer.h"

// baudot - tables
static const char letters_table[32] =
{
    '\0', 'E', '\n', 'A', ' ', 'S', 'I', 'U',
    '\r', 'D', 'R', 'J', 'N', 'F', 'C', 'K',
    'T', 'Z', 'L', 'W', 'H', 'Y', 'P', 'Q',
    'O', 'B', 'G', ' ', 'M', 'X', 'V', '\0'
};


static const char figures_table[32] =
{
    '\0', '3', '\n', '-', ' ', '\'', '8', '7',
    '\r', '$', '4', '\'', ',', '!', ':', '(',
    '5', '+', ')', '2', '#', '6', '0', '1',
    '9', '?', '&', ' ', '.', '/', '=', '\0'
};

// Dekodierungsvariablen 
static uint8_t half_count = 0;
static uint8_t first_half = 0;
static uint8_t rxbit = 0;
static uint8_t rxbyte = 0;                      // shift-in register
static uint8_t bit_count = 0;
static uint8_t bit_buffer = 0;
static const char* table = letters_table;       // Zeige auf die aktuelle Tabelle (LETTERS oder FIGURES)
static bool uos = false;    // Unshift On Space - Flag, um nach einem space-Zeichen automatisch in LETTERS-Modus zu wechseln

// Funktionszeiger für Zustandsmaschine
static void state1();
static void state2();
static void state3();
static void (*smRtty)() = state1;              // Initialzustand

void process_rtty(uint8_t bit)
{
    rxbyte = (rxbyte << 1) | bit;               // shift in new bit to LSB
    rxbit = bit;                                // Eingehendes Bit speichern
	uos = false; 
    smRtty();
}

void process_rtty_uos(uint8_t bit)
{
    rxbyte = (rxbyte << 1) | bit;               // shift in new bit to LSB
    rxbit = bit;                                // Eingehendes Bit speichern
    uos = true;
    smRtty();
}

static void state1(void)
{
    if ((rxbyte & 0x1F) == 0b11100)
    {
        bit_count = 0;
        half_count = 0;
        bit_buffer = 0;

        smRtty = state2;
    }
}

static void state2(void)
{
	if (half_count == 0)        // erstes Halbbit empfangen
    {
        first_half = rxbit;
        half_count = 1;
        return;
    }

    half_count = 0;

    if (first_half != rxbit)    // beide Halbbits müssen gleich sein
    {
        smRtty = state1;        // z.B. 10 oder 01 -> Fehler
        return;
    }

	bit_buffer >>= 1;           // shift buffer to the right

    if (rxbit)
		bit_buffer |= 0x10;     // set MSB if bit is 1

	bit_count++;                // Bitzähler erhöhen

	if (bit_count == 5)         // 5 Bits empfangen, jetzt prüfen auf Stopbits
    {
        smRtty = state3;
    }
}

static void state3(void)
{
    static uint8_t stop_count = 0;

    if (rxbit == 1)
    {
		stop_count++;     
		if (stop_count < 3)     // 3 Halb-Stopbits erwartet, weiter warten
            return;
    }
    else
    {
		stop_count = 0;         // Fehler: Stopbit muss 1 sein, wenn nicht, zurück zum Anfang
        smRtty = state1;
        return;
    }

    stop_count = 0;

	if ((bit_buffer & 0x1F) == 0b11111)  // Shift to Letters
    {
        table = letters_table;
        smRtty = state1;
        return;
    }

	if ((bit_buffer & 0x1F) == 0b11011)  // Shift to Figures
    {
        table = figures_table;
        smRtty = state1;
        return;
    }

	if (uos && ((bit_buffer & 0x1F) == 0b00100))  // UOS - Shift to Letters on Space
    {
        table = letters_table;
    }

	writebuf(table[bit_buffer & 0x1F]);     // Ausgabe des dekodierten Zeichens

    smRtty = state1;
}


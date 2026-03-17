/*
*   by dl8mcg Jan. 2025 .. März 2026       2FSK - RTTY - Decoder
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

static void (*smRtty)() = state1;              // Initialzustand

const char* table = letters_table;

static bool uos = false; // Unshift On Space - Flag, um nach einem space-Zeichen automatisch in LETTERS-Modus zu wechseln

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

static void state3()                            // Prüfung des (einzigen) Stopp-Bits und Ausgabe
{
    if (rxbit == 1) // Stopp-Bit korrekt -> Zeichen akzeptieren
    {
        // Sonderfälle für Figures/Letters und unshift on space
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

		if (uos && (bit_buffer & 0x1F) == 0b00100)  // Unshift On Space 
        {
            table = letters_table;
        }

        writebuf(table[bit_buffer & 0x1F]);
    }
    // Egal ob Stopbit korrekt oder nicht: zurück in Startbit‑Suche
    smRtty = state1;
}











///*
//*   by dl8mcg Jan. 2025 .. Feb. 2026       2FSK - RTTY - Decoder
//*/
//
//#include <stdint.h>
//#include <stdbool.h>
//#include "config.h"
//#include "buffer.h"
//
//// baudot - tables
//const char letters_table[32] =
//{
//    '\0', 'E', '\n', 'A', ' ', 'S', 'I', 'U',
//    '\r', 'D', 'R', 'J', 'N', 'F', 'C', 'K',
//    'T', 'Z', 'L', 'W', 'H', 'Y', 'P', 'Q',
//    'O', 'B', 'G', ' ', 'M', 'X', 'V', '\0'
//};
//
//const char figures_table[32] =
//{
//    '\0', '3', '\n', '-', ' ', '\'', '8', '7',
//    '\r', '$', '4', '\'', ',', '!', ':', '(',
//    '5', '+', ')', '2', '#', '6', '0', '1',
//    '9', '?', '&', ' ', '.', '/', '=', '\0'
//};
//
//enum RTTY_MODE { LETTERS, FIGURES };
//
//// some variables
//static enum RTTY_MODE current_mode = LETTERS;
//
//static uint8_t rxbit;
//static uint8_t rxbyte = 0;                      // shift-in register
//static uint8_t bit_count = 0;
//static uint8_t bit_buffer = 0;
//
//// Funktionszeiger für Zustandsmaschine
//static void state1();
//static void state2();
//static void state3();
//static void state4();
//
//static void (*smRtty)() = state1;              // Initialzustand
//
//const char* table = letters_table;
//
//void process_rtty(uint8_t bit)
//{
//    rxbyte = (rxbyte << 1) | bit;               // shift in new bit to LSB
//    rxbit = bit;                                // Eingehendes Bit speichern
//    smRtty();
//}
//
//static void state1()                            // Startbit-Suche
//{
//    if ((rxbyte & 0b111) == 0b110)                // Stopbit, Startbit erkannt
//    {
//        bit_count = 0;
//        bit_buffer = 0;
//        smRtty = state2;
//    }
//}
//
//static void state2()                            // Datenerfassung
//{
//    bit_buffer = (bit_buffer >> 1) | (rxbit << 4);  // Bits sammeln (LSB zuerst)
//    bit_count++;
//
//    if (bit_count == 5)                         // Alle Datenbits gesammelt
//    {
//        smRtty = state3;
//    }
//}
//
//static void state3()                            // Prüfung des ersten Stopp-Bits
//{
//    if (rxbit == 1)
//    {
//        smRtty = state4;
//        return;
//    }
//    smRtty = state1;
//}
//
//
//static void state4()                    // Prüfung des zweiten Stopp-Bits
//{
//    if (rxbit == 1)                     // Zweites Stopp-Bit korrekt
//    {
//        if ((bit_buffer & 0x1F) == 0b11111)
//        {
//            table = letters_table;
//            smRtty = state1;
//            return;
//        }
//
//        if ((bit_buffer & 0x1F) == 0b11011)
//        {
//            table = figures_table;
//            smRtty = state1;
//            return;
//        }
//
//        if( (bit_buffer & 0x1F) == 0b00100)  // space
//            table = letters_table;
//
//        writebuf(table[bit_buffer & 0x1F]);
//    }
//    smRtty = state1;                   // Zurücksetzen für nächstes Zeichen
//}


///*
//*   by dl8mcg Jan. 2025 .. Feb. 2026       2FSK - RTTY - Decoder
//*/
//
//#include <stdint.h>
//#include <stdbool.h>
//#include <stdio.h>
//#include "config.h"
//#include "buffer.h"
//
//// baudot - tables
//const char letters_table[32] =
//{
//    '\0', 'E', '\n', 'A', ' ', 'S', 'I', 'U',
//    '\r', 'D', 'R', 'J', 'N', 'F', 'C', 'K',
//    'T', 'Z', 'L', 'W', 'H', 'Y', 'P', 'Q',
//    'O', 'B', 'G', ' ', 'M', 'X', 'V', '\0'
//};
//
//const char figures_table[32] =
//{
//    '\0', '3', '\n', '-', ' ', '\'', '8', '7',
//    '\r', '$', '4', '\'', ',', '!', ':', '(',
//    '5', '+', ')', '2', '#', '6', '0', '1',
//    '9', '?', '&', ' ', '.', '/', '=', '\0'
//};
//
//enum RTTY_MODE { LETTERS, FIGURES };
//
//// some variables
//static enum RTTY_MODE current_mode = LETTERS;
//
//static uint8_t rxbit;
//static uint8_t rxbyte = 0;                      // shift-in register (recent bits, newest at LSB)
//static uint8_t bit_count = 0;
//static uint8_t bit_buffer = 0;
//
//// Funktionszeiger für Zustandsmaschine
//static void state1();
//static void state2();
//static void state3();
//static void state4(); // zweites Stopp‑Bit prüfen
//
//static void (*smRtty)() = state1;              // Initialzustand
//
//const char* table = letters_table;
//
//void process_rtty(uint8_t bit)
//{
//    // rxbyte speichert die letzten empfangenen Bits, newest LSB:
//    rxbyte = (rxbyte << 1) | (bit & 1);
//    rxbit = bit & 1;                                // Eingehendes Bit speichern
//    smRtty();
//}
//
//static void state1()                            // Startbit‑Suche (Idle=1 -> Start=0)
//{
//    // Wir erwarten: ... 1 (idle/stop), 1 (idle/stop), 0 (start)  => LSB newest -> pattern b2b1b0 == 110
//    if ((rxbyte & 0b111) == 0b110)
//    {
//        bit_count = 0;
//        bit_buffer = 0;
//        smRtty = state2;
//    }
//}
//
//static void state2()                            // Datenerfassung (5 LSB-first bits)
//{
//    // Wir sammeln LSB-first in bit_buffer; nach 5 Bits gehen wir zu Stopbit‑Prüfung
//    // Shift right and insert new bit at bit4 keeps LSB-first order when later indexing [0..4]
//    bit_buffer = (bit_buffer >> 1) | (rxbit << 4);
//    bit_count++;
//    if (bit_count == 5)
//    {
//        smRtty = state3; // prüfe erstes Stopbit
//    }
//}
//
//static void state3()                            // Prüfung des ersten Stopp‑Bits
//{
//    if (rxbit == 1) // erstes Stop‑Bit korrekt -> prüfe optional zweites Stop‑Bit
//    {
//        smRtty = state4;
//        return;
//    }
//
//    // erstes Stopbit falsch -> zurück in Startsuche
//    smRtty = state1;
//}
//
//static void state4()                    // Prüfung des zweiten Stopp‑Bits und Ausgabe
//{
//    if (rxbit == 1)                     // Zweites Stopp-Bit korrekt
//    {
//        // Sonderfälle für Figures/Letters
//        if ((bit_buffer & 0x1F) == 0b11111)
//        {
//            table = letters_table;
//            smRtty = state1;
//            return;
//        }
//
//        if ((bit_buffer & 0x1F) == 0b11011)
//        {
//            table = figures_table;
//            smRtty = state1;
//            return;
//        }
//
//        if ((bit_buffer & 0x1F) == 0b00100)  // space
//            table = letters_table;
//
//        writebuf(table[bit_buffer & 0x1F]);
//    }
//    // Egal ob zweites Stopbit korrekt oder nicht: zurück in Startbit‑Suche
//    smRtty = state1;
//}
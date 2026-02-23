/*
*   by dl8mcg Jan. 2025 ... Feb 2026  2FSK AX25 - Decoder 
*/

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "fsk_demod.h"
#include "config.h"
#include "buffer.h"
#include <windows.h>

// state machine frame
static void stateFrame00();
static void stateFrame01();
static void stateFrame02();
static void stateFrame1();
static void stateFrame2();
static void (*smFrame)() = stateFrame00;

// state machine content
static void stateContent0();
static void stateContent1();
static void stateContent2();
static void stateContent3();
static void (*smContent)() = stateContent0;

static uint8_t rxbyte = 0;          // Schieberegister für destuffed Datenbits
static uint8_t raw_shift_reg = 0;   // Schieberegister für Flag-Erkennung (vor Destuffing!)
static uint8_t oldbit = 0;
static uint8_t bitcnt = 0;
static uint8_t onecnt = 0;
static bool    rxbit = false;

static uint16_t current_crc = 0xFFFF;

static inline uint16_t update_crc(uint16_t crc, uint8_t data) 
{
    const uint16_t POLY = 0x1021;
    uint16_t c = crc;
    uint8_t d = data;

    // Verarbeite LSB zuerst — entspricht Ursprungscode, nur effizienter geschrieben.
    for (int i = 0; i < 8; ++i) {
        uint16_t mix = ((c >> 15) & 1) ^ (d & 1);
        c <<= 1;
        d >>= 1;
        if (mix) c ^= POLY;
    }
    return c;
}

void process_ax25(uint8_t bit)
{
    // 1. NRZI-Dekodierung: kein Wechsel = 1, Wechsel = 0
    rxbit = (bit == oldbit) ? 1 : 0;
    oldbit = bit;

    // 2. Raw-Schieberegister IMMER befüllen (vor Destuffing) → Flag-Erkennung
    raw_shift_reg = (raw_shift_reg >> 1) | (rxbit << 7);

    // 3. Bit-Destuffing: Nach fünf Einsen folgende Null ist gestopft → verwerfen
    if ((onecnt == 5) && (rxbit == 0))
    {
        onecnt = 0;
        return;             // Gestopftes Bit: NICHT in rxbyte schieben, NICHT zählen
    }

    // 4. Datenschieberegister befüllen (nur echte, destuffed Bits)
    rxbyte = (rxbyte >> 1) | (rxbit << 7);

    // 5. Eins-Zähler aktualisieren
    if (rxbit == 1)
        onecnt++;
    else
        onecnt = 0;

    // 6. Abort-Sequenz: mehr als 7 aufeinanderfolgende Einsen → Frame verwerfen
    if (onecnt > 7)
    {
        onecnt = 0;
        smFrame = stateFrame00;
        return;
    }

    // 7. Frame-State-Machine aufrufen
    smFrame();
}


// ---------------------------------------------------------------------------
// Frame-State-Machine
// ---------------------------------------------------------------------------
void stateFrame00()          // Warte auf das erste Flag (0x7E)
{
    if (raw_shift_reg == 0x7E)
    {
        bitcnt = 0;
        smFrame = stateFrame01;
    }
}

void stateFrame01()          // Warte auf das zweite Flag (0x7E), direkt anschließend
{
    bitcnt++;

    if (bitcnt == 8)
    {
		if (raw_shift_reg == 0x7E)
        {
            bitcnt = 0;
            smFrame = stateFrame02;
        }
        else
        {
            bitcnt = 0;
            smFrame = stateFrame00;   // Kein Flag → zurücksetzen
        }
    }
}

void stateFrame02()          // Warte auf das dritte Flag (0x7E), direkt anschließend
{
    bitcnt++;

    if (bitcnt == 8)
    {
        if (raw_shift_reg == 0x7E)
        {
            bitcnt = 0;
            smFrame = stateFrame1;
        }
        else
        {
            bitcnt = 0;
            smFrame = stateFrame00;   // Kein Flag → zurücksetzen
        }
    }
}

void stateFrame1()          // Flag empfangen – warte auf erstes Nutzdaten-Byte
{
    // Weiteres Flag empfangen → Zähler neu starten, in stateFrame1 bleiben
    if (raw_shift_reg == 0x7E)
    {
        bitcnt = 0;
        return;
    }

    bitcnt++;
    if (bitcnt < 8)         // Noch kein vollständiges Byte
        return;

    current_crc = 0xFFFF;
    // Erstes vollständiges Datenbyte nach dem Flag
    bitcnt = 0;
    smContent = stateContent0;
    smContent();            // Byte verarbeiten
    smFrame = stateFrame2;
}

void stateFrame2()          // Nutzdaten empfangen
{
    // Flag erkannt → End-Flag (bzw. nächster Frame beginnt)
    if (raw_shift_reg == 0x7E)
    {
        if (current_crc == 0x1D0F)
            printf("\n[CRC OK] (CRC: 0x%04X)\n", current_crc);
        else
            printf("\n[CRC ERROR] (CRC: 0x%04X)\n", current_crc);

        bitcnt = 0;
        smFrame = stateFrame1;   // Zurück: nächsten Frame erwarten
        return;
    }

    bitcnt++;
    if (bitcnt == 8)        // Vollständiges Datenbyte wurde empfangen
    {
        bitcnt = 0;
        smContent();
    }
}

// ---------------------------------------------------------------------------
// Content-State-Machine  (Adressen → Control → PID → Info)
// ---------------------------------------------------------------------------

void stateContent0()        // Adressfelder: Zeichen sind um 1 Bit nach links geshiftet
{
    current_crc = update_crc(current_crc, rxbyte);
    writebuf(rxbyte >> 1);

    if (rxbyte & 0x01)      // LSB gesetzt → letztes Adressbyte
        smContent = stateContent1;
}

void stateContent1()        // Control-Byte
{
    current_crc = update_crc(current_crc, rxbyte);
    if (!(rxbyte & 0x01))
    {
        smContent = stateContent2;  // I-Frame → PID folgt
        return;
    }
    if ((rxbyte & 0x03) == 0x01)
    {
        smContent = stateContent3;  // S-Frame → direkt Info/Ende
        return;
    }
    smContent = stateContent3;      // U-Frame
}

void stateContent2()        // PID-Byte (nur bei I-Frames)
{
    current_crc = update_crc(current_crc, rxbyte);
    smContent = stateContent3;
}

void stateContent3()        // Info-Feld: Nutzdaten ausgeben
{
    current_crc = update_crc(current_crc, rxbyte);
    writebuf(rxbyte);
}



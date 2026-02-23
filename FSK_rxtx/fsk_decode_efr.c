/*
*   by dl8mcg Jan. 2025       EFR - decode
*/

#include <stdint.h>
#include <stdio.h>
#include <windows.h>
#include "config.h"
#include "buffer.h"

static uint8_t rxbit;
static uint8_t rxbyte = 0;              // shift-in register
static uint8_t bit_count = 0;
static uint8_t bit_buffer = 0;

static uint8_t lenuserdata = 0;

static uint8_t cntdata = 0;


static uint8_t parity = 0;

static uint8_t checksum = 0;

volatile static bool okflag = false;

static uint8_t databuf[7];
static uint8_t cntbuf = 0;

char buffer[1000];

// Funktionszeiger für Zustandsmaschine Bytedetektion
static void state1();
static void state2();
static void state3();
static void state4();
static void (*smEfr)() = state1;        // Initialzustand


// Funktionszeiger für Zustandsmaschine Protokoll
static void stateprot1(uint8_t resbyte);
static void stateprot2(uint8_t resbyte);
static void stateprot3(uint8_t resbyte);
static void stateprot4(uint8_t resbyte);
static void stateprot5(uint8_t resbyte);
static void stateprot6(uint8_t resbyte);
static void stateprot7(uint8_t resbyte);
static void stateprot8(uint8_t resbyte);
static void stateprot9(uint8_t resbyte);
static void stateprot10(uint8_t resbyte);
static void (*smEfrprot)(uint8_t resbyte) = stateprot1;        // Initialzustand


void process_efr(uint8_t bit)
{
    rxbyte = (rxbyte << 1) | bit;       // shift in new bit to LSB
    rxbit = bit;                        // eingehendes Bit speichern
    smEfr();
}

static void state1()                    // Startbit-Suche
{
    if ((rxbyte & 0b11) == 0b10)        // Stopbit, Startbit erkannt
    {
        bit_count = 0;
        bit_buffer = 0;
        parity = 0;
        smEfr = state2;
    }
}

static void state2()                    // Datenerfassung
{
    parity += rxbit;
    bit_buffer = (bit_buffer >> 1) | (rxbit << 7);  // Bits sammeln (LSB zuerst)
    bit_count++;

    if (bit_count == 8)                 // Alle Datenbits gesammelt
    {
        smEfr = state3;
    }
}

static void state3()                    // Parity
{
    parity += rxbit;
    if ((parity & 0x01) == 0)
    {
        smEfr = state4;
        return;
    }
    smEfr = state1;                     // parity wrong
}

static void state4()                    // Prüfung des zweiten Stopp-Bits
{
    if (rxbit == 1)                     // Zweites Stopp-Bit korrekt
    {
        //writebuf(bit_buffer);
        //wprintf(L"%02X ", bit_buffer);
        smEfrprot(bit_buffer);
    }
    smEfr = state1;                     // Zurücksetzen für nächstes Zeichen
}



static void stateprot1(uint8_t resbyte)
{
    if (resbyte == 0x68)
    {
        //wprintf(L"start : %02X \n", resbyte);
        /*sprintf_s(buffer, sizeof(buffer), "start : %02X \n", resbyte);
        int size = strlen(buffer);
        for (int i = 0; i < size; i++)
        {
            writebuf(buffer[i]);
        }*/


        writeToRingBufferFormatted("start : %02X \n", resbyte);

        checksum = 0;
        okflag = false;
        smEfrprot = stateprot2;
    }
}

static void stateprot2(uint8_t resbyte)
{
    //wprintf(L"len   : %02X \n", resbyte);
    writeToRingBufferFormatted("len   : %02X \n", resbyte);
    lenuserdata = resbyte;
    smEfrprot = stateprot3;
}

static void stateprot3(uint8_t resbyte)
{
    //wprintf(L"len   : %02X ", resbyte);
    writeToRingBufferFormatted("len   : %02X ", resbyte);
    if (lenuserdata == resbyte)
    {
        //wprintf(L"\n");
        writeToRingBufferFormatted("\n");
        smEfrprot = stateprot4;
        return;
    }
    //wprintf(L"error\n\n");
    writeToRingBufferFormatted("error\n\n");
    smEfrprot = stateprot1;                 // Fehler, zurück auf 1
}

static void stateprot4(uint8_t resbyte)
{
    //wprintf(L"start : %02X \n", resbyte);
    writeToRingBufferFormatted("start : %02X \n", resbyte);
    smEfrprot = stateprot5;
}

static void stateprot5(uint8_t resbyte)
{
    checksum += resbyte;
    //wprintf(L"C     : %02X \n", resbyte);
    writeToRingBufferFormatted("C     : %02X \n", resbyte);
    smEfrprot = stateprot6;
}

static void stateprot6(uint8_t resbyte)
{
    checksum += resbyte;
    //wprintf(L"A     : %02X \n", resbyte);
    writeToRingBufferFormatted("A     : %02X \n", resbyte);
    smEfrprot = stateprot7;
}

static void stateprot7(uint8_t resbyte)
{
    checksum += resbyte;
    //wprintf(L"CI    : %02X \n", resbyte);
    writeToRingBufferFormatted("CI    : %02X \n", resbyte);
    if (lenuserdata > 3)
    {
        //wprintf(L"data  : ");
        writeToRingBufferFormatted("data  : ");
        cntdata = lenuserdata - 3;
        cntbuf = 0;
        smEfrprot = stateprot8;
    }
    else
        smEfrprot = stateprot9;                 // <-- kommt nicht vor ?
}

static void stateprot8(uint8_t resbyte)         // Data-Field
{
    checksum += resbyte;
    //wprintf(L"%02X ", resbyte);
    writeToRingBufferFormatted("%02X ", resbyte);
    if(cntbuf < 7)
        databuf[cntbuf++] = resbyte;
    cntdata--;
    if (cntdata == 0)
    {
        //wprintf(L"\n");
        writeToRingBufferFormatted("\n");
        smEfrprot = stateprot9;
    }
}

static void stateprot9(uint8_t resbyte)         // Checksum
{
    //wprintf(L"cs    : %02X  ", resbyte);
    writeToRingBufferFormatted("cs    : %02X  ", resbyte);
    if (checksum == resbyte)
    {
        //wprintf(L"ok\n");
        writeToRingBufferFormatted("ok\n");
        okflag = true;
    }
    else
        //wprintf(L"error\n");
        writeToRingBufferFormatted("error\n");

    smEfrprot = stateprot10;
}

static void stateprot10(uint8_t resbyte)
{
    if (resbyte == 0x16)
    {
        //wprintf(L"stop  : %02X \n\n", resbyte);
        writeToRingBufferFormatted("stop  : %02X \n\n", resbyte);
        if ((okflag == true) && (lenuserdata == 0x0a))
        {
            //wprintf(L"date : %02d:%02d:%02d    time : %02d:%02d:%02d \n\n", databuf[4]&0x1F, databuf[5], databuf[6], databuf[3], databuf[2], databuf[1] / 4);
            writeToRingBufferFormatted("date : %02d:%02d:%02d    time : %02d:%02d:%02d \n\n", databuf[4] & 0x1F, databuf[5], databuf[6], databuf[3], databuf[2], databuf[1] / 4);
        }
    }
    else
    {
        //wprintf(L"error\n\n");
        writeToRingBufferFormatted("error\n\n");
    }
    smEfrprot = stateprot1;
}

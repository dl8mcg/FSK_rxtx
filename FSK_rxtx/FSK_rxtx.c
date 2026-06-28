/*
*   by dl8mcg Jan. 2025 to June 2026       Hauptprogramm
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>
#include <conio.h>      
#include <stdint.h>
#include "config.h"
//#include "fsk_demod_float.h"
#include "fsk_demod_int.h"
#include "fsk_decode_rtty.h"
#include "sampleprocessing.h"
#include "fsk_decode_ascii.h"
#include "fsk_decode_ax25.h"
#include "buffer.h"

#define KEY_PGUP   73
#define KEY_PGDN   81

uint8_t modus = 1;

int read_key() 
{
    if (_kbhit())
        return _getch();
    return -1;
}

void setmodus(modus)
{
    // Modus setzen
    switch (modus)
    {
    case 1: init_fsk_demod_int(FSK_RTTY_45_BAUD_170Hz); break;
    case 2: init_fsk_demod_int(FSK_RTTY_50_BAUD_85Hz); break;
    case 3: init_fsk_demod_int(FSK_RTTY_50_BAUD_450Hz); break;
    case 4: init_fsk_demod_int(FSK_EFR_200_BAUD_340Hz); break;
    case 5: init_fsk_demod_int(FSK_ASCII_300_BAUD_850Hz); break;
    case 6: init_fsk_demod_int(FSK_AX25_1200_BAUD_1000Hz); break;
    case 7: init_fsk_demod_int(FSK_AX25_9600_BAUD); break;
	case 8: init_fsk_demod_int(FSK_SITORB_100_BAUD); break;
    }
}

int main()
{
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    printf("                           RYTLTYTL                                          by dl8mcg 2026\n\n");
    printf("            Mit den Page-Up- und Pade-Down-Tasten den Modus auszuwählen \n\n");
    initialize_audiostream();
    init_fsk_demod_int(FSK_RTTY_45_BAUD_170Hz);

    while (1)
    {
		int didWork = 0;            // Flag, um zu erkennen, ob etwas zu tun war (Tastendruck oder empfangenes Zeichen)
        int key = read_key();

        if (key > 0)
        {
            didWork = 1;
            int keynext = _getch();

            switch (keynext)
            {
                case KEY_PGUP:
                    modus = (modus % 8) + 1;
                    setmodus(modus);
                    break;

                case KEY_PGDN:
                    modus = (modus == 1) ? 8 : modus - 1;
                    setmodus(modus);
                    break;
            }
        }

        unsigned char value;
        if (readbuf(&value))
        {
			didWork = 1;                        // Flag, um zu erkennen, dass etwas zu tun war (empfangenes Zeichen)
            if (value < 0x80)
            {
                putchar(value);                  // unverändert ausgeben, da es sich um ein ASCII-Zeichen handelt (1-Byte UTF‑8-Sequenz)
            }
            else
            {
                putchar(0xC0 | (value >> 6));    // erstes Byte der UTF‑8-Sequenz: 110xxxxx, wobei x die oberen 2 Bits von value sind
                putchar(0x80 | (value & 0x3F));  // zweites Byte der UTF‑8-Sequenz: 10xxxxxx, wobei x die unteren 6 Bits von value sind
            }
        }

        if (!didWork)
        {
			Sleep(1);                           // CPU-Last reduzieren, wenn gerade nichts zu tun ist (keine Tasteneingabe und kein empfangenes Zeichen)
        }

    }
}

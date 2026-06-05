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
#include "fsk_demod.h"
#include "fsk_decode_rtty.h"
#include "sampleprocessing.h"
#include "fsk_decode_ascii.h"
#include "fsk_decode_ax25.h"
#include "buffer.h"

int main()
{
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    printf("                      RYTLTYTL                                          by dl8mcg 2026\n\n");
    printf("Mit F1, F2, F3, F4, F5 oder F6 den Modus auszuwählen              Mit F8 das Programm beenden\n");
    initialize_audiostream();
    init_fsk_demod(FSK_RTTY_45_BAUD_170Hz);

    while (1)
    {
        if (_kbhit())
        {
            int key = _getch(); // Erstes Zeichen lesen

            if (key == 0 || key == 224)
            {

                key = _getch(); // Zweites Zeichen lesen (Tastencode)

                switch (key)
                {
                    case 59: // F1
                        init_fsk_demod(FSK_RTTY_45_BAUD_170Hz);
                        break;
                    case 60: // F2
                       init_fsk_demod(FSK_RTTY_50_BAUD_85Hz);
                       break;
                    case 61: // F3
                        init_fsk_demod(FSK_RTTY_50_BAUD_450Hz);
                       break;
                    case 62: // F4
                        init_fsk_demod(FSK_EFR_200_BAUD_340Hz);
                        break;
                    case 63: // F5
                        init_fsk_demod(FSK_ASCII_300_BAUD_850Hz);
                        break;
                    case 64: // F6
                        init_fsk_demod(FSK_AX25_1200_BAUD_1000Hz);
                        break;
                    case 65: // F7
                        init_fsk_demod(FSK_AX25_9600_BAUD);
						break;
                    case 66: // F8
                        printf("\n\nProgramm beendet.\n\n");
                        stop_audiostream();
                        printf("73\n");
                        Sleep(1000);
                        return 0;
                    default:
                        ;
                }
            }
        }

        unsigned char value;
        if (readbuf(&value))
        {
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
    }
}
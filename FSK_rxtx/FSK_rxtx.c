/*
*   by dl8mcg Jan. 2025 to Nov. 2026       Hauptprogramm
*/
#include <stdio.h>
#include "console_rx.h"
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

/*
 * Liefert bei erweiterten Konsolentasten direkt den Scan-Code (z. B. 73 für
 * Page-Up).  _getch() liefert dafür zuerst 0 oder 0xE0 und anschließend den
 * eigentlichen Scan-Code.  Normale Tasten werden unverändert geliefert.
 */
int read_key(void)
{
    int key;

    if (_kbhit())
    {
        key = _getch();

        if (key == 0 || key == 0xE0)
        {
            return _getch();
        }

        return key;
    }

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

int main(int argc, char* argv[])
{
    /*
    * --------------------------------------------------------
    * Wurde diese EXE als RX-Konsolenprozess gestartet?
    * --------------------------------------------------------
    */
    if (argc >= 4 &&
        strcmp(argv[1], "--console") == 0)
    {
        return console_child(
            argv[2],
            argv[3]
        );
    }


    /*
     * --------------------------------------------------------
     * Normaler FSK-RX/TX-Hauptprozess
     * --------------------------------------------------------
     */

    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);

    printf("                           RYTLTYTL                                          by dl8mcg 2026\n\n");
    printf("            Mit den Page-Up- und Pade-Down-Tasten den Modus auszuwählen \n\n");

    if (console_rx_open("FSK RX"))
    {
        printf("RX-Konsole konnte nicht geöffnet werden.\n");
    }

    if (console_pkt_open("FSK PACKETS"))
    {
        printf("Packet-Konsole konnte nicht geöffnet werden.\n");
    }

    initialize_audiostream();
    init_fsk_demod_int(FSK_RTTY_45_BAUD_170Hz);

    while (1)
    {
		int handled_event = 0;            // Flag, um zu erkennen, ob etwas zu tun war (Tastendruck oder empfangenes Zeichen)
        int key = read_key();

        if (key != -1)
        {
			handled_event = 1;              // eine Taste wurde gedrückt

            switch (key)
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
        if (readbuf(&value))                // ins FSK RX - Fenster jedes empfangene Zeichen sofort ausgeben
        {
			handled_event = 1;              // ein Zeichen wurde empfangen

            static uint8_t last_was_cr = 0;

            /* CR behandeln */
            if (value == '\r')
            {
                printf_rx("\n");
                last_was_cr = 1;
                continue;
            }

            /* LF behandeln */
            if (value == '\n')
            {
                if (!last_was_cr)
                    printf_rx("\n");

                last_was_cr = 0;
                continue;
            }

            last_was_cr = 0;

            /* normales ASCII */
            if (value < 0x80)
            {
                printf_rx("%c", value);
            }
            else
            {   // Sonderzeichen in UTF8 wandeln
                printf_rx           
                (
                    "%c%c",
                    0xC0 | (value >> 6),
                    0x80 | (value & 0x3F)
                );
            }
        }


        const char* packet;

		if (ax25_packet_get(&packet))           // ins FSK PACKET - Fenster fehlerfrei empfangene Packete ausgeben
        {
            char packet_utf8[AX25_PACKET_BUFFER_SIZE * 2];
            const unsigned char* src = (const unsigned char*)packet;
            size_t dst = 0;

			handled_event = 1;                  // ein Packet wurde empfangen                  

            while (*src != '\0' && dst < sizeof(packet_utf8) - 1)
            {
                unsigned char value = *src++;

                static uint8_t last_was_cr = 0;

                /* CR behandeln */
                if (value == '\r')
                {
                    packet_utf8[dst++] = '\n';
                    last_was_cr = 1;
                    continue;
                }

                /* LF behandeln */
                if (value == '\n')
                {
                    if (!last_was_cr)
                    {
                        packet_utf8[dst++] = '\n';
                    }

                    last_was_cr = 0;
                    continue;
                }

                last_was_cr = 0;

                /* normales ASCII */
                if (value < 0x80)
                {
                    packet_utf8[dst++] = (char)value;
                }
                else
                {   // Sonderzeichen in UTF8 wandeln
                    if (dst + 2 >= sizeof(packet_utf8))
                        break;

                    packet_utf8[dst++] = (char)(0xC0 | (value >> 6));
                    packet_utf8[dst++] = (char)(0x80 | (value & 0x3F));
                }
            }

            packet_utf8[dst] = '\0';    // 0 am Ende des Strings

            printf_pkt("%s", packet_utf8);
            ax25_packet_release();
        }


        if (!handled_event)
        {
			Sleep(1);                           // CPU-Last reduzieren, wenn gerade nichts zu tun ist (keine Tasteneingabe und kein empfangenes Zeichen)
        }

    }
}

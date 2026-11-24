/*
*   by dl8mcg Okt. 2026 to Nov. 2026       Konsolenfenster
*/

#pragma once

#ifndef CONSOLE_RX_H
#define CONSOLE_RX_H

/*
 * ============================================================
 * FSK RX / PACKET Console
 *
 * Hauptfenster:
 *
 *     printf(...)
 *
 * FSK RX Fenster:
 *
 *     printf_rx(...)
 *
 * FSK PACKET Fenster:
 *
 *     printf_pkt(...)
 * ============================================================
 */

int console_child(const char* pipe_name, const char* title);

 /*
  * ------------------------------------------------------------
  * FSK RX
  * ------------------------------------------------------------
  */

int console_rx_open(const char* title);

void printf_rx(const char* format, ...);

void console_rx_close(void);


/*
 * ------------------------------------------------------------
 * FSK PACKETS
 * ------------------------------------------------------------
 */

int console_pkt_open(const char* title);

void printf_pkt(const char* format, ...);

void console_pkt_close(void);


#endif


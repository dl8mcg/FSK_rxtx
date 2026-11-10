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

//#ifndef CONSOLE_RX_H
//#define CONSOLE_RX_H
//
///*
// * ============================================================
// * FSK RX / PACKET Console
// *
// * Hauptfenster:
// *
// *     printf(...)
// *
// * FSK RX Fenster:
// *
// *     printf_rx(...)
// *
// * FSK PACKET Fenster:
// *
// *     printf_pkt(...)
// * ============================================================
// */
//
//
// /*
//  * ------------------------------------------------------------
//  * FSK RX
//  * ------------------------------------------------------------
//  */
//
//  /*
//   * Öffnet das zusätzliche FSK-RX-Konsolenfenster.
//   *
//   * Rückgabe:
//   *      0  = OK
//   *     -1  = Fehler
//   */
//int console_rx_open(const char* title);
//
//
///*
// * Ausgabe in das FSK-RX-Fenster.
// *
// * Verwendung wie printf().
// */
//void printf_rx(const char* format, ...);
//
//
///*
// * Schließt das FSK-RX-Fenster.
// */
//void console_rx_close(void);
//
//
///*
// * Wird intern vom RX-Konsolenprozess benutzt.
// */
////int console_rx_child(
////    const char* pipe_name,
////    const char* title);
//
//
///*
// * ------------------------------------------------------------
// * FSK PACKETS
// * ------------------------------------------------------------
// */
//
// /*
//  * Öffnet das zusätzliche FSK-PACKET-Konsolenfenster.
//  *
//  * Rückgabe:
//  *      0  = OK
//  *     -1  = Fehler
//  */
//int console_pkt_open(const char* title);
//
//
///*
// * Ausgabe in das FSK-PACKET-Fenster.
// *
// * Verwendung wie printf().
// */
//void printf_pkt(const char* format, ...);
//
//
///*
// * Schließt das FSK-PACKET-Fenster.
// */
//void console_pkt_close(void);
//
//
///*
// * Wird intern vom PACKET-Konsolenprozess benutzt.
// */
////int console_pkt_child(
////    const char* pipe_name,
////    const char* title);
//
//
//#endif
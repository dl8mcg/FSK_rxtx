/*
*   by dl8mcg Jan. 2025 to June 2026
*/

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include <stdarg.h>
#include <string.h>


#include <stddef.h>


#ifndef BUFFER_H
#define BUFFER_H

void writeToRingBuffer(const char* text);
void writeToRingBufferFormatted(const char* format, ...);

void writebuf(char val);
bool readbuf(char* val);




#ifndef AX25_PACKET_BUFFER_SIZE
#define AX25_PACKET_BUFFER_SIZE 2048
#endif

/* AX.25 Ping-Pong-Paketpuffer */

void ax25_packet_start(void);
void ax25_packet_put(char value);
void ax25_packet_finish(void);
void ax25_packet_abort(void);

/* Hauptprogramm: nächstes Paket ausgeben */
int ax25_packet_get(const char** data);
void ax25_packet_release(void);

#endif // BUFFER_H
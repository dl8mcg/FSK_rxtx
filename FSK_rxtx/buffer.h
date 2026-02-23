/*
*   by dl8mcg Jan. 2025
*/

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#ifndef BUFFER_H
#define BUFFER_H

void writeToRingBuffer(const char* text);
void writeToRingBufferFormatted(const char* format, ...);

void writebuf(char val);
bool readbuf(char* val);

#endif // BUFFER_H
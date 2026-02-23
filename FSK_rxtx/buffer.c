/*
*   by dl8mcg Jan. 2025       ASCII - decode
*/

#include "buffer.h"



#define RINGBUFFER_SIZE 1024 //256  // Größe des Buffers 
#define BUFFER_SIZE 1024 //256  // Größe des temporären Puffers

char ringbuffer[RINGBUFFER_SIZE];
volatile uint16_t write_index = 0;  // Schreibzeiger 
volatile uint16_t read_index = 0;   // Lesezeiger 

void writeToRingBuffer(const char* text) 
{
    int size = strlen(text);
    for (int i = 0; i < size; i++) 
    {
        writebuf(text[i]);  // Zeichenweise in den Ringpuffer schreiben
    }
}


//void writeToRingBufferFormatted(const char* format, unsigned char resbyte) 
//{
//    char buffer[100];  // Lokaler Buffer für formatierten String
//    sprintf_s(buffer, sizeof(buffer), format, resbyte);  // String formatieren
//
//    int size = strlen(buffer);
//    for (int i = 0; i < size; i++) {
//        writebuf(buffer[i]);  // Zeichenweise in den Ringpuffer schreiben
//    }
//}


void writeToRingBufferFormatted(const char* format, ...)
{
    char buffer[BUFFER_SIZE];  // Temporärer Puffer für den formatierten String
    va_list args;              // Variadische Argumente
    va_start(args, format);    // Initialisiere Argument-Liste

    vsnprintf(buffer, BUFFER_SIZE, format, args);  // String formatieren
    va_end(args);  // Beende Argumentverarbeitung

    int size = strlen(buffer);
    for (int i = 0; i < size; i++)
    {
        writebuf(buffer[i]);  // Zeichenweise in den Ringpuffer schreiben
    }
}

void writebuf(char val)
{
    // Schreibe das Zeichen in den Ringbuffer
    uint16_t next_write_index = (write_index + 1) % RINGBUFFER_SIZE;

    // Überprüfe, ob der Ringbuffer voll ist
    if (next_write_index != read_index)
    {
        ringbuffer[write_index] = val; 
        write_index = next_write_index;        
    }

}

bool readbuf(char* val)
{
    // Prüfe, ob Daten im Ringbuffer vorhanden sind
    if (read_index != write_index)
    {
        // Zeichen aus dem Ringbuffer lesen
        *val = ringbuffer[read_index];
        read_index = (read_index + 1) % RINGBUFFER_SIZE;
        return true;
    }
    return false;
}
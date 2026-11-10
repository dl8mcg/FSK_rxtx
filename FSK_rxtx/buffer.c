/*
*   by dl8mcg Jan. 2025 to Nov. 2026       Ringbuffer für dekodierte Zeichen
*/

#include "buffer.h"

#define RINGBUFFER_SIZE 4096        // Größe des Buffers 
#define BUFFER_SIZE 4096            // Größe des temporären Puffers

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

void writeToRingBufferFormatted(const char* format, ...)
{
    char buffer[BUFFER_SIZE];  // Temporärer Puffer für den formatierten String
    va_list args;              // Variadische Argumente
    va_start(args, format);    // Initialisiere Argument-Liste

    vsnprintf(buffer, BUFFER_SIZE, format, args);  // String formatieren
    va_end(args);               // Beende Argumentverarbeitung

    int size = strlen(buffer);
    for (int i = 0; i < size; i++)
    {
        writebuf(buffer[i]);    // Zeichenweise in den Ringpuffer schreiben
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


#include <stddef.h>
//#include "buffer.h"

/* Zwei Paketpuffer */
static char packet_a[AX25_PACKET_BUFFER_SIZE];
static char packet_b[AX25_PACKET_BUFFER_SIZE];

/* Länge der Pakete */
static size_t length_a = 0;
static size_t length_b = 0;

/* 1 = Paket fertig zur Ausgabe */
static volatile int ready_a = 0;
static volatile int ready_b = 0;

/* Schreibpuffer: 0 = A, 1 = B */
static int write_buffer = 0;

/* Aktueller Schreibzustand */
static char* write_data = packet_a;
static size_t* write_length = &length_a;
static volatile int* write_ready = &ready_a;

/* Aktueller Lesepuffer: 0 = A, 1 = B */
static int read_buffer = 0;

/* Schreibpuffer für ein neues Paket auswählen */
void ax25_packet_start(void)
{
    if (write_buffer == 0)
    {
        write_data = packet_a;
        write_length = &length_a;
        write_ready = &ready_a;
    }
    else
    {
        write_data = packet_b;
        write_length = &length_b;
        write_ready = &ready_b;
    }

    *write_length = 0;
    write_data[0] = '\0';
}

/* Zeichen hinzufügen */
void ax25_packet_put(char value)
{
    if (*write_length < AX25_PACKET_BUFFER_SIZE - 1)
    {
        write_data[*write_length] = value;
        (*write_length)++;
        write_data[*write_length] = '\0';
    }
}

/* Vollständiges Paket veröffentlichen und Schreibpuffer wechseln */
void ax25_packet_finish(void)
{
    write_data[*write_length] = '\0';
    *write_ready = 1;

    write_buffer ^= 1;
}

/* Paket bei falscher CRC verwerfen */
void ax25_packet_abort(void)
{
    *write_length = 0;
    write_data[0] = '\0';

    /* Bei Fehler nicht auf den nächsten Puffer wechseln.
       Der nächste gültige Frame verwendet denselben Puffer. */
}

/* Nächstes Paket in der Reihenfolge A, B, A, B abholen */
int ax25_packet_get(const char** data)
{
    if (data == NULL)
        return 0;

    if (read_buffer == 0)
    {
        if (!ready_a)
            return 0;

        *data = packet_a;
        return 1;
    }
    else
    {
        if (!ready_b)
            return 0;

        *data = packet_b;
        return 1;
    }
}

/* Ausgegebenen Puffer freigeben und Lesereihenfolge wechseln */
void ax25_packet_release(void)
{
    if (read_buffer == 0)
        ready_a = 0;
    else
        ready_b = 0;

    read_buffer ^= 1;
}

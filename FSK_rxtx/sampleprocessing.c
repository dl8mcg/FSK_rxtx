/*
*   by dl8mcg Jan. 2025
*/

#include <portaudio.h>
#include <stdio.h>
#include <locale.h>
#include <windows.h>
#include "config.h"
#include "fsk_demod.h"
#define _USE_MATH_DEFINES
#include <math.h>
#define DATA_RATE 1200
#define nco_inc  2 * (double)M_PI * DATA_RATE / SAMPLING_RATE

volatile double nco_phase;
volatile float sample;

PaStream* stream;

// Audio-Callback-Funktion
static int audioCallback(const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void* userData)
{
    if (inputBuffer == NULL)
        return paContinue;

    const float* input = (const float*)inputBuffer; 
    float* output = (float*)outputBuffer; 

    for (unsigned long i = 0; i < framesPerBuffer; i++)
    {
        nco_phase += nco_inc;
        if (nco_phase > 2 * (float)M_PI) nco_phase -= 2 * (float)M_PI;
        sample = cosf(nco_phase);
        output[i] = sample * 0.1;  //+ 0.9 * input[i];

        process_fsk_demodulation(input[i]);     // FSK-Demodulation

    }
    return paContinue;
}


int initialize_audiostream()
{
    Pa_Initialize();

    // Geräteinformationen abrufen
    int numDevices = Pa_GetDeviceCount();
    if (numDevices < 0)
    {
        printf("Fehler beim Abrufen der Geräteanzahl: %s\n", Pa_GetErrorText(numDevices));
        return 1;
    }

    printf("Verfügbare Geräte:\n");
    for (int i = 0; i < numDevices; i++)
    {
        const PaDeviceInfo* deviceInfo = Pa_GetDeviceInfo(i);
        printf("[%d] Name: %s, Eingabekanäle: %d, Ausgabekanäle: %d\n",
            i, deviceInfo->name, deviceInfo->maxInputChannels, deviceInfo->maxOutputChannels);
    }

    // Eingabe- und Ausgabegerät auswählen
    int inputDevice = 0;  // Standardmäßig erstes Gerät (kann angepasst werden)
    int outputDevice = 2; // Beispiel: USB-Soundkarte (ändere die ID basierend auf der Liste)

    // Eingabeparameter
    PaStreamParameters inputParameters;
    inputParameters.device = inputDevice;
    inputParameters.channelCount = 1;           // Mono
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = 0.05;    // Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
    inputParameters.hostApiSpecificStreamInfo = NULL;

    // Ausgabeparameter
    PaStreamParameters outputParameters;
    outputParameters.device = outputDevice;
    outputParameters.channelCount = 1;          // Mono
    outputParameters.sampleFormat = paFloat32;
    outputParameters.suggestedLatency = 0.05;   // Pa_GetDeviceInfo(outputParameters.device)->defaultLowOutputLatency;
    outputParameters.hostApiSpecificStreamInfo = NULL;

    // Stream öffnen
    PaError err = Pa_OpenStream(&stream, &inputParameters, &outputParameters, SAMPLING_RATE, FRAMES_PER_BUFFER, paClipOff, audioCallback, NULL);

    if (err != paNoError)
    {
        printf("Fehler beim Öffnen des Streams: %s\n", Pa_GetErrorText(err));
        Pa_Terminate();
        return 1;
    }

    // Stream starten
    Pa_StartStream(stream);
    printf("Audio-Streaming läuft... Drücke Enter zum Beenden.\n");
    return 0;
}

void stop_audiostream()
{
    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();
}

void stop()
{
    Pa_StopStream(stream);
}

void start()
{
    Pa_StartStream(stream);
}
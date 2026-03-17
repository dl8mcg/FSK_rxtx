/*
*   by dl8mcg Jan. 2025 .. März 2026       sample processing
*/

#include <portAudio.h>
#include <stdio.h>
#include <locale.h>
#include <windows.h>
#include "config.h"
#include "fsk_demod.h"
#define _USE_MATH_DEFINES
#include <math.h>
#define DATA_RATE 1200
#define nco_inc  2 * (float)M_PI * DATA_RATE / SAMPLING_RATE

volatile float nco_phase = 0.0f;
volatile float sample = 0.0f;

PaStream* stream = NULL;

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

        process_fsk_demodulation(input[i]);     // FSK-Demodulation

        if (output) output[i] = 0.0f; // optional: stummes Output, sicherstellen dass output nicht dereferenziert wird wenn NULL
    }
    return paContinue;
}


int initialize_audiostream()
{
    PaError err = Pa_Initialize();
    if (err != paNoError)
    {
        printf("Pa_Initialize Fehler: %s\n", Pa_GetErrorText(err));
        return 1;
    }

    // Geräteinformationen abrufen
    int numDevices = Pa_GetDeviceCount();
    if (numDevices < 0)
    {
        printf("Fehler beim Abrufen der Geräteanzahl: %s\n", Pa_GetErrorText(numDevices));
        Pa_Terminate();
        return 1;
    }

    printf("Verfügbare Geräte:\n");
    for (int i = 0; i < numDevices; i++)
    {
        const PaDeviceInfo* deviceInfo = Pa_GetDeviceInfo(i);
        printf("[%d] Name: %s, Eingabekanäle: %d, Ausgabekanäle: %d\n",
            i, deviceInfo->name, deviceInfo->maxInputChannels, deviceInfo->maxOutputChannels);
    }

    // Versuche Default-Devices zuerst
    int inputDevice = Pa_GetDefaultInputDevice();
    int outputDevice = Pa_GetDefaultOutputDevice();

    // Fallback: erstes verfügbares Device mit Eingabekanälen / Ausgabekanälen
    if (inputDevice == paNoDevice)
    {
        inputDevice = -1;
        for (int i = 0; i < numDevices; i++)
        {
            const PaDeviceInfo* di = Pa_GetDeviceInfo(i);
            if (di && di->maxInputChannels > 0) { inputDevice = i; break; }
        }
    }
    if (outputDevice == paNoDevice)
    {
        outputDevice = -1;
        for (int i = 0; i < numDevices; i++)
        {
            const PaDeviceInfo* di = Pa_GetDeviceInfo(i);
            if (di && di->maxOutputChannels > 0) { outputDevice = i; break; }
        }
    }

    if (inputDevice < 0)
    {
        printf("Kein Eingabegerät gefunden.\n");
        Pa_Terminate();
        return 1;
    }
    if (outputDevice < 0)
    {
        printf("Kein Ausgabegerät gefunden.\n");
        Pa_Terminate();
        return 1;
    }

    const PaDeviceInfo* inInfo = Pa_GetDeviceInfo(inputDevice);
    const PaDeviceInfo* outInfo = Pa_GetDeviceInfo(outputDevice);

    // Eingabeparameter (initialisiert, um uninitialisierte Felder zu vermeiden)
    PaStreamParameters inputParameters;
    memset(&inputParameters, 0, sizeof(inputParameters));
    inputParameters.device = inputDevice;
    inputParameters.channelCount = 1;           // Mono
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = inInfo ? inInfo->defaultLowInputLatency : 0.05;
    inputParameters.hostApiSpecificStreamInfo = NULL;

    // Ausgabeparameter
    PaStreamParameters outputParameters;
    memset(&outputParameters, 0, sizeof(outputParameters));
    outputParameters.device = outputDevice;
    outputParameters.channelCount = 1;          // Mono
    outputParameters.sampleFormat = paFloat32;
    outputParameters.suggestedLatency = outInfo ? outInfo->defaultLowOutputLatency : 0.05;
    outputParameters.hostApiSpecificStreamInfo = NULL;

    // Prüfen ob das Format unterstützt wird
    err = Pa_IsFormatSupported(&inputParameters, &outputParameters, (double)SAMPLING_RATE);
    if (err != paNoError)
    {
        printf("Geräteformat nicht unterstützt (Pa_IsFormatSupported): %s\n", Pa_GetErrorText(err));
        printf("Input-Device [%d]: %s, defaultSampleRate: %.0f, maxInputChannels: %d\n",
            inputDevice, inInfo ? inInfo->name : "unknown", inInfo ? inInfo->defaultSampleRate : 0.0, inInfo ? inInfo->maxInputChannels : 0);
        printf("Output-Device [%d]: %s, defaultSampleRate: %.0f, maxOutputChannels: %d\n",
            outputDevice, outInfo ? outInfo->name : "unknown", outInfo ? outInfo->defaultSampleRate : 0.0, outInfo ? outInfo->maxOutputChannels : 0);
        Pa_Terminate();
        return 1;
    }

    // Stream öffnen
    err = Pa_OpenStream(&stream, &inputParameters, &outputParameters, SAMPLING_RATE, FRAMES_PER_BUFFER, paClipOff, audioCallback, NULL);

    if (err != paNoError)
    {
        printf("Fehler beim Öffnen des Streams: %s\n", Pa_GetErrorText(err));
        printf("Versuchtes Input-Device [%d]: %s\n", inputDevice, inInfo ? inInfo->name : "unknown");
        printf("Versuchtes Output-Device [%d]: %s\n", outputDevice, outInfo ? outInfo->name : "unknown");
        Pa_Terminate();
        return 1;
    }

    // Stream starten
    err = Pa_StartStream(stream);
    if (err != paNoError)
    {
        printf("Fehler beim Starten des Streams: %s\n", Pa_GetErrorText(err));
        Pa_CloseStream(stream);
        Pa_Terminate();
        return 1;
    }

    printf("Audio-Streaming läuft... Drücke Enter zum Beenden.\n");
    return 0;
}

void stop_audiostream()
{
    if (stream)
    {
        Pa_StopStream(stream);
        Pa_CloseStream(stream);
        stream = NULL;
    }
    Pa_Terminate();
}

void stop()
{
    if (stream) Pa_StopStream(stream);
}

void start()
{
    if (stream) Pa_StartStream(stream);
}
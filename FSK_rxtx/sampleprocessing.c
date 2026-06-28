/*
*   by dl8mcg Jan. 2025 to June 2026       sample processing
*/

#include <portAudio.h>
#include <stdio.h>
#include <windows.h>
#include <string.h>
#include "config.h"
//#include "fsk_demod_float.h"
#include "fsk_demod_int.h"
#define _USE_MATH_DEFINES
#include <math.h>
#define DATA_RATE 1200
#define nco_inc  2 * (float)M_PI * DATA_RATE / SAMPLING_RATE

volatile float nco_phase = 0.0f;
volatile float sample = 0.0f;

PaStream* stream = NULL;


static int32_t float_to_q31(float x)
{
    if (!isfinite(x)) return 0;
    if (x >= 1.0f) return INT32_MAX;
    if (x <= -1.0f) return INT32_MIN;
    return (int32_t)llround((double)x * 2147483648.0);
}

// Audio-Callback-Funktion
static int audioCallback(const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void* userData)
{
    if (inputBuffer == NULL)
        return paContinue;

    const float* input = (const float*)inputBuffer;
    float* output = (float*)outputBuffer;

    for (unsigned long i = 0; i < framesPerBuffer; i++)
    {
		//output[i * 2] = input[i];     // Linker Kanal Debugging
		//output[i * 2 + 1] = input[i]; // Rechter Kanal Debugging

        int32_t sample_q31 = float_to_q31(input[i] );
		FskAmplitudes amp = smDemod_int(sample_q31);   // FSK-Demodulation int32-Version

        //FskAmplitudes amp = smDemod_float(input[i]);   // FSK-Demodulation float-Version  

        if (output)
        {
            output[i * 2] = amp.amp1; // Linker Kanal debugging
            output[i * 2 + 1] = amp.amp2; // Rechter Kanal debugging
        }
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
        printf("Fehler beim Abrufen der Geraeteanzahl: %s\n", Pa_GetErrorText((PaError)numDevices));
        Pa_Terminate();
        return 1;
    }

    //printf("Verfuegbare Geraete:\n");
    //for (int i = 0; i < numDevices; i++)
    //{
    //    const PaDeviceInfo* deviceInfo = Pa_GetDeviceInfo(i);
    //    if (!deviceInfo) continue;
    //    printf("[%d] Name: %s, Eingabekanaele: %d, Ausgabekanaele: %d\n",
    //        i, deviceInfo->name, deviceInfo->maxInputChannels, deviceInfo->maxOutputChannels);
    //}

    // Default-Devices initialisieren (sicherer Startpunkt)
    int inputDevice = Pa_GetDefaultInputDevice();
    int outputDevice = Pa_GetDefaultOutputDevice();

    // Suche nach einem USB-Device (case-insensitive "USB" im Namen)
    int usbInput = -1;
    int usbOutput = -1;

    for (int i = 0; i < numDevices; i++)
    {
        const PaDeviceInfo* di = Pa_GetDeviceInfo(i);
        if (!di) continue;
        const char* name = di->name;
        if (name == NULL) continue;

        // einfache, robuste Prüfung auf "USB" (groß/klein egal)
        if (strstr(name, "USB") || strstr(name, "usb"))
        {
            if (di->maxInputChannels > 0 && usbInput < 0) usbInput = i;
            if (di->maxOutputChannels > 0 && usbOutput < 0) usbOutput = i;
        }
    }

    // Wenn USB-Geräte gefunden wurden, bevorzugen (überschreibt Default)
    if (usbInput >= 0) inputDevice = usbInput;
    if (usbOutput >= 0) outputDevice = usbOutput;

    // Falls kein Default bzw. USB-Device vorhanden: Fallback auf erstes geeignetes Gerät
    if (inputDevice == paNoDevice || inputDevice < 0)
    {
        inputDevice = -1;
        for (int i = 0; i < numDevices; i++)
        {
            const PaDeviceInfo* di = Pa_GetDeviceInfo(i);
            if (di && di->maxInputChannels > 0)
            {
                inputDevice = i;
                break;
            }
        }
    }
    if (outputDevice == paNoDevice || outputDevice < 0)
    {
        outputDevice = -1;
        for (int i = 0; i < numDevices; i++)
        {
            const PaDeviceInfo* di = Pa_GetDeviceInfo(i);
            if (di && di->maxOutputChannels > 0)
            {
                outputDevice = i;
                break;
            }
        }
    }

    if (inputDevice < 0)
    {
        printf("Kein Eingabegeraet gefunden.\n");
        Pa_Terminate();
        return 1;
    }
    if (outputDevice < 0)
    {
        printf("Kein Ausgabegeraet gefunden.\n");
        Pa_Terminate();
        return 1;
    }

    const PaDeviceInfo* inInfo = Pa_GetDeviceInfo(inputDevice);
    const PaDeviceInfo* outInfo = Pa_GetDeviceInfo(outputDevice);

    printf("Verwende Input-Device  [%d]: %s\n", inputDevice, inInfo ? inInfo->name : "unknown");
    printf("Verwende Output-Device [%d]: %s\n", outputDevice, outInfo ? outInfo->name : "unknown");

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
    outputParameters.channelCount = 2;          // Stereo
    outputParameters.sampleFormat = paFloat32;
    outputParameters.suggestedLatency = outInfo ? outInfo->defaultLowOutputLatency : 0.05;
    outputParameters.hostApiSpecificStreamInfo = NULL;

    // Prüfen ob das Format unterstützt wird
    err = Pa_IsFormatSupported(&inputParameters, &outputParameters, (double)SAMPLING_RATE);
    if (err != paNoError)
    {
        printf("Geraeteformat nicht unterstuetzt (Pa_IsFormatSupported): %s\n", Pa_GetErrorText(err));
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
        printf("Fehler beim Oeffnen des Streams: %s\n", Pa_GetErrorText(err));
        printf("Versuchtes Input-Device [%d]: %s\n", inputDevice, inInfo ? inInfo->name : "unknown");
        printf("Versuchtes Output-Device [%d]: %s\n", outputDevice, outInfo ? outInfo->name : "unknown");
        Pa_Terminate();
        return 1;
    }

    const PaStreamInfo* streamInfo = Pa_GetStreamInfo(stream);
    if (streamInfo)
    {
        printf("Tatsaechliche Samplerate:       %.1f Hz\n", streamInfo->sampleRate);
        //printf("Input Latenz:                  %.3f ms\n", streamInfo->inputLatency * 1000.0);
        //printf("Output Latenz:                 %.3f ms\n", streamInfo->outputLatency * 1000.0);
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

    printf("\n");
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


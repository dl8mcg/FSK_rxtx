#include <stdio.h>
#include <portaudio.h>
#define _USE_MATH_DEFINES
#include <math.h>

#define SAMPLE_RATE 44100
#define FRAMES_PER_BUFFER 256

#define F_TONE 1000

#define nco_inc  2 * (double)M_PI * F_TONE / SAMPLE_RATE;

volatile double nco_phase;

volatile float sample;

// Audio-Callback-Funktion
static int audioCallback(const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void* userData)
{
    const float* in = (const float*)inputBuffer;
    float* out = (float*)outputBuffer;
    (void)timeInfo; (void)statusFlags; (void)userData;

    for (unsigned long i = 0; i < framesPerBuffer; i++)
    {
        nco_phase += nco_inc;
        if (nco_phase > 2 * (float)M_PI) nco_phase -= 2 * (float)M_PI;
        sample = cosf(nco_phase);
        out[i] = sample * 0.91 + 0.9 * in[i];

        //out[i] = in[i];             // Einfaches Durchleiten
    }
    return paContinue;
}

int main()
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
    PaStream* stream;
    PaError err = Pa_OpenStream(&stream, &inputParameters, &outputParameters, SAMPLE_RATE, FRAMES_PER_BUFFER, paClipOff, audioCallback, NULL);

    if (err != paNoError)
    {
        printf("Fehler beim Öffnen des Streams: %s\n", Pa_GetErrorText(err));
        Pa_Terminate();
        return 1;
    }

    // Stream starten
    Pa_StartStream(stream);
    printf("Audio-Streaming läuft... Drücke Enter zum Beenden.\n");
    getchar();

    // Stream stoppen und beenden
    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();

    return 0;
}

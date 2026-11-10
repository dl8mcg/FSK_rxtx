/*
*   by dl8mcg Okt. 2026 to Nov. 2026       Konsolenfenster
*/

#define _CRT_SECURE_NO_WARNINGS
#include "console_rx.h"
#include <windows.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/*
 * ============================================================
 * Globale Pipe-Handles
 * ============================================================
 */

 /*
  * Verbindung zum FSK-RX-Fenster
  */
static HANDLE hPipeRx = INVALID_HANDLE_VALUE;


/*
 * Verbindung zum FSK-PACKET-Fenster
 */
static HANDLE hPipePkt = INVALID_HANDLE_VALUE;


/*
 * Die Pipe ist nur eine Anzeige.  PIPE_NOWAIT verhindert, dass ein voller
 * Pipepuffer den Echtzeit-/Decoderpfad blockiert.  Bei einem angehaltenen
 * oder geschlossenen Anzeigefenster dürfen daher höchstens Anzeigedaten,
 * niemals Empfang oder Tastaturverarbeitung, verloren gehen.
 */
static void console_set_pipe_nonblocking(HANDLE pipe)
{
    DWORD mode = PIPE_READMODE_MESSAGE | PIPE_NOWAIT;

    SetNamedPipeHandleState(pipe, &mode, NULL, NULL);
}


/* Die Anzeige darf den Decoder nie anhalten.  Bei einer vollen Pipe wird
   deshalb nur der aktuelle Anzeige-Block verworfen. */
static void console_write_display(
    HANDLE* pipe,
    const char* buffer,
    DWORD length)
{
    DWORD written;

    if (*pipe == INVALID_HANDLE_VALUE || length == 0)
    {
        return;
    }

    if (!WriteFile(*pipe, buffer, length, &written, NULL))
    {
        DWORD error = GetLastError();

        if (error == ERROR_BROKEN_PIPE ||
            error == ERROR_PIPE_NOT_CONNECTED)
        {
            CloseHandle(*pipe);
            *pipe = INVALID_HANDLE_VALUE;
        }
    }
}


/*
 * ============================================================
 * Interne Funktion:
 *
 * Öffnet einen neuen Konsolenprozess und verbindet ihn
 * über eine Named Pipe.
 *
 * Diese Funktion wird von
 *
 *     console_rx_open()
 *
 * und
 *
 *     console_pkt_open()
 *
 * verwendet.
 * ============================================================
 */

static HANDLE console_open_channel(
    const char* pipe_prefix,
    const char* title)
{
    char pipe_name[128];
    char exe_name[MAX_PATH];
    char command_line[512];

    HANDLE hPipeServer;

    STARTUPINFOA si;
    PROCESS_INFORMATION pi;

    DWORD pid;


    /*
     * --------------------------------------------------------
     * Pipe-Namen erzeugen
     *
     * Beispiel:
     *
     * \\.\pipe\FSK_RX_12345
     *
     * bzw.
     *
     * \\.\pipe\FSK_PKT_12345
     * --------------------------------------------------------
     */

    pid = GetCurrentProcessId();

    sprintf_s(
        pipe_name,
        sizeof(pipe_name),
        "\\\\.\\pipe\\%s_%lu",
        pipe_prefix,
        (unsigned long)pid
    );


    /*
     * --------------------------------------------------------
     * Named Pipe erzeugen
     * --------------------------------------------------------
     */

    hPipeServer = CreateNamedPipeA(
        pipe_name,
        PIPE_ACCESS_OUTBOUND,
        PIPE_TYPE_MESSAGE |
        PIPE_READMODE_MESSAGE |
        PIPE_WAIT,
        1,
        4096,
        4096,
        0,
        NULL
    );

    if (hPipeServer == INVALID_HANDLE_VALUE)
    {
        return INVALID_HANDLE_VALUE;
    }


    /*
     * --------------------------------------------------------
     * Namen der eigenen EXE ermitteln
     * --------------------------------------------------------
 */

    if (GetModuleFileNameA(
        NULL,
        exe_name,
        sizeof(exe_name)) == 0)
    {
        CloseHandle(hPipeServer);

        return INVALID_HANDLE_VALUE;
    }


    /*
     * --------------------------------------------------------
     * Kommandozeile für den neuen Prozess.
     *
     * Die EXE wird nochmals gestartet.
     *
     * Beispiel:
     *
     * fsk_rxtx.exe
     * --console
     * "\\.\pipe\FSK_RX_12345"
     * "FSK RX"
     * --------------------------------------------------------
     */

    sprintf_s(
        command_line,
        sizeof(command_line),
        "\"%s\" --console \"%s\" \"%s\"",
        exe_name,
        pipe_name,
        title ? title : "FSK"
    );


    /*
     * --------------------------------------------------------
     * STARTUPINFO
     * --------------------------------------------------------
     */

    ZeroMemory(
        &si,
        sizeof(si)
    );

    si.cb = sizeof(si);


    /*
     * PROCESS_INFORMATION
     */

    ZeroMemory(
        &pi,
        sizeof(pi)
    );


    /*
     * --------------------------------------------------------
     * Neuen Prozess starten.
     *
     * CREATE_NEW_CONSOLE erzeugt ein neues
     * Konsolenfenster.
     * --------------------------------------------------------
     */

    if (!CreateProcessA(
        NULL,
        command_line,
        NULL,
        NULL,
        FALSE,
        CREATE_NEW_CONSOLE,
        NULL,
        NULL,
        &si,
        &pi))
    {
        CloseHandle(hPipeServer);

        return INVALID_HANDLE_VALUE;
    }


    /*
     * Prozess-Handles werden nicht mehr benötigt.
     */

    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);


    /*
     * --------------------------------------------------------
     * Warten bis der neue Prozess die Pipe öffnet.
     * --------------------------------------------------------
     */

    if (!ConnectNamedPipe(
        hPipeServer,
        NULL))
    {
        DWORD error;

        error = GetLastError();

        if (error != ERROR_PIPE_CONNECTED)
        {
            CloseHandle(hPipeServer);

            return INVALID_HANDLE_VALUE;
        }
    }


    console_set_pipe_nonblocking(hPipeServer);


    /*
     * Verbindung steht.
     */

    return hPipeServer;
}


/*
 * ============================================================
 * FSK RX
 * ============================================================
 */


 /*
  * ------------------------------------------------------------
  * console_rx_open()
  * ------------------------------------------------------------
  */

int console_rx_open(const char* title)
{
    HANDLE h;


    h = console_open_channel(
        "FSK_RX",
        title ? title : "FSK RX"
    );


    if (h == INVALID_HANDLE_VALUE)
    {
        return -1;
    }


    hPipeRx = h;

    return 0;
}


/*
 * ------------------------------------------------------------
 * printf_rx()
 *
 * Verwendung wie printf():
 *
 *     printf_rx("RX: %c\n", c);
 *     printf_rx("Byte = %02X\n", byte);
 * ------------------------------------------------------------
 */

void printf_rx(const char* format, ...)
{
    char buffer[4096];
    static char pending[4096];
    static size_t pending_length = 0;
    static DWORD last_flush = 0;

    int n;
    DWORD now;

    va_list args;


    /*
     * Keine RX-Konsole geöffnet?
     */

    if (hPipeRx == INVALID_HANDLE_VALUE)
    {
        return;
    }


    /*
     * Argumente wie bei printf() formatieren.
     */

    va_start(
        args,
        format
    );


    n = vsnprintf_s(
        buffer,
        sizeof(buffer),
        _TRUNCATE,
        format,
        args
    );


    va_end(args);


    /*
     * Fehler bei der Formatierung.
     */

    if (n < 0)
    {
        return;
    }

    /*
     * Nicht jedes Zeichen einzeln über Prozess- und Konsolengrenze senden.
     * Besonders bei 1200/9600 Baud kann das Kindfenster sonst trotz leerer
     * Pipe mit tausenden ReadFile()/WriteConsole()-Aufrufen pro Sekunde
     * hinterherlaufen.  Ein Block wird spätestens nach 25 ms ausgegeben.
     */
    if ((size_t)n > sizeof(pending) - pending_length)
    {
        console_write_display(&hPipeRx, pending, (DWORD)pending_length);
        pending_length = 0;
        last_flush = GetTickCount();
    }

    memcpy(pending + pending_length, buffer, (size_t)n);
    pending_length += (size_t)n;

    now = GetTickCount();

    if (memchr(buffer, '\n', (size_t)n) != NULL ||
        pending_length >= 256 ||
        (DWORD)(now - last_flush) >= 25)
    {
        console_write_display(&hPipeRx, pending, (DWORD)pending_length);
        pending_length = 0;
        last_flush = now;
    }
}


/*
 * ------------------------------------------------------------
 * console_rx_close()
 * ------------------------------------------------------------
 */

void console_rx_close(void)
{
    if (hPipeRx != INVALID_HANDLE_VALUE)
    {
        /* Anzeige ist best effort: FlushFileBuffers kann hier unbegrenzt
           warten, wenn das RX-Fenster nicht mehr aus der Pipe liest. */
        DisconnectNamedPipe(hPipeRx);

        CloseHandle(hPipeRx);

        hPipeRx = INVALID_HANDLE_VALUE;
    }
}


/*
 * ============================================================
 * FSK PACKETS
 * ============================================================
 */


 /*
  * ------------------------------------------------------------
  * console_pkt_open()
  * ------------------------------------------------------------
  */

int console_pkt_open(const char* title)
{
    HANDLE h;


    h = console_open_channel(
        "FSK_PKT",
        title ? title : "FSK PACKETS"
    );


    if (h == INVALID_HANDLE_VALUE)
    {
        return -1;
    }


    hPipePkt = h;

    return 0;
}


/*
 * ------------------------------------------------------------
 * printf_pkt()
 *
 * Verwendung wie printf():
 *
 *     printf_pkt("CRC OK\n");
 *     printf_pkt("Length = %d\n", length);
 * ------------------------------------------------------------
 */

void printf_pkt(const char* format, ...)
{
    char buffer[4096];

    int n;

    va_list args;


    /*
     * Keine Packet-Konsole geöffnet?
     */

    if (hPipePkt == INVALID_HANDLE_VALUE)
    {
        return;
    }


    /*
     * Argumente wie bei printf() formatieren.
     */

    va_start(
        args,
        format
    );


    n = vsnprintf_s(
        buffer,
        sizeof(buffer),
        _TRUNCATE,
        format,
        args
    );


    va_end(args);


    /*
     * Fehler bei der Formatierung.
     */

    if (n < 0)
    {
        return;
    }


    console_write_display(&hPipePkt, buffer, (DWORD)n);
}


/*
 * ------------------------------------------------------------
 * console_pkt_close()
 * ------------------------------------------------------------
 */

void console_pkt_close(void)
{
    if (hPipePkt != INVALID_HANDLE_VALUE)
    {
        /* Auch beim Beenden darf ein blockiertes Anzeigefenster den
           Hauptprozess nicht aufhalten. */
        DisconnectNamedPipe(hPipePkt);

        CloseHandle(hPipePkt);

        hPipePkt = INVALID_HANDLE_VALUE;
    }
}


/*
 * ============================================================
 * Gemeinsame Child-Funktion
 *
 * Diese Funktion liest aus einer Named Pipe und schreibt
 * den Inhalt in die eigene Console.
 * ============================================================
 */

int console_child(
    const char* pipe_name,
    const char* title)
{
    HANDLE pipe;

    HANDLE stdout_handle;

    char buffer[4096];

    DWORD n;

    DWORD written;


    /*
     * --------------------------------------------------------
     * Fenstertitel
     * --------------------------------------------------------
     */

    if (title != NULL)
    {
        SetConsoleTitleA(title);
    }


    /*
     * --------------------------------------------------------
     * UTF-8
     * --------------------------------------------------------
     */

    SetConsoleOutputCP(CP_UTF8);

    SetConsoleCP(CP_UTF8);


    /*
     * --------------------------------------------------------
     * Named Pipe öffnen
     * --------------------------------------------------------
     */

    pipe = CreateFileA(
        pipe_name,
        GENERIC_READ | FILE_WRITE_ATTRIBUTES,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );


    if (pipe == INVALID_HANDLE_VALUE)
    {
        printf(
            "Pipe konnte nicht geoeffnet werden.\n"
        );

        return 1;
    }


    /*
     * --------------------------------------------------------
     * MESSAGE-Modus setzen
     * --------------------------------------------------------
     */

    {
        DWORD mode;

        mode = PIPE_READMODE_MESSAGE;

        SetNamedPipeHandleState(
            pipe,
            &mode,
            NULL,
            NULL
        );
    }


    /*
     * --------------------------------------------------------
     * Handle der eigenen Console
     * --------------------------------------------------------
     */

    stdout_handle =
        GetStdHandle(STD_OUTPUT_HANDLE);


    /*
     * --------------------------------------------------------
     * Empfangsschleife
     * --------------------------------------------------------
 */

    while (1)
    {
        if (!ReadFile(
            pipe,
            buffer,
            sizeof(buffer) - 1,
            &n,
            NULL))
        {
            /*
             * Hauptprogramm beendet oder Pipe geschlossen.
             */

            break;
        }


        /*
         * String-Ende setzen.
         */

        buffer[n] = '\0';


        /*
         * Nachricht direkt in die eigene Console schreiben.
         */

        WriteConsoleA(
            stdout_handle,
            buffer,
            n,
            &written,
            NULL
        );
    }


    /*
     * Pipe schließen.
     */

    CloseHandle(pipe);


    return 0;
}



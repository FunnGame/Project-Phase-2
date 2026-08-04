using System.ComponentModel;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;

namespace Station.ControlApp.Transport;

/// <summary>
/// A minimal write-only serial link to the station board, built on
/// <see cref="SerialNative"/> so the app needs no NuGet packages.
///
/// The Nucleo-F446RE's ST-Link enumerates as a USB virtual COM port; the
/// firmware listens on USART2 at 115200 8N1
/// (see station/app/app_config.h: APP_UART_BAUD).
///
/// UI-agnostic on purpose, so a console tool could reuse it.
/// </summary>
public sealed class SerialLink : IDisposable
{
    /// <summary>Baud rate the station firmware expects.</summary>
    public const int DefaultBaud = 115200;

    private SafeFileHandle? _handle;

    /// <summary>True while the port is open.</summary>
    public bool IsOpen => _handle is { IsInvalid: false, IsClosed: false };

    /// <summary>Name of the open port, or null.</summary>
    public string? PortName { get; private set; }

    /// <summary>Frames successfully written since the port was opened.</summary>
    public uint FramesSent { get; private set; }

    /// <summary>Last error message, or null if the last operation succeeded.</summary>
    public string? LastError { get; private set; }

    /// <summary>
    /// Lists the COM ports currently present, e.g. ["COM3", "COM7"].
    /// Sorted by number so the list is stable between calls.
    /// </summary>
    public static IReadOnlyList<string> GetPortNames()
    {
        var names = new List<string>();

        // 64 KB is enough for the full DOS device list on any normal system.
        var buffer = new char[64 * 1024];
        uint written = SerialNative.QueryDosDevice(null, buffer, (uint)buffer.Length);
        if (written == 0)
        {
            return names;
        }

        // The result is a sequence of NUL-terminated names, ending with an
        // extra NUL. Walk it and keep the ones shaped like "COM<number>".
        int start = 0;
        for (int i = 0; i < written; i++)
        {
            if (buffer[i] != '\0') continue;

            int length = i - start;
            if (length > 3)
            {
                var name = new string(buffer, start, length);
                if (name.StartsWith("COM", StringComparison.Ordinal) &&
                    int.TryParse(name.AsSpan(3), out _))
                {
                    names.Add(name);
                }
            }
            start = i + 1;
            if (start < written && buffer[start] == '\0') break; // end of list
        }

        names.Sort((a, b) => int.Parse(a.AsSpan(3)).CompareTo(int.Parse(b.AsSpan(3))));
        return names;
    }

    /// <summary>
    /// Opens <paramref name="portName"/> (e.g. "COM3") for writing.
    /// </summary>
    /// <returns>True on success; on failure see <see cref="LastError"/>.</returns>
    public bool Open(string portName, int baud = DefaultBaud)
    {
        Close();
        LastError = null;

        // The \\.\ prefix is required for COM10 and above, and harmless below.
        string path = @"\\.\" + portName;

        SafeFileHandle handle = SerialNative.CreateFile(
            path,
            SerialNative.GENERIC_READ | SerialNative.GENERIC_WRITE,
            0,                       // no sharing — a serial port is exclusive
            IntPtr.Zero,
            SerialNative.OPEN_EXISTING,
            SerialNative.FILE_ATTRIBUTE_NORMAL,
            IntPtr.Zero);

        if (handle.IsInvalid)
        {
            LastError = Describe(portName, Marshal.GetLastWin32Error());
            handle.Dispose();
            return false;
        }

        if (!Configure(handle, baud))
        {
            handle.Dispose();
            return false;
        }

        _handle = handle;
        PortName = portName;
        FramesSent = 0;
        return true;
    }

    /// <summary>Applies 8N1 at <paramref name="baud"/> plus write timeouts.</summary>
    private bool Configure(SafeFileHandle handle, int baud)
    {
        var dcb = new SerialNative.DCB { DCBlength = (uint)Marshal.SizeOf<SerialNative.DCB>() };
        if (!SerialNative.GetCommState(handle, ref dcb))
        {
            LastError = $"GetCommState failed: {new Win32Exception(Marshal.GetLastWin32Error()).Message}";
            return false;
        }

        dcb.BaudRate = (uint)baud;
        dcb.ByteSize = 8;
        dcb.Parity = SerialNative.NOPARITY;
        dcb.StopBits = SerialNative.ONESTOPBIT;

        // Binary mode (mandatory on Windows), DTR/RTS asserted, no flow control
        // — the ST-Link VCP ignores handshake lines but some adapters need DTR.
        dcb.Flags = SerialNative.FLAG_BINARY |
                    SerialNative.FLAG_DTR_ENABLE |
                    SerialNative.FLAG_RTS_ENABLE;

        if (!SerialNative.SetCommState(handle, ref dcb))
        {
            LastError = $"SetCommState failed: {new Win32Exception(Marshal.GetLastWin32Error()).Message}";
            return false;
        }

        // Bound every write so an unplugged adapter cannot stall the caller.
        var timeouts = new SerialNative.COMMTIMEOUTS
        {
            ReadIntervalTimeout = 0,
            ReadTotalTimeoutMultiplier = 0,
            ReadTotalTimeoutConstant = 0,
            WriteTotalTimeoutMultiplier = 0,
            WriteTotalTimeoutConstant = 100,   // ms
        };
        if (!SerialNative.SetCommTimeouts(handle, ref timeouts))
        {
            LastError = $"SetCommTimeouts failed: {new Win32Exception(Marshal.GetLastWin32Error()).Message}";
            return false;
        }

        return true;
    }

    /// <summary>
    /// Writes one frame. On failure the port is closed so the UI can show the
    /// link as down and the user can reconnect.
    /// </summary>
    /// <returns>True if every byte was written.</returns>
    public bool Write(byte[] data)
    {
        if (_handle is null || _handle.IsInvalid || _handle.IsClosed)
        {
            return false;
        }

        if (!SerialNative.WriteFile(_handle, data, (uint)data.Length, out uint written, IntPtr.Zero)
            || written != data.Length)
        {
            LastError = $"Write failed: {new Win32Exception(Marshal.GetLastWin32Error()).Message}";
            Close();                       // treat as a disconnect
            return false;
        }

        FramesSent++;
        return true;
    }

    /// <summary>Closes the port if open. Safe to call repeatedly.</summary>
    public void Close()
    {
        _handle?.Dispose();
        _handle = null;
        PortName = null;
    }

    public void Dispose() => Close();

    /// <summary>Turns a Win32 error into something a user can act on.</summary>
    private static string Describe(string portName, int error) => error switch
    {
        2 => $"{portName} not found — is the board plugged in?",
        5 => $"{portName} is in use — close any other terminal (PuTTY, IDE monitor).",
        _ => $"Could not open {portName}: {new Win32Exception(error).Message}",
    };
}

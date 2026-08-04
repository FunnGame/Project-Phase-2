using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;

namespace Station.ControlApp.Transport;

/// <summary>
/// Raw P/Invoke bindings for the Win32 serial API. This is the only file that
/// touches the native layer; the rest of the app works through
/// <see cref="SerialLink"/>.
///
/// Using Win32 directly (rather than the System.IO.Ports NuGet package) keeps
/// the app dependency-free, matching how Input/XInputNative.cs handles the
/// controller.
/// </summary>
internal static class SerialNative
{
    private const string Kernel32 = "kernel32.dll";

    // ---- CreateFile ---------------------------------------------------------
    public const uint GENERIC_READ = 0x80000000;
    public const uint GENERIC_WRITE = 0x40000000;
    public const uint OPEN_EXISTING = 3;
    public const uint FILE_ATTRIBUTE_NORMAL = 0x80;

    [DllImport(Kernel32, SetLastError = true, CharSet = CharSet.Unicode, EntryPoint = "CreateFileW")]
    public static extern SafeFileHandle CreateFile(
        string lpFileName, uint dwDesiredAccess, uint dwShareMode,
        IntPtr lpSecurityAttributes, uint dwCreationDisposition,
        uint dwFlagsAndAttributes, IntPtr hTemplateFile);

    [DllImport(Kernel32, SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool WriteFile(
        SafeFileHandle hFile, byte[] lpBuffer, uint nNumberOfBytesToWrite,
        out uint lpNumberOfBytesWritten, IntPtr lpOverlapped);

    [DllImport(Kernel32, SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool FlushFileBuffers(SafeFileHandle hFile);

    // ---- Port configuration -------------------------------------------------

    /// <summary>
    /// Win32 DCB. The native struct packs 16 single/double-bit flags into one
    /// DWORD; they are exposed here as <see cref="Flags"/> with the bit
    /// positions defined below.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct DCB
    {
        public uint DCBlength;
        public uint BaudRate;
        public uint Flags;          // packed bitfield — see FLAG_* constants
        public ushort wReserved;
        public ushort XonLim;
        public ushort XoffLim;
        public byte ByteSize;
        public byte Parity;         // 0 = none
        public byte StopBits;       // 0 = one stop bit
        public sbyte XonChar;
        public sbyte XoffChar;
        public sbyte ErrorChar;
        public sbyte EofChar;
        public sbyte EvtChar;
        public ushort wReserved1;
    }

    /* Bit positions inside DCB.Flags. */
    public const uint FLAG_BINARY = 0x0001;        // fBinary   (bit 0) - must be set
    public const uint FLAG_DTR_ENABLE = 0x0010;    // fDtrControl = 1 (bits 4..5)
    public const uint FLAG_RTS_ENABLE = 0x1000;    // fRtsControl = 1 (bits 12..13)

    public const byte NOPARITY = 0;
    public const byte ONESTOPBIT = 0;

    [StructLayout(LayoutKind.Sequential)]
    public struct COMMTIMEOUTS
    {
        public uint ReadIntervalTimeout;
        public uint ReadTotalTimeoutMultiplier;
        public uint ReadTotalTimeoutConstant;
        public uint WriteTotalTimeoutMultiplier;
        public uint WriteTotalTimeoutConstant;
    }

    [DllImport(Kernel32, SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool GetCommState(SafeFileHandle hFile, ref DCB lpDCB);

    [DllImport(Kernel32, SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool SetCommState(SafeFileHandle hFile, ref DCB lpDCB);

    [DllImport(Kernel32, SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool SetCommTimeouts(SafeFileHandle hFile, ref COMMTIMEOUTS lpCommTimeouts);

    // ---- Port enumeration ---------------------------------------------------

    /// <summary>
    /// With a null device name this returns every DOS device as a
    /// double-NUL-terminated list, which we filter for COM ports. Avoids both
    /// System.IO.Ports and a registry dependency.
    /// </summary>
    [DllImport(Kernel32, SetLastError = true, CharSet = CharSet.Unicode, EntryPoint = "QueryDosDeviceW")]
    public static extern uint QueryDosDevice(string? lpDeviceName, char[] lpTargetPath, uint ucchMax);
}

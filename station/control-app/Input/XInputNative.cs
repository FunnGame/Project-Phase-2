using System.Runtime.InteropServices;

namespace Station.ControlApp.Input;

/// <summary>
/// Raw P/Invoke bindings for the Windows XInput API (Xbox controllers).
/// This is the only file that touches the native layer; everything else in the
/// app works through <see cref="Gamepad"/> / <see cref="GamepadState"/>.
/// </summary>
internal static class XInputNative
{
    // xinput1_4.dll ships with Windows 8 and later (Windows 10/11 included).
    private const string Dll = "xinput1_4.dll";

    /// <summary>Mirror of the native XINPUT_GAMEPAD structure.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct XInputGamepad
    {
        public ushort wButtons;      // GamepadButtons bit field
        public byte bLeftTrigger;    // 0..255
        public byte bRightTrigger;   // 0..255
        public short sThumbLX;       // -32768..32767
        public short sThumbLY;
        public short sThumbRX;
        public short sThumbRY;
    }

    /// <summary>Mirror of the native XINPUT_STATE structure.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct XInputState
    {
        public uint dwPacketNumber;  // increments only when the state changes
        public XInputGamepad Gamepad;
    }

    /// <summary>
    /// Reads the current state of controller <paramref name="dwUserIndex"/> (0..3).
    /// Returns <see cref="ERROR_SUCCESS"/> or <see cref="ERROR_DEVICE_NOT_CONNECTED"/>.
    /// </summary>
    [DllImport(Dll)]
    public static extern uint XInputGetState(uint dwUserIndex, ref XInputState pState);

    public const uint ERROR_SUCCESS = 0x0000;
    public const uint ERROR_DEVICE_NOT_CONNECTED = 0x048F;
}

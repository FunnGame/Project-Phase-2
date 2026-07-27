namespace Station.ControlApp.Input;

/// <summary>
/// Friendly, UI-agnostic wrapper over <see cref="XInputNative"/>. Poll it on a
/// timer to get an immutable <see cref="GamepadState"/> snapshot. Kept free of
/// any WPF dependency so it can be reused by a console tool or the future
/// PC -> F446RE control-frame sender.
/// </summary>
public sealed class Gamepad
{
    /// <summary>XInput supports up to four controllers (indices 0..3).</summary>
    public const uint MaxControllers = 4;

    /// <summary>Zero-based controller index this instance reads.</summary>
    public uint Index { get; }

    public Gamepad(uint index = 0)
    {
        if (index >= MaxControllers)
            throw new ArgumentOutOfRangeException(nameof(index), "Valid controller indices are 0..3.");
        Index = index;
    }

    /// <summary>
    /// Reads the controller now. Never throws for a missing controller — it
    /// returns <see cref="GamepadState.Disconnected"/> instead.
    /// </summary>
    public GamepadState Poll()
    {
        var native = new XInputNative.XInputState();
        uint result = XInputNative.XInputGetState(Index, ref native);

        if (result != XInputNative.ERROR_SUCCESS)
            return GamepadState.Disconnected;

        ref readonly var g = ref native.Gamepad;
        return new GamepadState(
            isConnected: true,
            packetNumber: native.dwPacketNumber,
            buttons: (GamepadButtons)g.wButtons,
            lx: g.sThumbLX, ly: g.sThumbLY,
            rx: g.sThumbRX, ry: g.sThumbRY,
            lt: g.bLeftTrigger, rt: g.bRightTrigger);
    }

    /// <summary>Scans indices 0..3 and returns the first connected one, or null.</summary>
    public static uint? FindFirstConnected()
    {
        for (uint i = 0; i < MaxControllers; i++)
        {
            var native = new XInputNative.XInputState();
            if (XInputNative.XInputGetState(i, ref native) == XInputNative.ERROR_SUCCESS)
                return i;
        }
        return null;
    }
}

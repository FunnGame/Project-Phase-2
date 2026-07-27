namespace Station.ControlApp.Input;

/// <summary>
/// An immutable snapshot of an Xbox controller at one poll. Raw values match
/// XInput ranges; the normalised properties (-1..1 for sticks, 0..1 for
/// triggers) are what the UI and control mapping consume.
/// </summary>
public readonly struct GamepadState
{
    /// <summary>True if a controller was connected at this poll.</summary>
    public bool IsConnected { get; }

    /// <summary>XInput packet counter; unchanged between identical reads.</summary>
    public uint PacketNumber { get; }

    public GamepadButtons Buttons { get; }

    // Raw axes (XInput native ranges).
    public short RawLeftStickX { get; }
    public short RawLeftStickY { get; }
    public short RawRightStickX { get; }
    public short RawRightStickY { get; }
    public byte RawLeftTrigger { get; }
    public byte RawRightTrigger { get; }

    public GamepadState(bool isConnected, uint packetNumber, GamepadButtons buttons,
                        short lx, short ly, short rx, short ry, byte lt, byte rt)
    {
        IsConnected = isConnected;
        PacketNumber = packetNumber;
        Buttons = buttons;
        RawLeftStickX = lx;
        RawLeftStickY = ly;
        RawRightStickX = rx;
        RawRightStickY = ry;
        RawLeftTrigger = lt;
        RawRightTrigger = rt;
    }

    /// <summary>A disconnected snapshot (all inputs neutral).</summary>
    public static GamepadState Disconnected =>
        new(false, 0, GamepadButtons.None, 0, 0, 0, 0, 0, 0);

    // Normalised axes ---------------------------------------------------------

    /// <summary>Left stick X in -1..1 (left..right).</summary>
    public float LeftStickX => Normalize(RawLeftStickX);
    /// <summary>Left stick Y in -1..1 (down..up).</summary>
    public float LeftStickY => Normalize(RawLeftStickY);
    /// <summary>Right stick X in -1..1.</summary>
    public float RightStickX => Normalize(RawRightStickX);
    /// <summary>Right stick Y in -1..1.</summary>
    public float RightStickY => Normalize(RawRightStickY);
    /// <summary>Left trigger in 0..1.</summary>
    public float LeftTrigger => RawLeftTrigger / 255f;
    /// <summary>Right trigger in 0..1.</summary>
    public float RightTrigger => RawRightTrigger / 255f;

    /// <summary>True if every button in <paramref name="button"/> is pressed.</summary>
    public bool IsPressed(GamepadButtons button) => (Buttons & button) == button;

    private static float Normalize(short value)
    {
        // The stick range is asymmetric (-32768..32767); scale each side to 1.0.
        return value < 0 ? value / 32768f : value / 32767f;
    }
}

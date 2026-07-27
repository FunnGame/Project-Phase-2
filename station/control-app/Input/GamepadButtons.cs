namespace Station.ControlApp.Input;

/// <summary>
/// Digital buttons on an Xbox controller. Values are the XInput
/// <c>XINPUT_GAMEPAD_*</c> bit masks, so the raw <c>wButtons</c> field maps
/// straight onto this flags enum.
/// </summary>
[Flags]
public enum GamepadButtons : ushort
{
    None          = 0x0000,
    DPadUp        = 0x0001,
    DPadDown      = 0x0002,
    DPadLeft      = 0x0004,
    DPadRight     = 0x0008,
    Start         = 0x0010,
    Back          = 0x0020,
    LeftThumb     = 0x0040, // left stick click (L3)
    RightThumb    = 0x0080, // right stick click (R3)
    LeftShoulder  = 0x0100, // LB
    RightShoulder = 0x0200, // RB
    A             = 0x1000,
    B             = 0x2000,
    X             = 0x4000,
    Y             = 0x8000,
}

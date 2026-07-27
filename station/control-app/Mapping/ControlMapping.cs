using Station.ControlApp.Input;

namespace Station.ControlApp.Mapping;

/// <summary>
/// The intent extracted from the controller for driving the car. Values are
/// normalised and deadzone-corrected, ready to be scaled into whatever units
/// the car expects (PWM duty, steering angle, ...) when the radio link is added.
/// </summary>
/// <param name="Steering">-1 (full left) .. +1 (full right).</param>
/// <param name="Throttle">0 .. 1 (drive magnitude).</param>
/// <param name="Brake">0 .. 1 (brake).</param>
/// <param name="Reverse">True while the reverse-direction button is held.</param>
/// <param name="Armed">True when the car is enabled (armed). When false the car
/// must ignore drive commands and stay stopped.</param>
public readonly record struct CarControl(
    float Steering, float Throttle, float Brake, bool Reverse, bool Armed);

/// <summary>
/// Translates a raw <see cref="GamepadState"/> into a <see cref="CarControl"/>.
///
/// Mapping (analogue driving; discrete buttons):
///   Steering  = left stick X
///   Throttle  = right trigger (RT)
///   Brake     = left trigger  (LT)
///   Reverse   = left bumper  (LB), held   — flips drive direction
///   Arm/enable= right bumper (RB), toggle — see <see cref="ArmButton"/>
///
/// This is the single place to change how the pad drives the car. Note the arm
/// state is a latched toggle, so it is tracked by the caller (the ViewModel)
/// and passed in here — this method stays a pure, stateless mapper.
/// </summary>
public static class ControlMapping
{
    // XInput's recommended left-thumb deadzone, as a normalised fraction.
    private const float SteeringDeadzone = 7849f / 32767f;

    /// <summary>Button that signals reverse direction while held.</summary>
    public const GamepadButtons ReverseButton = GamepadButtons.LeftShoulder;

    /// <summary>Button that toggles the car enable (arm) state.</summary>
    public const GamepadButtons ArmButton = GamepadButtons.RightShoulder;

    public static CarControl FromState(in GamepadState s, bool armed)
    {
        if (!s.IsConnected)
            return new CarControl(0f, 0f, 0f, false, false);

        float steering = ApplyDeadzone(s.LeftStickX, SteeringDeadzone);
        float throttle = s.RightTrigger;
        float brake = s.LeftTrigger;
        bool reverse = s.IsPressed(ReverseButton);

        return new CarControl(steering, throttle, brake, reverse, armed);
    }

    /// <summary>
    /// Removes jitter near centre, then rescales the remaining travel to the
    /// full -1..1 range so control still reaches the extremes.
    /// </summary>
    private static float ApplyDeadzone(float value, float deadzone)
    {
        float magnitude = MathF.Abs(value);
        if (magnitude < deadzone)
            return 0f;

        float sign = MathF.Sign(value);
        float scaled = (magnitude - deadzone) / (1f - deadzone);
        return sign * Math.Clamp(scaled, 0f, 1f);
    }
}

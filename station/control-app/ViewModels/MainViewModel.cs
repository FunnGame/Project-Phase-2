using System.Collections.ObjectModel;
using System.Windows.Threading;
using Station.ControlApp.Input;
using Station.ControlApp.Mapping;
using Station.ControlApp.Transport;

namespace Station.ControlApp.ViewModels;

/// <summary>
/// Drives the main window: polls the Xbox controller at ~60 Hz on the UI
/// thread and exposes bindable properties for the visualisation (stick dots,
/// trigger bars, button lights) plus the derived car-control intent.
/// </summary>
public sealed class MainViewModel : ObservableObject
{
    // Thumbstick display geometry (device-independent pixels).
    public const double StickArea = 140;
    public const double StickRadius = 60;
    public const double DotSize = 20;
    public const double TriggerBarHeight = 120;

    private static readonly TimeSpan PollInterval = TimeSpan.FromMilliseconds(16); // ~60 Hz

    private readonly DispatcherTimer _timer;
    private Gamepad _pad = new(0);
    private readonly ControlFrameSerializer _frame = new();

    // Latched arm state + previous arm-button level for rising-edge detection.
    private bool _armed;
    private bool _prevArmButton;
    private bool _reverseLatched;
    private bool _prevReverseButton;

    public MainViewModel()
    {
        _timer = new DispatcherTimer(DispatcherPriority.Render) { Interval = PollInterval };
        _timer.Tick += (_, _) => Update();

        ConnectCommand = new RelayCommand(ToggleConnection);
        RefreshPortsCommand = new RelayCommand(RefreshPorts);
        RefreshPorts();
    }

    /// <summary>Begin polling. Call once the window is loaded.</summary>
    public void Start() => _timer.Start();

    /// <summary>Stop polling and close the serial link. Call on window close.</summary>
    public void Stop()
    {
        _timer.Stop();
        _link.Dispose();
    }

    /* ----- Serial transport ------------------------------------------------ */

    private readonly SerialLink _link = new();

    /// <summary>COM ports currently present on the machine.</summary>
    public ObservableCollection<string> AvailablePorts { get; } = new();

    public RelayCommand ConnectCommand { get; }
    public RelayCommand RefreshPortsCommand { get; }

    /// <summary>Re-scan for COM ports, keeping the current selection if it survives.</summary>
    private void RefreshPorts()
    {
        string? previous = SelectedPort;
        AvailablePorts.Clear();
        foreach (string p in SerialLink.GetPortNames())
        {
            AvailablePorts.Add(p);
        }
        SelectedPort = (previous is not null && AvailablePorts.Contains(previous))
            ? previous
            : AvailablePorts.FirstOrDefault();
    }

    private void ToggleConnection()
    {
        if (_link.IsOpen)
        {
            _link.Close();
            LinkStatusText = "Disconnected";
        }
        else if (SelectedPort is not null && _link.Open(SelectedPort))
        {
            LinkStatusText = $"Sending on {SelectedPort} @ {SerialLink.DefaultBaud} 8N1";
        }
        else
        {
            LinkStatusText = _link.LastError ?? "No COM port selected";
        }

        IsLinkOpen = _link.IsOpen;
        ConnectButtonText = _link.IsOpen ? "Disconnect" : "Connect";
    }

    private void Update()
    {
        GamepadState s = _pad.Poll();

        // If nothing is on our index, look for a controller on another slot.
        if (!s.IsConnected)
        {
            uint? found = Gamepad.FindFirstConnected();
            if (found is uint idx && idx != _pad.Index)
            {
                _pad = new Gamepad(idx);
                s = _pad.Poll();
            }
        }

        ApplyState(s);
    }

    private void ApplyState(in GamepadState s)
    {
        IsConnected = s.IsConnected;
        StatusText = s.IsConnected
            ? $"Controller {_pad.Index} connected"
            : "No controller detected — plug in an Xbox pad";

        // Arm/enable: toggle on a rising edge of the arm button; force-disarm
        // whenever the controller is not connected (safety).
        UpdateArmState(s);
        UpdateReverseState(s);

        // Sticks: numeric read-out + dot position on the canvas.
        LeftStickText = $"X {s.RawLeftStickX,6}   Y {s.RawLeftStickY,6}";
        RightStickText = $"X {s.RawRightStickX,6}   Y {s.RawRightStickY,6}";
        LeftDotX = DotX(s.LeftStickX);
        LeftDotY = DotY(s.LeftStickY);
        RightDotX = DotX(s.RightStickX);
        RightDotY = DotY(s.RightStickY);

        // Triggers: fill bars + raw 0..255 read-out.
        LeftTriggerFill = s.LeftTrigger * TriggerBarHeight;
        RightTriggerFill = s.RightTrigger * TriggerBarHeight;
        LeftTriggerText = s.RawLeftTrigger.ToString();
        RightTriggerText = s.RawRightTrigger.ToString();

        // Buttons.
        A = s.IsPressed(GamepadButtons.A);
        B = s.IsPressed(GamepadButtons.B);
        X = s.IsPressed(GamepadButtons.X);
        Y = s.IsPressed(GamepadButtons.Y);
        LeftBumper = s.IsPressed(GamepadButtons.LeftShoulder);
        RightBumper = s.IsPressed(GamepadButtons.RightShoulder);
        LeftThumb = s.IsPressed(GamepadButtons.LeftThumb);
        RightThumb = s.IsPressed(GamepadButtons.RightThumb);
        Start_ = s.IsPressed(GamepadButtons.Start);
        Back = s.IsPressed(GamepadButtons.Back);
        DPadUp = s.IsPressed(GamepadButtons.DPadUp);
        DPadDown = s.IsPressed(GamepadButtons.DPadDown);
        DPadLeft = s.IsPressed(GamepadButtons.DPadLeft);
        DPadRight = s.IsPressed(GamepadButtons.DPadRight);

        // Derived car-control intent + the actual frame the radio link will send.
        CarControl c = ControlMapping.FromState(s, _armed, _reverseLatched);
        SteeringText = $"{c.Steering,+6:0.00}";
        ThrottleText = $"{c.Throttle,6:0.00}";
        BrakeText = $"{c.Brake,6:0.00}";
        Reverse = c.Reverse;

        byte[] frame = _frame.Build(c);
        FrameHexText = Convert.ToHexString(frame);

        /* Stream every tick (~60 Hz), even when nothing changed: the car's
         * failsafe treats silence as a lost link, so a steady rate is what
         * keeps it alive. */
        if (_link.IsOpen)
        {
            if (_link.Write(frame))
            {
                FramesSentText = _link.FramesSent.ToString();
            }
            else
            {
                /* Write() closes the port on failure — surface it and stop. */
                IsLinkOpen = false;
                ConnectButtonText = "Connect";
                LinkStatusText = _link.LastError ?? "Serial write failed";
            }
        }
    }

    private void UpdateArmState(in GamepadState s)
    {
        if (!s.IsConnected)
        {
            _armed = false;
            _prevArmButton = false;
        }
        else
        {
            bool pressed = s.IsPressed(ControlMapping.ArmButton);
            if (pressed && !_prevArmButton)   // rising edge -> toggle
                _armed = !_armed;
            _prevArmButton = pressed;
        }

        Armed = _armed;
        ArmStatusText = _armed ? "ARMED" : "DISARMED";
    }

    /// <summary>
    /// Latch the drive direction on the rising edge of the reverse button.
    ///
    /// Cleared on disconnect, exactly like the arm latch: a pad that drops out
    /// and comes back must not silently restore a direction the operator set
    /// minutes ago and has since forgotten about. Unlike arm, it is NOT cleared
    /// on disarm - reversing, stopping to reposition, then driving on again is
    /// a normal manoeuvre, and clearing it there would surprise more than it
    /// protects.
    /// </summary>
    private void UpdateReverseState(in GamepadState s)
    {
        if (!s.IsConnected)
        {
            _reverseLatched = false;
            _prevReverseButton = false;
            return;
        }

        bool pressed = s.IsPressed(ControlMapping.ReverseButton);
        if (pressed && !_prevReverseButton)   // rising edge -> toggle
            _reverseLatched = !_reverseLatched;
        _prevReverseButton = pressed;
    }

    private static double DotX(float normX) => StickArea / 2 + normX * StickRadius - DotSize / 2;
    // Screen Y grows downward; stick up (+Y) should move the dot up.
    private static double DotY(float normY) => StickArea / 2 - normY * StickRadius - DotSize / 2;

    // ----- Bindable properties ----------------------------------------------

    private bool _isConnected;
    public bool IsConnected { get => _isConnected; private set => SetProperty(ref _isConnected, value); }

    private string _statusText = "Starting…";
    public string StatusText { get => _statusText; private set => SetProperty(ref _statusText, value); }

    private string _leftStickText = "";
    public string LeftStickText { get => _leftStickText; private set => SetProperty(ref _leftStickText, value); }

    private string _rightStickText = "";
    public string RightStickText { get => _rightStickText; private set => SetProperty(ref _rightStickText, value); }

    private double _leftDotX, _leftDotY, _rightDotX, _rightDotY;
    public double LeftDotX { get => _leftDotX; private set => SetProperty(ref _leftDotX, value); }
    public double LeftDotY { get => _leftDotY; private set => SetProperty(ref _leftDotY, value); }
    public double RightDotX { get => _rightDotX; private set => SetProperty(ref _rightDotX, value); }
    public double RightDotY { get => _rightDotY; private set => SetProperty(ref _rightDotY, value); }

    private double _leftTriggerFill, _rightTriggerFill;
    public double LeftTriggerFill { get => _leftTriggerFill; private set => SetProperty(ref _leftTriggerFill, value); }
    public double RightTriggerFill { get => _rightTriggerFill; private set => SetProperty(ref _rightTriggerFill, value); }

    private string _leftTriggerText = "0", _rightTriggerText = "0";
    public string LeftTriggerText { get => _leftTriggerText; private set => SetProperty(ref _leftTriggerText, value); }
    public string RightTriggerText { get => _rightTriggerText; private set => SetProperty(ref _rightTriggerText, value); }

    private bool _a, _b, _x, _y, _lb, _rb, _lThumb, _rThumb, _start, _back;
    private bool _up, _down, _left, _right;
    public bool A { get => _a; private set => SetProperty(ref _a, value); }
    public bool B { get => _b; private set => SetProperty(ref _b, value); }
    public bool X { get => _x; private set => SetProperty(ref _x, value); }
    public bool Y { get => _y; private set => SetProperty(ref _y, value); }
    public bool LeftBumper { get => _lb; private set => SetProperty(ref _lb, value); }
    public bool RightBumper { get => _rb; private set => SetProperty(ref _rb, value); }
    public bool LeftThumb { get => _lThumb; private set => SetProperty(ref _lThumb, value); }
    public bool RightThumb { get => _rThumb; private set => SetProperty(ref _rThumb, value); }
    public bool Start_ { get => _start; private set => SetProperty(ref _start, value); }
    public bool Back { get => _back; private set => SetProperty(ref _back, value); }
    public bool DPadUp { get => _up; private set => SetProperty(ref _up, value); }
    public bool DPadDown { get => _down; private set => SetProperty(ref _down, value); }
    public bool DPadLeft { get => _left; private set => SetProperty(ref _left, value); }
    public bool DPadRight { get => _right; private set => SetProperty(ref _right, value); }

    private string _steeringText = "0.00", _throttleText = "0.00", _brakeText = "0.00";
    public string SteeringText { get => _steeringText; private set => SetProperty(ref _steeringText, value); }
    public string ThrottleText { get => _throttleText; private set => SetProperty(ref _throttleText, value); }
    public string BrakeText { get => _brakeText; private set => SetProperty(ref _brakeText, value); }

    private bool _reverse;
    public bool Reverse { get => _reverse; private set => SetProperty(ref _reverse, value); }

    private bool _armedProp;
    public bool Armed { get => _armedProp; private set => SetProperty(ref _armedProp, value); }

    private string _armStatusText = "DISARMED";
    public string ArmStatusText { get => _armStatusText; private set => SetProperty(ref _armStatusText, value); }

    private string _frameHexText = "";
    public string FrameHexText { get => _frameHexText; private set => SetProperty(ref _frameHexText, value); }

    private string? _selectedPort;
    public string? SelectedPort { get => _selectedPort; set => SetProperty(ref _selectedPort, value); }

    private bool _isLinkOpen;
    public bool IsLinkOpen { get => _isLinkOpen; private set => SetProperty(ref _isLinkOpen, value); }

    private string _connectButtonText = "Connect";
    public string ConnectButtonText { get => _connectButtonText; private set => SetProperty(ref _connectButtonText, value); }

    private string _linkStatusText = "Not connected";
    public string LinkStatusText { get => _linkStatusText; private set => SetProperty(ref _linkStatusText, value); }

    private string _framesSentText = "0";
    public string FramesSentText { get => _framesSentText; private set => SetProperty(ref _framesSentText, value); }
}

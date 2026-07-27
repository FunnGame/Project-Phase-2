# Station Control App (Xbox controller)

A Windows desktop app (C# / WPF, .NET 10) that reads an **Xbox controller** via
**XInput** and visualises its input live. It is the operator interface for the
control station; the car link (PC → F446RE → nRF24) is **not wired yet** — the
footer already previews the control frame that link will send.

## Requirements

- Windows 10/11 (XInput ships with the OS — no drivers/NuGet packages needed)
- .NET 10 SDK
- An Xbox controller (wired, or wireless via the Xbox Wireless Adapter / Bluetooth)

## Build & run

```sh
cd station/control-app
dotnet build          # compile
dotnet run            # build + launch the window
```

## What you see

- **Thumbsticks** — dot position + raw X/Y (−32768..32767)
- **Triggers** — LT/RT fill bars + raw value (0..255)
- **Buttons / D-pad** — light up while held
- **Car control** (footer) — the derived driving intent:

| Action   | Input              |
|----------|--------------------|
| Steering | Left stick X       |
| Throttle | Right trigger (RT) |
| Brake    | Left trigger (LT)  |
| E-stop   | B button           |

## Structure

```
control-app/
├── XboxControllerApp.csproj
├── App.xaml (+.cs)              # app shell, theme resources
├── MainWindow.xaml (+.cs)       # visualiser window (thin; binds to the VM)
├── Input/                       # UI-agnostic controller layer
│   ├── XInputNative.cs          # P/Invoke to xinput1_4.dll (only native code)
│   ├── Gamepad.cs               # Poll() wrapper + connection scan
│   ├── GamepadState.cs          # immutable snapshot, normalised values
│   └── GamepadButtons.cs        # [Flags] enum of XInput button masks
├── ViewModels/
│   ├── ObservableObject.cs      # tiny INotifyPropertyChanged base
│   └── MainViewModel.cs         # 60 Hz poll loop + bindable state
├── Mapping/
│   └── ControlMapping.cs        # gamepad -> CarControl (steering/throttle/brake)
└── Converters/
    └── BoolToBrushConverter.cs  # lights indicators on button press
```

## Extending toward the car

When you add the radio link, serialise `ControlMapping.FromState(...)` (a
`CarControl`) into a packet and send it over the F446RE's USB virtual COM port.
`Input/` and `Mapping/` are deliberately WPF-free so they can be reused by that
sender without dragging in the UI.

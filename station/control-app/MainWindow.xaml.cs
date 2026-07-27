using System.Windows;
using Station.ControlApp.ViewModels;

namespace Station.ControlApp;

/// <summary>
/// The visualiser window. Kept intentionally thin: it owns a
/// <see cref="MainViewModel"/> and starts/stops its poll loop with the window
/// lifetime. All state and logic live in the ViewModel.
/// </summary>
public partial class MainWindow : Window
{
    private readonly MainViewModel _vm = new();

    public MainWindow()
    {
        InitializeComponent();
        DataContext = _vm;
        Loaded += (_, _) => _vm.Start();
        Closed += (_, _) => _vm.Stop();
    }
}

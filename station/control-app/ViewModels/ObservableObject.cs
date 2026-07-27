using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace Station.ControlApp.ViewModels;

/// <summary>
/// Minimal INotifyPropertyChanged base so the UI updates itself when the
/// ViewModel changes. Avoids pulling in a full MVVM framework for one window.
/// </summary>
public abstract class ObservableObject : INotifyPropertyChanged
{
    public event PropertyChangedEventHandler? PropertyChanged;

    protected void OnPropertyChanged([CallerMemberName] string? name = null)
        => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));

    /// <summary>Sets <paramref name="field"/> and raises change notification if it differs.</summary>
    protected bool SetProperty<T>(ref T field, T value, [CallerMemberName] string? name = null)
    {
        if (EqualityComparer<T>.Default.Equals(field, value))
            return false;
        field = value;
        OnPropertyChanged(name);
        return true;
    }
}

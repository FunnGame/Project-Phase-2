using System.Globalization;
using System.Windows.Data;
using System.Windows.Media;

namespace Station.ControlApp.Converters;

/// <summary>
/// Returns <see cref="OnBrush"/> when the bound value is <c>true</c>, otherwise
/// <see cref="OffBrush"/>. Used to light up button / D-pad indicators.
/// </summary>
public sealed class BoolToBrushConverter : IValueConverter
{
    public Brush OnBrush { get; set; } = Brushes.DeepSkyBlue;
    public Brush OffBrush { get; set; } = Brushes.Gray;

    public object Convert(object? value, Type targetType, object? parameter, CultureInfo culture)
        => value is true ? OnBrush : OffBrush;

    public object ConvertBack(object? value, Type targetType, object? parameter, CultureInfo culture)
        => throw new NotSupportedException();
}

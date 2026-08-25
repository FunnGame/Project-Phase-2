namespace Station.ControlApp.Mapping;

/// <summary>
/// Serialises a <see cref="CarControl"/> into the compact, fixed-size binary
/// frame sent to the car (PC -> F446RE -> nRF24). Layout (7 bytes, little-endian
/// scalars — all fields are single bytes so byte order is moot):
///
/// <code>
///   [0] magic     0xA5   frame start / sync
///   [1] seq       0..255 increments each frame (drop / duplicate detection)
///   [2] steering  int8   -100..+100  (left..right)
///   [3] throttle  uint8  0..100      (drive magnitude)
///   [4] brake     uint8  0..100
///   [5] buttons   uint8  bit0 = reverse, bit1 = armed; bits 2..7 reserved
///   [6] crc        uint8  CRC-8 (poly 0x07) over bytes [0..5]
/// </code>
///
/// The matching C struct on the STM32 side is <c>control_frame_t</c>. The car
/// must apply a failsafe (coast/brake) if no valid frame arrives for ~150 ms,
/// so the PC should send at a steady rate (e.g. 50 Hz) even when idle.
/// </summary>
public sealed class ControlFrameSerializer
{
    public const byte Magic = 0xA5;
    public const int FrameSize = 7;

    /// <summary>Button bit positions inside the frame's <c>buttons</c> byte.</summary>
    [Flags]
    public enum FrameButtons : byte
    {
        None    = 0x00,
        Reverse = 0x01,
        Armed   = 0x02,
    }

    private byte _seq;

    /// <summary>Sequence number that will be used for the next <see cref="Build"/>.</summary>
    public byte NextSequence => _seq;

    /// <summary>
    /// Builds the next frame from <paramref name="c"/> and advances the
    /// sequence counter. The returned array is exactly <see cref="FrameSize"/>
    /// bytes, ready to hand to the transport.
    /// </summary>  
    public byte[] Build(in CarControl c)
    {
        var frame = new byte[FrameSize];
        frame[0] = Magic;
        frame[1] = _seq;
        frame[2] = (byte)EncodeSigned(c.Steering);   // int8 as raw byte
        frame[3] = EncodeUnsigned(c.Throttle);
        frame[4] = EncodeUnsigned(c.Brake);

        FrameButtons buttons = FrameButtons.None;
        if (c.Reverse) buttons |= FrameButtons.Reverse;
        if (c.Armed)   buttons |= FrameButtons.Armed;
        frame[5] = (byte)buttons;

        frame[6] = Crc8(frame.AsSpan(0, FrameSize - 1));

        _seq++; // wraps at 255 -> 0
        return frame;
    }

    // -1..1  -> -100..+100
    private static sbyte EncodeSigned(float v)
        => (sbyte)Math.Clamp((int)MathF.Round(v * 100f), -100, 100);

    // 0..1   -> 0..100
    private static byte EncodeUnsigned(float v)
        => (byte)Math.Clamp((int)MathF.Round(v * 100f), 0, 100);

    /// <summary>
    /// CRC-8 (polynomial 0x07, init 0x00, no reflection) — cheap and adequate
    /// for a 6-byte payload. Must match the car-side implementation.
    /// </summary>
    public static byte Crc8(ReadOnlySpan<byte> data)
    {
        byte crc = 0x00;
        foreach (byte b in data)
        {
            crc ^= b;
            for (int i = 0; i < 8; i++)
                crc = (byte)((crc & 0x80) != 0 ? (crc << 1) ^ 0x07 : crc << 1);
        }
        return crc;
    }
}

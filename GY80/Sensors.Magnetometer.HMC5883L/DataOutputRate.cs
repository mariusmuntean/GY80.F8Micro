namespace GY80.Sensors.Magnetometer.HMC5883L
{
    /// <summary>
    /// All  selectable output rates in continuous measurement mode.All three channels shall be measured within a given output rate. Other output rates with maximum rate of 160Hz can be achieved in single measurement mode.
    /// </summary>
    public enum DataOutputRate
    {
        _0_75Hz = 0b000_000_00,
        _1_5Hz = 0b000_001_00,
        _3Hz = 0b000_010_00,
        _7_5Hz = 0b000_011_00,
        _15Hz = 0b000_100_00,
        _30Hz = 0b000_101_00,
        _75Hz = 0b000_110_00,
    }
}

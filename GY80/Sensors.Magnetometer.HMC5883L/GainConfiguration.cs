namespace GY80.Sensors.Magnetometer.HMC5883L
{
    public enum GainConfiguration
    {
        /// <summary>
        /// ± 0.88Ga
        /// </summary>
        _0_88 = 0b000_00000,

        /// <summary>
        /// ± 1.3Ga
        /// </summary>
        _1_30 = 0b001_00000,

        /// <summary>
        /// ± 1.9Ga
        /// </summary>
        _1_90 = 0b010_00000,

        /// <summary>
        /// ± 2.5Ga
        /// </summary>
        _2_50 = 0b011_00000,

        /// <summary>
        /// ± 4.0Ga
        /// </summary>
        _4_00 = 0b100_00000,

        /// <summary>
        /// ± 4.7Ga
        /// </summary>
        _4_70 = 0b101_00000,

        /// <summary>
        /// ± 5.6Ga
        /// </summary>
        _5_60 = 0b110_00000,

        /// <summary>
        /// ± 8.1Ga
        /// </summary>
        _8_10 = 0b111_00000,
    }
}

namespace GY80.Sensors.Magnetometer.HMC5883L
{
    /// <summary>
    /// MODES OF OPERATION
    /// <para>This  device  has  several operating modes  whose  primary  purpose  is  power  managementand  is  controlled  by  the  Mode Register.</para>
    /// </summary>
    public enum MeasurementMode
    {
        /// <summary>
        /// Normal measurement configuration (Default).  In normal measurement configuration the device follows normal measurement flow. The positive and negative pins of the resistive loadare left floating and high impedance.
        /// </summary>
        Normal = 0b000000_00,

        /// <summary>
        /// Positive bias configurationfor X, Y,and Zaxes. In this configuration, a positive currentis forced across the resistive load for all three axes.
        /// </summary>
        PositiveBias = 0b000000_01,

        /// <summary>
        /// Negative bias configurationfor X, Yand Zaxes. In this configuration, a negative currentis forced across the resistive load for all three axes.
        /// </summary>
        NegativeBias = 0b000000_10
    }
}

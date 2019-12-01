namespace GY80.Sensors.Magnetometer.HMC5883L
{
        /// <summary>
        /// Select numberof samples averaged (1 to 8) per measurement output.
        /// </summary>
        public enum SamplesToAverage
        {
            One = 0b00000000,
            Two = 0b00100000,
            Four = 0b01000000,
            Eight = 0b01100000
        }
}

namespace GY80.Sensors.Magnetometer.HMC5883L
{
        public enum OperatingMode
        {
            /// <summary>
            /// Continuous-MeasurementMode. In continuous-measurementmode, the device continuously performs measurements andplaces the result in the data register.  RDY goes high when new data is placed in all three registers.  After a power-on or a write to the mode or configuration register, the first measurement set is available from all three data output registers after a period of 2/fDOand subsequent measurements are available at a frequency of fDO, where fDOis the frequency of data output.
            /// </summary>
            ContinuousMeasurement = 0b000000_00,

            /// <summary>
            /// Single-MeasurementMode(Default). When single-measurementmode is selected, device performs a single measurement, sets RDY high and returned to idle mode.  Mode register returns to idle mode bit values.  The measurement remains in the data output register and RDY remains high until the data output register is read or another measurementis performed.
            /// </summary>
            SingleMeasurement = 0b000000_01,

            /// <summary>
            /// Idle Mode.  Device is placed in idle mode.
            /// </summary>
            Idle = 0b000000_10,
        }
}

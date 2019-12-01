using System;
using System.Threading;
using Meadow.Foundation.Sensors;
using Meadow.Foundation.Spatial;
using Meadow.Hardware;

namespace GY80
{
    public class HMC5883L
    {
        private int _updateInterval;
        private byte CRA = 0x10;
        private byte CRB = 0x20;
        private byte MR = 0x02;

        private byte DXRA = 0x03;
        private byte DXRB = 0x04;
        private byte DYRA = 0x05;
        private byte DYRB = 0x06;
        private byte DZRA = 0x07;
        private byte DZRB = 0x08;


        II2cPeripheral _device;
        private short _lastX = 0;
        private short _lastZ = 0;
        private short _lastY = 0;
        private short _currentX, _currentY, _currentZ = 0;
        private float Threshold = 0.1f;

        public enum SamplesToAverage
        {
            One = 0b00000000,
            Two = 0b00100000,
            Four = 0b01000000,
            Eight = 0b01100000
        }

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

        public enum MeasurementMode
        {
            /// <summary>
            /// Normal measurement configuration (Default).  In normal measurement configuration the device follows normal measurement flow.  The positive and negative pins of the resistive loadare left floating and high impedance.
            /// </summary>
            Normal = 0b000000_00,

            /// <summary>
            /// Positive bias configurationfor X, Y,and Zaxes.In thisconfiguration, a positive currentis forced across the resistive load for all threeaxes.
            /// </summary>
            PositiveBias = 0b000000_01,

            /// <summary>
            /// Negative bias configurationfor X, Yand Zaxes.In thisconfiguration, a negative currentis forced across the resistive load for all threeaxes.
            /// </summary>
            NegativeBias = 0b000000_10
        }

        public enum GainConfiguration
        {
            _0_88 = 0b000_00000,
            _1_30 = 0b001_00000,
            _1_90 = 0b010_00000,
            _2_50 = 0b011_00000,
            _4_00 = 0b100_00000,
            _4_70 = 0b101_00000,
            _5_60 = 0b110_00000,
            _8_10 = 0b111_00000,
        }

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

        public HMC5883L(II2cBus i2CBus,
            byte address = 0x1E,
            SamplesToAverage samplesToAverage = SamplesToAverage.One,
            DataOutputRate dataOutputRate = DataOutputRate._15Hz,
            MeasurementMode measurementMode = MeasurementMode.Normal,

            GainConfiguration gainConfiguration = GainConfiguration._1_30,

            OperatingMode operatingMode = OperatingMode.SingleMeasurement
            )
        {

            Mode = operatingMode;

            _device = new I2cPeripheral(i2CBus, address);

            // Config - Register A
            byte configRegisterAData = 0b0000_0000;
            configRegisterAData |= (byte)samplesToAverage;
            configRegisterAData |= (byte)dataOutputRate;
            configRegisterAData |= (byte)measurementMode;
            _device.WriteRegister(CRA, configRegisterAData);
            Console.WriteLine($"CRA: {Convert.ToString(configRegisterAData, 2).PadLeft(8, '0')}");

            // Config - Register A
            byte configRegisterBData = 0b0000_0000;
            configRegisterBData |= (byte)gainConfiguration;
            _device.WriteRegister(CRB, configRegisterBData);
            Console.WriteLine($"CRB: {Convert.ToString(configRegisterBData, 2).PadLeft(8, '0')}");

            // Select Mode
            byte modeRegisterData = 0b00000000;
            modeRegisterData |= (byte)operatingMode;
            _device.WriteRegister(MR, modeRegisterData);
            Console.WriteLine($"MR: {Convert.ToString(modeRegisterData, 2).PadLeft(8, '0')}");
        }

        public OperatingMode Mode { get; private set; }

        /// <summary>
        /// Reads the current values of the magnetic field strength
        /// </summary>
        /// <returns>A <see cref="Vector"/> with the values of the magnetic field strenth on three axes, expressed in Gaus</returns>
        public Vector Read()
        {
            // In case the sensor is in single measurement mode it is necessary to update the Mode Register, wait at least 6 ms and only then read the data.
            if (Mode == OperatingMode.SingleMeasurement)
            {
                var currentModeRegisterData = (byte)OperatingMode.SingleMeasurement;
                _device.WriteRegister(MR, currentModeRegisterData);

                Thread.Sleep(7);
            }

            var data = _device.ReadRegisters(DXRA, 6);
            _currentX = (short)((data[0] << 8) + data[1]);
            _currentY = (short)((data[2] << 8) + data[3]);
            _currentZ = (short)((data[4] << 8) + data[5]);

            return new Vector(_currentX, _currentY, _currentZ);
        }

        /// <summary>
        ///     Event to be raised when the magnetic field change is greater than +/- Threshold.
        /// </summary>
        public event SensorVectorEventHandler FieldStrengthChanged = delegate { };

        public void StartInternalPolling(TimeSpan pollingPeriod)
        {
            _updateInterval = (int)pollingPeriod.TotalMilliseconds;

            Thread t = new Thread(() =>
            {
                while (true)
                {
                    var newestValues = Read();
                    //Console.WriteLine($"Newest: {newestValues.X} {newestValues.Y} {newestValues.Z}");
                    if (
                    Math.Abs(newestValues.X - _lastX) > Threshold
                    || Math.Abs(newestValues.Y - _lastY) > Threshold
                    || Math.Abs(newestValues.Z - _lastZ) > Threshold
                    )
                    {
                        FieldStrengthChanged?.Invoke(this, new SensorVectorEventArgs(new Vector(_lastX, _lastY, _lastZ), new Vector(_currentX, _currentY, _currentZ)));

                        _lastX = _currentX;
                        _lastY = _currentY;
                        _lastZ = _currentZ;
                    }

                    Thread.Sleep(_updateInterval);
                }
            });
            t.Start();
        }
    }
}

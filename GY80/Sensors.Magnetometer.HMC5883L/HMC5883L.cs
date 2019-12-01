using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using Meadow.Foundation.Sensors;
using Meadow.Foundation.Spatial;
using Meadow.Hardware;

namespace GY80.Sensors.Magnetometer.HMC5883L
{
    /// <summary>
    /// Honeywell 3-Axis Digital Compass ICHMC5883L
    /// 
    /// <para>
    ///     Datasheet: https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
    /// </para>
    /// </summary>
    public partial class HMC5883L
    {
        private byte CRA = 0x10;
        private byte CRB = 0x20;
        private byte MR = 0x02;

        private byte DXRA = 0x03;
        private byte DXRB = 0x04;
        private byte DYRA = 0x05;
        private byte DYRB = 0x06;
        private byte DZRA = 0x07;
        private byte DZRB = 0x08;

        private float _lastX, _lastY, _lastZ = 0.0f;
        private float _currentX, _currentY, _currentZ = 0.0f;

        /// <summary>
        /// See datasheet page 13
        /// </summary>
        private Dictionary<GainConfiguration, float> _gainToDividingFactorMap = new Dictionary<GainConfiguration, float>()
        {
            {GainConfiguration._0_88 , 1370.0f},
            {GainConfiguration._1_30 , 1090.0f},
            {GainConfiguration._1_90 , 820.0f},
            {GainConfiguration._2_50 , 660.0f},
            {GainConfiguration._4_00 , 440.0f},
            {GainConfiguration._4_70 , 390.0f},
            {GainConfiguration._5_60 , 330.0f},
            {GainConfiguration._8_10 , 230.0f}
        };

        II2cPeripheral _device;
        CancellationTokenSource _cts;

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
            Gain = gainConfiguration;

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

        /// <summary>
        /// Threshold for the difference between old and new readings during internal polling.
        /// </summary>
        public float Threshold { get; set; } = float.Epsilon;

        /// <summary>
        /// The current operating mode.
        /// </summary>
        public OperatingMode Mode { get; private set; }

        /// <summary>
        /// The currently chosen gain
        /// </summary>
        public GainConfiguration Gain { get; private set; }

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

            // Read raw sensor data and convert it to Gauss
            var data = _device.ReadRegisters(DXRA, 6);
            _currentX = ToGauss((short)((data[0] << 8) + data[1]));
            _currentY = ToGauss((short)((data[2] << 8) + data[3]));
            _currentZ = ToGauss((short)((data[4] << 8) + data[5]));

            return new Vector(_currentX, _currentY, _currentZ);
        }

        /// <summary>
        ///     Event to be raised when the magnetic field change is greater than +/- Threshold.
        ///
        /// <para> Make sure to start the internal polling after you register a handler. </para>
        /// </summary>
        public event SensorVectorEventHandler FieldStrengthChanged = delegate { };

        /// <summary>
        /// Updates the sensor output with the specified period. Invokes any registered <see cref="SensorVectorEventHandler"/>
        /// </summary>
        /// <param name="pollingPeriod">The time to wait between consecutive data updates</param>
        public void StartInternalPolling(TimeSpan pollingPeriod)
        {
            // Sanitize input
            if (pollingPeriod < TimeSpan.FromMilliseconds(7))
            {
                throw new ArgumentException("Cannot poll faster than ~160Hz");
            }

            // Stop any previous polling task - we don't want multiple tasks polling in parallel
            StopInternalPolling();

            _cts = new CancellationTokenSource();

            Task.Run(async () =>
            {
                var cancellationToken = _cts.Token;
                while (!cancellationToken.IsCancellationRequested)
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

                    await Task.Delay(pollingPeriod);
                }
            });
        }

        /// <summary>
        /// Stops internal polling, if running.
        /// </summary>
        public void StopInternalPolling()
        {
            _cts?.Cancel();
            _cts?.Dispose();
            _cts = null;
        }

        private float ToGauss(short rawSensorOutput)
        {
            var dividingFactor = _gainToDividingFactorMap[Gain];

            return rawSensorOutput / dividingFactor;
        }
    }
}

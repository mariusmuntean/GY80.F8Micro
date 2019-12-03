using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using Meadow.Foundation.Sensors;
using Meadow.Foundation.Spatial;
using Meadow.Hardware;

namespace GY80.Sensors.Gyro.L3G4200
{
    /// <summary>
    /// ST Microelectronics 3 axis gyro
    ///
    /// <para>Datasheet: http://www.forkrobotics.com/wp-content/uploads/2013/05/L3G4200D.pdf </para>
    /// </summary>
    public class L3G4200D
    {
        private byte _address;
        private FullScale _fullScale;
        private DataRate _dataRate;

        private double _prevX, _prevY, _prevZ;

        CancellationTokenSource _cts;
        float _factor;

        // Calibration factors
        double _xAxisZeroRate = 0.0d;
        double _yAxisZeroRate = 0.0d;
        double _zAxisZeroRate = 0.0d;

        double _xAxisThreshold = 0.0d;
        double _yAxisThreshold = 0.0d;
        double _zAxisThreshold = 0.0d;

        private const int CalibrationSampleCount = 100;
        private const int SigmaMultiple = 3;

        private const byte CTRL_REG1 = 0x20;
        private const byte CTRL_REG2 = 0x21;
        private const byte CTRL_REG3 = 0x22;
        private const byte CTRL_REG4 = 0x23;
        private const byte CTRL_REG5 = 0x24;

        private const byte REFERENCE = 0x25;
        private const byte OUT_TEMP = 0x26;
        private const byte STATUS_REG = 0x27;

        private const byte OUT_X_L = 0x28;
        private const byte OUT_X_H = 0x29;

        private const byte OUT_Y_L = 0x2A;
        private const byte OUT_Y_H = 0x2B;

        private const byte OUT_Z_L = 0x2C;
        private const byte OUT_Z_H = 0x2D;

        private Dictionary<FullScale, float> _fullScaleToFactorMap = new Dictionary<FullScale, float> {
            { FullScale.dps_250, 0.00875f},
            { FullScale.dps_500, 0.017f},
            { FullScale.dps_2000, 0.070f}
        };

        private II2cPeripheral _device;

        public enum DataRate
        {
            _100Hz = 0b0000_0000,
            _200Hz = 0b0100_0000,
            _400Hz = 0b1000_0000,
            _800Hz = 0b1100_0000
        }

        public enum PowerMode
        {
            PowerDown = 0b0000_0000,
            NormalOrSleep = 0b0000_1000
        }

        [Flags]
        public enum Axis
        {
            XAxis = 0b0000_0001,
            YAxis = 0b0000_0010,
            ZAxis = 0b0000_0100,
            NoAxis = 0b0000_0000
        }

        public enum FullScale
        {
            dps_250 = 0b00_00_0000,
            dps_500 = 0b00_01_0000,
            dps_2000 = 0b00_10_0000
        }

        public L3G4200D(II2cBus i2CBus,
            byte address = 0x69,
            DataRate dataRate = DataRate._100Hz,
            PowerMode powerMode = PowerMode.NormalOrSleep,
            FullScale fullScale = FullScale.dps_250,
            Axis enabledAxes = Axis.XAxis | Axis.YAxis | Axis.ZAxis)
        {
            _address = address;
            _fullScale = fullScale;
            _dataRate = dataRate;
            _device = new I2cPeripheral(i2CBus, address);

            // Config control register 1
            byte ctrlReg1Data = 0b0000_0000;
            ctrlReg1Data |= (byte)dataRate;
            ctrlReg1Data |= (byte)powerMode;
            ctrlReg1Data |= (byte)enabledAxes;
            _device.WriteRegister(CTRL_REG1, ctrlReg1Data);
            Console.WriteLine($"CTRL_REG1: {Convert.ToString(ctrlReg1Data, 2).PadLeft(8, '0')}");

            // Config control register 4
            byte ctrlReg4Data = 0b0000_0000;
            //ctrlReg4Data |= 0b1000_0000; // BDU 1 ensures that the output registers of an axis are updated only after both have been read. This avoids data corruption.
            ctrlReg4Data |= (byte)fullScale;
            _device.WriteRegister(CTRL_REG4, ctrlReg4Data);
            Console.WriteLine($"CTRL_REG4: {Convert.ToString(ctrlReg4Data, 2).PadLeft(8, '0')}");

            _device.WriteRegister(CTRL_REG2, (byte)0b0000_0000);
            _device.WriteRegister(CTRL_REG3, (byte)0b0000_1000);
            _device.WriteRegister(CTRL_REG5, (byte)0b0000_0000);

            // Based on the chosen FullScale we can choose the scaling factor
            _factor = _fullScaleToFactorMap[_fullScale];


            Thread.Sleep(100);
        }

        public Vector Read()
        {
            // Get raw sensor values
            var rawData = ReadRaw();

            // Factor in the calibration data
            rawData.X -= _xAxisZeroRate;
            rawData.X = Math.Abs(rawData.X) < _xAxisThreshold ? 0.0d : rawData.X;

            rawData.Y -= _yAxisZeroRate;
            rawData.Y = Math.Abs(rawData.Y) < _yAxisThreshold ? 0.0d : rawData.Y;

            rawData.Z -= _zAxisZeroRate;
            rawData.Z = Math.Abs(rawData.Z) < _zAxisThreshold ? 0.0d : rawData.Z;

            // Convert raw readings based on datasheet L3G4200D.pdf page 10/42
            return new Vector(rawData.X * _factor, rawData.Y * _factor, rawData.Z * _factor);
        }

        public void Calibrate()
        {
            Console.WriteLine("Calibrating");

            var sumX = 0.0d;
            var sumY = 0.0d;
            var sumZ = 0.0d;
            var sigmaX = 0.0d;
            var sigmaY = 0.0d;
            var sigmaZ = 0.0d;

            for (int i = 0; i < CalibrationSampleCount; i++)
            {
                var rawData = ReadRaw();

                sumX += rawData.X;
                sumY += rawData.Y;
                sumZ += rawData.Z;

                sigmaX += rawData.X * rawData.X;
                sigmaY += rawData.Y * rawData.Y;
                sigmaZ += rawData.Z * rawData.Z;

                Thread.Sleep(7);
            }

            // The zero rate is just the average of the samples
            _xAxisZeroRate = sumX / CalibrationSampleCount;
            _yAxisZeroRate = sumY / CalibrationSampleCount;
            _zAxisZeroRate = sumZ / CalibrationSampleCount;

            // The threshold seems to be based on the standard deviation of the samples times 3
            _xAxisThreshold = Math.Sqrt((sigmaX / CalibrationSampleCount) - (_xAxisZeroRate * _xAxisZeroRate)) * SigmaMultiple;
            _yAxisThreshold = Math.Sqrt((sigmaY / CalibrationSampleCount) - (_yAxisZeroRate * _yAxisZeroRate)) * SigmaMultiple;
            _zAxisThreshold = Math.Sqrt((sigmaZ / CalibrationSampleCount) - (_zAxisZeroRate * _zAxisZeroRate)) * SigmaMultiple;

            Console.WriteLine($"_xAxisZeroRate: {_xAxisZeroRate} _yAxisZeroRate: {_yAxisZeroRate} _zAxisZeroRate: {_zAxisZeroRate}");
            Console.WriteLine($"_xAxisThreshold: {_xAxisThreshold} _yAxisThreshold: {_yAxisThreshold} _zAxisThreshold: {_zAxisThreshold}");
        }

        public event SensorVectorEventHandler RotationChanged = delegate { };

        public bool IsNewDataReady()
        {
            var statusRegister = _device.ReadRegister(STATUS_REG);
            return (statusRegister & (byte)0b0000_1000) == (byte)0b0000_1000;
        }

        public void StopInternalPolling()
        {
            _cts?.Cancel();
            _cts?.Dispose();
            _cts = null;
        }

        public void StartInternalpolling()
        {
            StopInternalPolling();

            _cts = new CancellationTokenSource();

            Task.Run(async () =>
            {
                var delay = GetDelayFromDataRate();
                var token = _cts.Token;
                while (!token.IsCancellationRequested)
                {
                    while (!IsNewDataReady())
                    {
                        Console.WriteLine("Data not yet ready, waiting ...");
                        await Task.Delay(1);
                    }

                    var currentValues = Read();
                    var args = new SensorVectorEventArgs(new Vector(_prevX, _prevY, _prevZ), currentValues);
                    RotationChanged?.Invoke(this, args);

                    _prevX = currentValues.X;
                    _prevY = currentValues.Y;
                    _prevZ = currentValues.Z;

                    await Task.Delay(delay);
                }
            });
        }

        //private Vector ReadRaw()
        //{
        //    // For sequential ready we need to OR the address with 0b1000_0000. Datasheet L3G4200D.pdf page 23/42
        //    var rawData = _device.ReadRegisters(OUT_X_L | 0b1000_0000, 6);
        //    var rawX = (rawData[1] << 8) | rawData[0];
        //    var rawY = (rawData[3] << 8) | rawData[2];
        //    var rawZ = (rawData[5] << 8) | rawData[4];

        //    return new Vector(rawX, rawY, rawZ);
        //}


        private Vector ReadRaw()
        {
            // For sequential ready we need to OR the address with 0b1000_0000. Datasheet L3G4200D.pdf page 23/42
            var rawData = _device.ReadRegisters(OUT_X_L | 0b1000_0000, 6);
            var rawX = (_device.ReadRegister(OUT_X_H) << 8) | _device.ReadRegister(OUT_X_L);
            var rawY = (_device.ReadRegister(OUT_Y_H) << 8) | _device.ReadRegister(OUT_Y_L);
            var rawZ = (_device.ReadRegister(OUT_Z_H) << 8) | _device.ReadRegister(OUT_Z_L);

            return new Vector(rawX, rawY, rawZ);
        }

        private TimeSpan GetDelayFromDataRate()
        {
            switch (_dataRate)
            {
                case DataRate._100Hz:
                    return TimeSpan.FromMilliseconds(10);
                case DataRate._200Hz:
                    return TimeSpan.FromMilliseconds(5);
                case DataRate._400Hz:
                    return TimeSpan.FromMilliseconds(2.5d);
                case DataRate._800Hz:
                    return TimeSpan.FromMilliseconds(1.25d);
                default:
                    throw new NotSupportedException($"'{_dataRate}' not supported!");
            }
        }
    }
}

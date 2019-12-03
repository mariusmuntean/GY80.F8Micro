using System;
using System.Threading;
using GY80.Sensors.Gyro.L3G4200;
using GY80.Sensors.Magnetometer.HMC5883L;
using Meadow;
using Meadow.Devices;

namespace GY80
{
    public class GY80 : App<F7Micro, GY80>
    {
        public GY80()
        {
            //MagnetometerExample();
            GyroExample();
        }

        private void GyroExample()
        {
            Console.WriteLine("\n\n");
            Console.WriteLine("L3G4200D Interrupt Example.");
            Console.WriteLine("--------------------------");

            var l3g4200d = new L3G4200D(Device.CreateI2cBus());
            l3g4200d.Calibrate();

            l3g4200d.RotationChanged += (s, e) =>
            {
                Console.WriteLine($"X: {e.CurrentValue.X}, Y: {e.CurrentValue.Y}, Z: {e.CurrentValue.Z}");
            };

            l3g4200d.StartInternalpolling();

            // Put Meadow to sleep as the interrupt handler will deal 
            // with changes in orientation.
            Thread.Sleep(Timeout.Infinite);
        }

        private static void MagnetometerExample()
        {
            Console.WriteLine("\n\n");
            Console.WriteLine("hmc5883l Interrupt Example.");
            Console.WriteLine("--------------------------");

            var hmc5883l = new HMC5883L(Device.CreateI2cBus());

            // Attach an interrupt handler.            
            hmc5883l.FieldStrengthChanged += (s, e) =>
            {
                Console.WriteLine($"X: {e.CurrentValue.X}, Y: {e.CurrentValue.Y}, Z: {e.CurrentValue.Z}");
            };

            hmc5883l.StartInternalPolling(TimeSpan.FromMilliseconds(200));
            Console.WriteLine("Running first time");
            Thread.Sleep(1000);

            Console.WriteLine("Stopping");
            hmc5883l.StopInternalPolling();
            Console.WriteLine("Waiting one second");
            Thread.Sleep(1000);

            Console.WriteLine("Continuing");
            hmc5883l.StartInternalPolling(TimeSpan.FromMilliseconds(200));

            // Put Meadow to sleep as the interrupt handler will deal 
            // with changes in orientation.
            Thread.Sleep(Timeout.Infinite);
        }
    }
}
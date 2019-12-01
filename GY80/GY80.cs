using System;
using System.Threading;
using Meadow;
using Meadow.Devices;
using Meadow.Hardware;

namespace GY80
{
    public class GY80 : App<F7Micro, GY80>
    {
        IDigitalOutputPort redLed;
        IDigitalOutputPort blueLed;
        IDigitalOutputPort greenLed;

        public GY80()
        {
            Console.WriteLine("\n\n");
            Console.WriteLine("hmc5883l Interrupt Example.");
            Console.WriteLine("--------------------------");

            var hmc5883l = new HMC5883L(Device.CreateI2cBus());

            // Attach an interrupt handler.            
            hmc5883l.FieldStrengthChanged += (s, e) =>
            {
                Console.WriteLine("X: " + e.CurrentValue.X.ToString() +
                            ", Y: " + e.CurrentValue.Y.ToString() +
                            ", Z: " + e.CurrentValue.Z.ToString());
            };

            hmc5883l.StartInternalPolling(TimeSpan.FromMilliseconds(200));

            // Put Meadow to sleep as the interrupt handler will deal 
            // with changes in orientation.
            Thread.Sleep(Timeout.Infinite);
        }


    }
}
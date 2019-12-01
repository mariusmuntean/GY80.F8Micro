using System.Threading;
using Meadow;

namespace GY80
{
    class Program
    {
        static IApp app;
        public static void Main(string[] args)
        {
            // instantiate and run new meadow app
            app = new GY80();

            Thread.Sleep(Timeout.Infinite);
        }
    }
}

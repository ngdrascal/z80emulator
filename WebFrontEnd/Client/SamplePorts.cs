using z80;

namespace WebFrontEnd.Client
{
    internal class SamplePorts : IPorts
    {
        public byte ReadPort(ushort port)
        {
            Console.WriteLine($"IN 0x{port:X4}");
            return 0;
        }
        public void WritePort(ushort port, byte value)
        {
            Console.WriteLine($"OUT 0x{port:X4}, 0x{value:X2}");
        }
        public bool NMI => false;
        public bool MI => false;
        public byte Data => 0x00;
    }
}

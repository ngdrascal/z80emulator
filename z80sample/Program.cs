﻿using System;
using System.IO;
using z80;

namespace z80Sample
{
    internal static class Program
    {
        private static void Main()
        {
            var ram = new byte[65536];
            Array.Clear(ram, 0, ram.Length);
            var inp = File.ReadAllBytes("48.rom");
            if (inp.Length != 16384) throw new InvalidOperationException("Invalid 48.rom file");

            Array.Copy(inp, ram, 16384);

            var myZ80 = new Z80(new Memory(ram, 16384), new SamplePorts(), new ColorConsoleLogger());
            Console.Clear();
            //var counter = 0;
            while (!myZ80.Halt)
            {
                myZ80.Parse();
                //counter++;
                //if (counter % 1000 == 1)
                //{
                //    if (Console.KeyAvailable)
                //        break;
                //    var registers = myZ80.GetState();
                //    Console.WriteLine($"0x{(ushort)(registers[25] + (registers[24] << 8)):X4}");
                //}
            }


            Console.WriteLine(Environment.NewLine + myZ80.DumpState());
            for (var i = 0; i < 0x80; i++)
            {
                if (i % 16 == 0) Console.Write("{0:X4} | ", i);
                Console.Write("{0:x2} ", ram[i]);
                if (i % 8 == 7) Console.Write("  ");
                if (i % 16 == 15) Console.WriteLine();
            }
            Console.WriteLine();
            for (var i = 0x4000; i < 0x4100; i++)
            {
                if (i % 16 == 0) Console.Write("{0:X4} | ", i);
                Console.Write("{0:x2} ", ram[i]);
                if (i % 8 == 7) Console.Write("  ");
                if (i % 16 == 15) Console.WriteLine();
            }
        }
    }

    class SamplePorts : IPorts
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
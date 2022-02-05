using System;

namespace z80
{
    public interface IInstructionLogger
    {
        void LogMemRead(ushort addr, byte val);
        void Log(string text);
        string RName(byte n);
        string R16Name(byte n);
    }

    public class NullLogger : IInstructionLogger
    {
        public static readonly IInstructionLogger DefaultLogger = new NullLogger();
 
        public void LogMemRead(ushort addr, byte val)
        {
        }

        public void Log(string text)
        {
        }

        public string RName(byte n)
        {
            return string.Empty;
        }

        public string R16Name(byte n)
        {
            return string.Empty;
        }
    }

    public class ConsoleLogger : IInstructionLogger
    {
        private  bool _debugAtStart = true;

        public void LogMemRead(ushort addr, byte val)
        {
            if (_debugAtStart)
            {
                Console.ForegroundColor = ConsoleColor.Green;
                Console.Write($"{addr:X4} ");
                _debugAtStart = false;
            }
            Console.ForegroundColor = ConsoleColor.Yellow;
            Console.Write($"{val:X2} ");
            Console.ForegroundColor = ConsoleColor.White;
        }

        public void Log(string text)
        {
            Console.CursorLeft = 20;
            Console.ForegroundColor = ConsoleColor.Cyan;
            Console.WriteLine(text);
            Console.ForegroundColor = ConsoleColor.White;
            _debugAtStart = true;
        }

        public string RName(byte n)
        {
            switch (n)
            {
                case 0:
                    return "B";
                case 1:
                    return "C";
                case 2:
                    return "D";
                case 3:
                    return "E";
                case 4:
                    return "H";
                case 5:
                    return "L";
                case 7:
                    return "A";
                default:
                    return "";
            }
        }

        public string R16Name(byte n)
        {
            switch (n)
            {
                case 0x00:
                    return "BC";
                case 0x10:
                    return "DE";
                case 0x20:
                    return "HL";
                case 0x30:
                    return "SP";
            }
            return "";
        }
    }


}

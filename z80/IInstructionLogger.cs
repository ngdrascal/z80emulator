using System;
using System.Text;

namespace z80
{
    public interface IInstructionLogger
    {
        void LogMemRead(ushort addr, byte val);
        void Log(string text);
        string RName(byte n);
        string R16Name(byte n);
    }

    public class LoggerBase
    {
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

    public class ConsoleLogger : LoggerBase, IInstructionLogger
    {
        public static readonly IInstructionLogger DefaultLogger = new ConsoleLogger();

        private bool _debugAtStart = true;
        private readonly StringBuilder _sb = new();

        public void LogMemRead(ushort addr, byte val)
        {
            if (_debugAtStart)
            {
                _sb.Append($"{addr:X4} ");
            }
            _sb.Append($"{val:X2} ");
        }

        public void Log(string text)
        {
            Console.Write(_sb.ToString().PadRight(32));
            Console.WriteLine(text);
            _sb.Clear();
            _debugAtStart = true;
        }
    }

    public class ColorConsoleLogger : LoggerBase, IInstructionLogger
    {
        private bool _debugAtStart = true;

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
    }
}

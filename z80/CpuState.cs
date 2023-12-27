namespace z80
{
    public class CpuState
    {

        public CpuState(byte[] registers)
        {
            Registers = new Registers(registers);
            Flags = new Flags(registers[6]);
        }

        public Registers Registers { get; }
        public Flags Flags { get; }
    }

    public class Registers
    {
        private const byte BIdx = 0;
        private const byte CIdx = 1;
        private const byte DIdx = 2;
        private const byte EIdx = 3;
        private const byte HIdx = 4;
        private const byte LIdx = 5;
        private const byte FIdx = 6;
        private const byte AIdx = 7;
        private const byte BpIdx = 8;
        private const byte CpIdx = 9;
        private const byte DpIdx = 10;
        private const byte EpIdx = 11;
        private const byte HpIdx = 12;
        private const byte LpIdx = 13;
        private const byte FpIdx = 14;
        private const byte ApIdx = 15;
        private const byte IIdx = 16;
        private const byte RIdx = 17;
        private const byte IXIdx = 18;
        private const byte IYIdx = 20;
        private const byte SPIdx = 22;
        private const byte PCIdx = 24;

        private readonly byte[] _registers;

        public Registers(byte[] registers)
        {
            _registers = registers;
        }

        public byte A => _registers[AIdx];
        public byte B => _registers[BIdx];
        public byte C => _registers[CIdx];
        public byte D => _registers[DIdx];
        public byte E => _registers[EIdx];
        public byte H => _registers[HIdx];
        public byte L => _registers[LIdx];
        public byte F => _registers[FIdx];

        public byte Ap => _registers[ApIdx];
        public byte Bp => _registers[BpIdx];
        public byte Cp => _registers[CpIdx];
        public byte Dp => _registers[DpIdx];
        public byte Ep => _registers[EpIdx];
        public byte Hp => _registers[HpIdx];
        public byte Lp => _registers[LpIdx];
        public byte Fp => _registers[FpIdx];

        public ushort HL => (ushort)(_registers[LIdx] + (_registers[HIdx] << 8));
        public ushort SP => (ushort)(_registers[SPIdx + 1] + (_registers[SPIdx] << 8));
        public ushort IX => (ushort)(_registers[IXIdx + 1] + (_registers[IXIdx] << 8));
        public ushort IY => (ushort)(_registers[IYIdx + 1] + (_registers[IYIdx] << 8));
        public ushort BC => (ushort)((_registers[BIdx] << 8) + _registers[CIdx]);
        public ushort DE => (ushort)((_registers[DIdx] << 8) + _registers[EIdx]);
        public ushort PC => (ushort)(_registers[PCIdx + 1] + (_registers[PCIdx] << 8));
    }

    public class Flags
    {
        private byte _flags;

        public Flags(byte flags)
        {
            _flags = flags;
        }

        public bool S { get; set; }
        public bool Z { get; set; }
        public bool H { get; set; }
        public bool PV { get; set; }
        public bool N { get; set; }
        public bool C { get; set; }
    }
}

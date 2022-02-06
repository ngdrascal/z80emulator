using System;
using System.Threading;

namespace z80
{
    public class Z80
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
        private const byte IxIdx = 18;
        private const byte IyIdx = 20;
        private const byte SpIdx = 22;
        private const byte PcIdx = 24;
        private readonly byte[] _registers = new byte[26];

        private readonly Memory mem;
        private DateTime _clock = DateTime.UtcNow;
        private bool IFF1;
        private bool IFF2;
        private int interruptMode;

        private readonly IPorts ports;
        private readonly IInstructionLogger _logger;

        public Z80(Memory memory, IPorts ports, IInstructionLogger logger)
        {
            if (memory == null) throw new ArgumentNullException(nameof(memory));
            if (ports == null) throw new ArgumentNullException(nameof(ports));
            mem = memory;
            this.ports = ports;
            _logger = logger;
            Reset();
        }

        public ushort Hl => (ushort)(_registers[LIdx] + (_registers[HIdx] << 8));
        public ushort Sp => (ushort)(_registers[SpIdx + 1] + (_registers[SpIdx] << 8));
        public ushort Ix => (ushort)(_registers[IxIdx + 1] + (_registers[IxIdx] << 8));
        public ushort Iy => (ushort)(_registers[IyIdx + 1] + (_registers[IyIdx] << 8));
        public ushort Bc => (ushort)((_registers[BIdx] << 8) + _registers[CIdx]);
        public ushort De => (ushort)((_registers[DIdx] << 8) + _registers[EIdx]);
        public ushort Pc => (ushort)(_registers[PcIdx + 1] + (_registers[PcIdx] << 8));

        public bool Halt { get; private set; }

        public void Parse()
        {
            if (ports.NMI)
            {
                var stack = Sp;
                mem[--stack] = (byte)(Pc >> 8);
                mem[--stack] = (byte)(Pc);
                _registers[SpIdx] = (byte)(stack >> 8);
                _registers[SpIdx + 1] = (byte)(stack);
                _registers[PcIdx] = 0x00;
                _registers[PcIdx + 1] = 0x66;
                IFF1 = IFF2;
                IFF1 = false;
#if (DEBUG)
                _logger.Log("NMI");
#endif
                Wait(17);
                Halt = false;
                return;
            }
            if (IFF1 && ports.MI)
            {
                IFF1 = false;
                IFF2 = false;
                switch (interruptMode)
                {
                    case 0:
                        {
                            // This is not quite correct, as it only runs a RST xx
                            // Instead, it should also support any other instruction
                            var instruction = ports.Data;
                            var stack = Sp;
                            mem[--stack] = (byte)(Pc >> 8);
                            mem[--stack] = (byte)(Pc);
                            _registers[SpIdx] = (byte)(stack >> 8);
                            _registers[SpIdx + 1] = (byte)(stack);
                            _registers[PcIdx] = 0x00;
                            _registers[PcIdx + 1] = (byte)(instruction & 0x38);
                            Wait(17);

#if (DEBUG)
                            _logger.Log("MI 0");
#endif
                            Halt = false;
                            return;
                        }
                    case 1:
                        {
                            var stack = Sp;
                            mem[--stack] = (byte)(Pc >> 8);
                            mem[--stack] = (byte)(Pc);
                            _registers[SpIdx] = (byte)(stack >> 8);
                            _registers[SpIdx + 1] = (byte)(stack);
                            _registers[PcIdx] = 0x00;
                            _registers[PcIdx + 1] = 0x38;
#if (DEBUG)
                            _logger.Log("MI 1");
#endif
                            Wait(17);
                            Halt = false;
                            return;
                        }
                    case 2:
                        {
                            var vector = ports.Data;
                            var stack = Sp;
                            mem[--stack] = (byte)(Pc >> 8);
                            mem[--stack] = (byte)(Pc);
                            _registers[SpIdx] = (byte)(stack >> 8);
                            _registers[SpIdx + 1] = (byte)(stack);
                            var address = (ushort)((_registers[IIdx] << 8) + vector);
                            _registers[PcIdx] = mem[address++];
                            _registers[PcIdx + 1] = mem[address];
#if (DEBUG)
                            _logger.Log("MI 2");
#endif
                            Wait(17);
                            Halt = false;
                            return;
                        }
                }
                return;
            }
            if (Halt) return;
            var mc = Fetch();
            var hi = (byte)(mc >> 6);
            var lo = (byte)(mc & 0x07);
            var r = (byte)((mc >> 3) & 0x07);
            if (hi == 1)
            {
                var useHL1 = r == 6;
                var useHL2 = lo == 6;
                if (useHL2 && useHL1)
                {
#if(DEBUG)
                    _logger.Log("HALT");
#endif
                    Halt = true;
                    return;
                }
                var reg = useHL2 ? mem[Hl] : _registers[lo];

                if (useHL1)
                    mem[Hl] = reg;
                else
                    _registers[r] = reg;
                Wait(useHL1 || useHL2 ? 7 : 4);
#if (DEBUG)
                _logger.Log($"LD {(useHL1 ? "(HL)" : _logger.RName(r))}, {(useHL2 ? "(HL)" : _logger.RName(lo))}");
#endif
                return;
            }
            switch (mc)
            {
                case 0xCB:
                    ParseCB();
                    return;
                case 0xDD:
                    ParseDD();
                    return;
                case 0xED:
                    ParseED();
                    return;
                case 0xFD:
                    ParseFD();
                    return;
                case 0x00:
                    // NOP
#if(DEBUG)
                    _logger.Log("NOP");
#endif
                    Wait(4);
                    return;
                case 0x01:
                case 0x11:
                case 0x21:
                    {
                        // LD dd, nn
                        _registers[r + 1] = Fetch();
                        _registers[r] = Fetch();
#if (DEBUG)
                        _logger.Log($"LD {_logger.RName(r)}{_logger.RName((byte)(r + 1))}, 0x{_registers[r]:X2}{_registers[r + 1]:X2}");
#endif
                        Wait(10);
                        return;
                    }
                case 0x31:
                    {
                        // LD SP, nn
                        _registers[SpIdx + 1] = Fetch();
                        _registers[SpIdx] = Fetch();
#if (DEBUG)
                        _logger.Log($"LD SP, 0x{_registers[SpIdx]:X2}{_registers[SpIdx + 1]:X2}");
#endif
                        Wait(10);
                        return;
                    }
                case 0x06:
                case 0x0e:
                case 0x16:
                case 0x1e:
                case 0x26:
                case 0x2e:
                case 0x3e:
                    {
                        // LD r,n
                        var n = Fetch();
                        _registers[r] = n;
#if (DEBUG)
                        _logger.Log($"LD {_logger.RName(r)}, 0x{n:X2}");
#endif
                        Wait(7);
                        return;
                    }
                case 0x36:
                    {
                        // LD (HL), n
                        var n = Fetch();
                        mem[Hl] = n;
#if (DEBUG)
                        _logger.Log($"LD (HL), {n}");
#endif
                        Wait(10);
                        return;
                    }
                case 0x0A:
                    {
                        // LD A, (BC)
                        _registers[AIdx] = mem[Bc];
#if (DEBUG)
                        _logger.Log("LD A, (BC)");
#endif
                        Wait(7);
                        return;
                    }
                case 0x1A:
                    {
                        // LD A, (DE)
                        _registers[AIdx] = mem[De];
#if (DEBUG)
                        _logger.Log("LD A, (DE)");
#endif
                        Wait(7);
                        return;
                    }
                case 0x3A:
                    {
                        // LD A, (nn)
                        var addr = Fetch16();
                        _registers[AIdx] = mem[addr];
#if (DEBUG)
                        _logger.Log($"LD A, (0x{addr:X4})");
#endif
                        Wait(13);
                        return;
                    }
                case 0x02:
                    {
                        // LD (BC), A
                        mem[Bc] = _registers[AIdx];
#if (DEBUG)
                        _logger.Log("LD (BC), A");
#endif
                        Wait(7);
                        return;
                    }
                case 0x12:
                    {
                        // LD (DE), A
                        mem[De] = _registers[AIdx];
#if (DEBUG)
                        _logger.Log("LD (DE), A");
#endif
                        Wait(7);
                        return;
                    }
                case 0x32:
                    {
                        // LD (nn), A 
                        var addr = Fetch16();
                        mem[addr] = _registers[AIdx];
#if (DEBUG)
                        _logger.Log($"LD (0x{addr:X4}), A");
#endif
                        Wait(13);
                        return;
                    }
                case 0x2A:
                    {
                        // LD HL, (nn) 
                        var addr = Fetch16();
                        _registers[LIdx] = mem[addr++];
                        _registers[HIdx] = mem[addr];
#if (DEBUG)
                        _logger.Log($"LD HL, (0x{--addr:X4})");
#endif
                        Wait(16);
                        return;
                    }
                case 0x22:
                    {
                        // LD (nn), HL
                        var addr = Fetch16();
                        mem[addr++] = _registers[LIdx];
                        mem[addr] = _registers[HIdx];
#if (DEBUG)
                        _logger.Log($"LD (0x{--addr:X4}), HL");
#endif
                        Wait(16);
                        return;
                    }
                case 0xF9:
                    {
                        // LD SP, HL
                        _registers[SpIdx + 1] = _registers[LIdx];
                        _registers[SpIdx] = _registers[HIdx];
#if (DEBUG)
                        _logger.Log("LD SP, HL");
#endif
                        Wait(6);
                        return;
                    }

                case 0xC5:
                    {
                        // PUSH BC
                        var addr = Sp;
                        mem[--addr] = _registers[BIdx];
                        mem[--addr] = _registers[CIdx];
                        _registers[SpIdx + 1] = (byte)(addr & 0xFF);
                        _registers[SpIdx] = (byte)(addr >> 8);
#if (DEBUG)
                        _logger.Log("PUSH BC");
#endif
                        Wait(11);
                        return;
                    }
                case 0xD5:
                    {
                        // PUSH DE
                        var addr = Sp;
                        mem[--addr] = _registers[DIdx];
                        mem[--addr] = _registers[EIdx];
                        _registers[SpIdx + 1] = (byte)(addr & 0xFF);
                        _registers[SpIdx] = (byte)(addr >> 8);
#if (DEBUG)
                        _logger.Log("PUSH DE");
#endif
                        Wait(11);
                        return;
                    }
                case 0xE5:
                    {
                        // PUSH HL
                        var addr = Sp;
                        mem[--addr] = _registers[HIdx];
                        mem[--addr] = _registers[LIdx];
                        _registers[SpIdx + 1] = (byte)(addr & 0xFF);
                        _registers[SpIdx] = (byte)(addr >> 8);
#if (DEBUG)
                        _logger.Log("PUSH HL");
#endif
                        Wait(11);
                        return;
                    }
                case 0xF5:
                    {
                        // PUSH AF
                        var addr = Sp;
                        mem[--addr] = _registers[AIdx];
                        mem[--addr] = _registers[FIdx];
                        _registers[SpIdx + 1] = (byte)(addr & 0xFF);
                        _registers[SpIdx] = (byte)(addr >> 8);
#if (DEBUG)
                        _logger.Log("PUSH AF");
#endif
                        Wait(11);
                        return;
                    }
                case 0xC1:
                    {
                        // POP BC
                        var addr = Sp;
                        _registers[CIdx] = mem[addr++];
                        _registers[BIdx] = mem[addr++];
                        _registers[SpIdx + 1] = (byte)(addr & 0xFF);
                        _registers[SpIdx] = (byte)(addr >> 8);
#if (DEBUG)
                        _logger.Log("POP BC");
#endif
                        Wait(10);
                        return;
                    }
                case 0xD1:
                    {
                        // POP DE
                        var addr = Sp;
                        _registers[EIdx] = mem[addr++];
                        _registers[DIdx] = mem[addr++];
                        _registers[SpIdx + 1] = (byte)(addr & 0xFF);
                        _registers[SpIdx] = (byte)(addr >> 8);
#if (DEBUG)
                        _logger.Log("POP DE");
#endif
                        Wait(10);
                        return;
                    }
                case 0xE1:
                    {
                        // POP HL
                        var addr = Sp;
                        _registers[LIdx] = mem[addr++];
                        _registers[HIdx] = mem[addr++];
                        _registers[SpIdx + 1] = (byte)(addr & 0xFF);
                        _registers[SpIdx] = (byte)(addr >> 8);
#if (DEBUG)
                        _logger.Log("POP HL");
#endif
                        Wait(10);
                        return;
                    }
                case 0xF1:
                    {
                        // POP AF
                        var addr = Sp;
                        _registers[FIdx] = mem[addr++];
                        _registers[AIdx] = mem[addr++];
                        _registers[SpIdx + 1] = (byte)(addr & 0xFF);
                        _registers[SpIdx] = (byte)(addr >> 8);
#if (DEBUG)
                        _logger.Log("POP AF");
#endif
                        Wait(10);
                        return;
                    }
                case 0xEB:
                    {
                        // EX DE, HL
                        SwapReg8(DIdx, HIdx);
                        SwapReg8(EIdx, LIdx);
#if (DEBUG)
                        _logger.Log("EX DE, HL");
#endif
                        Wait(4);
                        return;
                    }
                case 0x08:
                    {
                        // EX AF, AF'
                        SwapReg8(ApIdx, AIdx);
                        SwapReg8(FpIdx, FIdx);
#if (DEBUG)
                        _logger.Log("EX AF, AF'");
#endif
                        Wait(4);
                        return;
                    }
                case 0xD9:
                    {
                        // EXX
                        SwapReg8(BIdx, BpIdx);
                        SwapReg8(CIdx, CpIdx);
                        SwapReg8(DIdx, DpIdx);
                        SwapReg8(EIdx, EpIdx);
                        SwapReg8(HIdx, HpIdx);
                        SwapReg8(LIdx, LpIdx);
#if (DEBUG)
                        _logger.Log("EXX");
#endif
                        Wait(4);
                        return;
                    }
                case 0xE3:
                    {
                        // EX (SP), HL
                        var addr = Sp;

                        var tmp = _registers[LIdx];
                        _registers[LIdx] = mem[addr];
                        mem[addr++] = tmp;

                        tmp = _registers[HIdx];
                        _registers[HIdx] = mem[addr];
                        mem[addr] = tmp;

#if (DEBUG)
                        _logger.Log("EX (SP), HL");
#endif
                        Wait(19);
                        return;
                    }
                case 0x80:
                case 0x81:
                case 0x82:
                case 0x83:
                case 0x84:
                case 0x85:
                case 0x87:
                    {
                        // ADD A, r
                        Add(_registers[lo]);
#if (DEBUG)
                        _logger.Log($"ADD A, {_logger.RName(lo)}");
#endif
                        Wait(4);
                        return;
                    }
                case 0xC6:
                    {
                        // ADD A, n
                        var b = Fetch();
                        Add(b);
#if (DEBUG)
                        _logger.Log($"ADD A, 0x{b:X2}");
#endif
                        Wait(4);
                        Wait(4);
                        return;
                    }
                case 0x86:
                    {
                        // ADD A, (HL)
                        Add(mem[Hl]);
#if (DEBUG)
                        _logger.Log("ADD A, (HL)");
#endif
                        Wait(7);
                        return;
                    }
                case 0x88:
                case 0x89:
                case 0x8A:
                case 0x8B:
                case 0x8C:
                case 0x8D:
                case 0x8F:
                    {
                        // ADC A, r
                        Adc(_registers[lo]);
#if (DEBUG)
                        _logger.Log($"ADC A, {_logger.RName(lo)}");
#endif
                        Wait(4);
                        return;
                    }
                case 0xCE:
                    {
                        // ADC A, n
                        var b = Fetch();
                        Adc(b);
#if (DEBUG)
                        _logger.Log($"ADC A, 0x{b:X2}");
#endif
                        Wait(4);
                        return;
                    }
                case 0x8E:
                    {
                        // ADC A, (HL)
                        Adc(mem[Hl]);
#if (DEBUG)
                        _logger.Log("ADC A, (HL)");
#endif
                        Wait(7);
                        return;
                    }
                case 0x90:
                case 0x91:
                case 0x92:
                case 0x93:
                case 0x94:
                case 0x95:
                case 0x97:
                    {
                        // SUB A, r
                        Sub(_registers[lo]);
#if (DEBUG)
                        _logger.Log($"SUB A, {_logger.RName(lo)}");
#endif
                        Wait(4);
                        return;
                    }
                case 0xD6:
                    {
                        // SUB A, n
                        var b = Fetch();
                        Sub(b);
#if (DEBUG)
                        _logger.Log($"SUB A, 0x{b:X2}");
#endif
                        Wait(4);
                        return;
                    }
                case 0x96:
                    {
                        // SUB A, (HL)
                        Sub(mem[Hl]);
#if (DEBUG)
                        _logger.Log("SUB A, (HL)");
#endif
                        Wait(7);
                        return;
                    }
                case 0x98:
                case 0x99:
                case 0x9A:
                case 0x9B:
                case 0x9C:
                case 0x9D:
                case 0x9F:
                    {
                        // SBC A, r
                        Sbc(_registers[lo]);
#if (DEBUG)
                        _logger.Log($"SBC A, {_logger.RName(lo)}");
#endif
                        Wait(4);
                        return;
                    }
                case 0xDE:
                    {
                        // SBC A, n
                        var b = Fetch();
                        Sbc(b);
#if (DEBUG)
                        _logger.Log($"SBC A, 0x{b:X2}");
#endif
                        Wait(4);
                        return;
                    }
                case 0x9E:
                    {
                        // SBC A, (HL)
                        Sbc(mem[Hl]);
#if (DEBUG)
                        _logger.Log("SBC A, (HL)");
#endif
                        Wait(7);
                        return;
                    }

                case 0xA0:
                case 0xA1:
                case 0xA2:
                case 0xA3:
                case 0xA4:
                case 0xA5:
                case 0xA7:
                    {
                        // AND A, r
                        And(_registers[lo]);
#if (DEBUG)
                        _logger.Log($"AND A, {_logger.RName(lo)}");
#endif
                        Wait(4);
                        return;
                    }
                case 0xE6:
                    {
                        // AND A, n
                        var b = Fetch();

                        And(b);
#if (DEBUG)
                        _logger.Log($"AND A, 0x{b:X2}");
#endif
                        Wait(4);
                        return;
                    }
                case 0xA6:
                    {
                        // AND A, (HL)
                        And(mem[Hl]);
#if (DEBUG)
                        _logger.Log("AND A, (HL)");
#endif
                        Wait(7);
                        return;
                    }
                case 0xB0:
                case 0xB1:
                case 0xB2:
                case 0xB3:
                case 0xB4:
                case 0xB5:
                case 0xB7:
                    {
                        // OR A, r
                        Or(_registers[lo]);
#if (DEBUG)
                        _logger.Log($"OR A, {_logger.RName(lo)}");
#endif
                        Wait(4);
                        return;
                    }
                case 0xF6:
                    {
                        // OR A, n
                        var b = Fetch();
                        Or(b);
#if (DEBUG)
                        _logger.Log($"OR A, 0x{b:X2}");
#endif
                        Wait(4);
                        return;
                    }
                case 0xB6:
                    {
                        // OR A, (HL)
                        Or(mem[Hl]);
#if (DEBUG)
                        _logger.Log("OR A, (HL)");
#endif
                        Wait(7);
                        return;
                    }
                case 0xA8:
                case 0xA9:
                case 0xAA:
                case 0xAB:
                case 0xAC:
                case 0xAD:
                case 0xAF:
                    {
                        // XOR A, r
                        Xor(_registers[lo]);
#if (DEBUG)
                        _logger.Log($"XOR A, {_logger.RName(lo)}");
#endif
                        Wait(4);
                        return;
                    }
                case 0xEE:
                    {
                        // XOR A, n
                        var b = Fetch();
                        Xor(b);
#if (DEBUG)
                        _logger.Log($"XOR A, 0x{b:X2}");
#endif
                        Wait(4);
                        return;
                    }
                case 0xAE:
                    {
                        // XOR A, (HL)
                        Xor(mem[Hl]);
#if (DEBUG)
                        _logger.Log("XOR A, (HL)");
#endif
                        Wait(7);
                        return;
                    }

                case 0xF3:
                    {
                        // DI
                        IFF1 = false;
                        IFF2 = false;
#if (DEBUG)
                        _logger.Log("DI");
#endif
                        Wait(4);
                        return;
                    }
                case 0xFB:
                    {
                        // EI
                        IFF1 = true;
                        IFF2 = true;
#if (DEBUG)
                        _logger.Log("EI");
#endif
                        Wait(4);
                        return;
                    }
                case 0xB8:
                case 0xB9:
                case 0xBA:
                case 0xBB:
                case 0xBC:
                case 0xBD:
                case 0xBF:
                    {
                        // CP A, r
                        Cmp(_registers[lo]);
#if (DEBUG)
                        _logger.Log($"CP A, {_logger.RName(lo)}");
#endif
                        Wait(4);
                        return;
                    }
                case 0xFE:
                    {
                        // CP A, n
                        var b = Fetch();
                        Cmp(b);
#if (DEBUG)
                        _logger.Log($"CP A, 0x{b:X2}");
#endif
                        Wait(4);
                        return;
                    }
                case 0xBE:
                    {
                        // CP A, (HL)
                        Cmp(mem[Hl]);
#if (DEBUG)
                        _logger.Log("CP A, (HL)");
#endif
                        Wait(7);
                        return;
                    }
                case 0x04:
                case 0x0C:
                case 0x14:
                case 0x1C:
                case 0x24:
                case 0x2C:
                case 0x3C:
                    {
                        // INC r
                        _registers[r] = Inc(_registers[r]);
#if (DEBUG)
                        _logger.Log($"INC {_logger.RName(r)}");
#endif
                        Wait(4);
                        return;
                    }
                case 0x34:
                    {
                        // INC (HL)
                        mem[Hl] = Inc(mem[Hl]);
#if (DEBUG)
                        _logger.Log("INC (HL)");
#endif
                        Wait(7);
                        return;
                    }

                case 0x05:
                case 0x0D:
                case 0x15:
                case 0x1D:
                case 0x25:
                case 0x2D:
                case 0x3D:
                    {
                        // DEC r
                        _registers[r] = Dec(_registers[r]);
#if (DEBUG)
                        _logger.Log($"DEC {_logger.RName(r)}");
#endif
                        Wait(7);
                        return;
                    }
                case 0x35:
                    {
                        // DEC (HL)
                        mem[Hl] = Dec(mem[Hl]);
#if (DEBUG)
                        _logger.Log("DEC (HL)");
#endif
                        Wait(7);
                        return;
                    }
                case 0x27:
                    {
                        // DAA
                        var a = _registers[AIdx];
                        var f = _registers[FIdx];
                        if ((a & 0x0F) > 0x09 || (f & (byte)Fl.H) > 0)
                        {
                            Add(0x06);
                            a = _registers[AIdx];
                        }
                        if ((a & 0xF0) > 0x90 || (f & (byte)Fl.C) > 0)
                        {
                            Add(0x60);
                        }
#if (DEBUG)
                        _logger.Log("DAA");
#endif
                        Wait(4);
                        return;
                    }
                case 0x2F:
                    {
                        // CPL
                        _registers[AIdx] ^= 0xFF;
                        _registers[FIdx] |= (byte)(Fl.H | Fl.N);
#if (DEBUG)
                        _logger.Log("CPL");
#endif
                        Wait(4);
                        return;
                    }
                case 0x3F:
                    {
                        // CCF
                        _registers[FIdx] &= (byte)~(Fl.N);
                        _registers[FIdx] ^= (byte)(Fl.C);
#if (DEBUG)
                        _logger.Log("CCF");
#endif
                        Wait(4);
                        return;
                    }
                case 0x37:
                    {
                        // SCF
                        _registers[FIdx] &= (byte)~(Fl.N);
                        _registers[FIdx] |= (byte)(Fl.C);
#if (DEBUG)
                        _logger.Log("SCF");
#endif
                        Wait(4);
                        return;
                    }
                case 0x09:
                    {
                        AddHl(Bc);

#if (DEBUG)
                        _logger.Log("ADD HL, BC");
#endif
                        Wait(4);
                        return;
                    }
                case 0x19:
                    {
                        AddHl(De);
#if (DEBUG)
                        _logger.Log("ADD HL, DE");
#endif
                        Wait(4);
                        return;
                    }
                case 0x29:
                    {
                        AddHl(Hl);
#if (DEBUG)
                        _logger.Log("ADD HL, HL");
#endif
                        Wait(4);
                        return;
                    }
                case 0x39:
                    {
                        AddHl(Sp);
#if (DEBUG)
                        _logger.Log("ADD HL, SP");
#endif
                        Wait(4);
                        return;
                    }
                case 0x03:
                    {
                        var val = Bc + 1;
                        _registers[BIdx] = (byte)(val >> 8);
                        _registers[CIdx] = (byte)(val & 0xFF);
#if (DEBUG)
                        _logger.Log("INC BC");
#endif
                        Wait(4);
                        return;
                    }
                case 0x13:
                    {
                        var val = De + 1;
                        _registers[DIdx] = (byte)(val >> 8);
                        _registers[EIdx] = (byte)(val & 0xFF);
#if (DEBUG)
                        _logger.Log("INC DE");
#endif
                        Wait(4);
                        return;
                    }
                case 0x23:
                    {
                        var val = Hl + 1;
                        _registers[HIdx] = (byte)(val >> 8);
                        _registers[LIdx] = (byte)(val & 0xFF);
#if (DEBUG)
                        _logger.Log("INC HL");
#endif
                        Wait(4);
                        return;
                    }
                case 0x33:
                    {
                        var val = Sp + 1;
                        _registers[SpIdx] = (byte)(val >> 8);
                        _registers[SpIdx + 1] = (byte)(val & 0xFF);
#if (DEBUG)
                        _logger.Log("INC SP");
#endif
                        Wait(4);
                        return;
                    }
                case 0x0B:
                    {
                        var val = Bc - 1;
                        _registers[BIdx] = (byte)(val >> 8);
                        _registers[CIdx] = (byte)(val & 0xFF);
#if (DEBUG)
                        _logger.Log("DEC BC");
#endif
                        Wait(4);
                        return;
                    }
                case 0x1B:
                    {
                        var val = De - 1;
                        _registers[DIdx] = (byte)(val >> 8);
                        _registers[EIdx] = (byte)(val & 0xFF);
#if (DEBUG)
                        _logger.Log("DEC DE");
#endif
                        Wait(4);
                        return;
                    }
                case 0x2B:
                    {
                        var val = Hl - 1;
                        _registers[HIdx] = (byte)(val >> 8);
                        _registers[LIdx] = (byte)(val & 0xFF);
#if (DEBUG)
                        _logger.Log("DEC HL");
#endif
                        Wait(4);
                        return;
                    }
                case 0x3B:
                    {
                        var val = Sp - 1;
                        _registers[SpIdx] = (byte)(val >> 8);
                        _registers[SpIdx + 1] = (byte)(val & 0xFF);
#if (DEBUG)
                        _logger.Log("DEC SP");
#endif
                        Wait(4);
                        return;
                    }
                case 0x07:
                    {
                        var a = _registers[AIdx];
                        var c = (byte)((a & 0x80) >> 7);
                        a <<= 1;
                        _registers[AIdx] = a;
                        _registers[FIdx] &= (byte)~(Fl.H | Fl.N | Fl.C);
                        _registers[FIdx] |= c;
#if (DEBUG)
                        _logger.Log("RLCA");
#endif
                        Wait(4);
                        return;
                    }
                case 0x17:
                    {
                        var a = _registers[AIdx];
                        var c = (byte)((a & 0x80) >> 7);
                        a <<= 1;
                        var f = _registers[FIdx];
                        a |= (byte)(f & (byte)Fl.C);
                        _registers[AIdx] = a;
                        f &= (byte)~(Fl.H | Fl.N | Fl.C);
                        f |= c;
                        _registers[FIdx] = f;
#if (DEBUG)
                        _logger.Log("RLA");
#endif
                        Wait(4);
                        return;
                    }
                case 0x0F:
                    {
                        var a = _registers[AIdx];
                        var c = (byte)(a & 0x01);
                        a >>= 1;
                        _registers[AIdx] = a;
                        _registers[FIdx] &= (byte)~(Fl.H | Fl.N | Fl.C);
                        _registers[FIdx] |= c;
#if (DEBUG)
                        _logger.Log("RRCA");
#endif
                        Wait(4);
                        return;
                    }
                case 0x1F:
                    {
                        var a = _registers[AIdx];
                        var c = (byte)(a & 0x01);
                        a >>= 1;
                        var f = _registers[FIdx];
                        a |= (byte)((f & (byte)Fl.C) << 7);
                        _registers[AIdx] = a;
                        f &= (byte)~(Fl.H | Fl.N | Fl.C);
                        f |= c;
                        _registers[FIdx] = f;
#if (DEBUG)
                        _logger.Log("RRA");
#endif
                        Wait(4);
                        return;
                    }
                case 0xC3:
                    {
                        var addr = Fetch16();
                        _registers[PcIdx] = (byte)(addr >> 8);
                        _registers[PcIdx + 1] = (byte)(addr);
#if (DEBUG)
                        _logger.Log($"JP 0x{addr:X4}");
#endif
                        Wait(10);
                        return;
                    }
                case 0xC2:
                case 0xCA:
                case 0xD2:
                case 0xDA:
                case 0xE2:
                case 0xEA:
                case 0xF2:
                case 0xFA:
                    {
                        var addr = Fetch16();
                        if (JumpCondition(r))
                        {
                            _registers[PcIdx] = (byte)(addr >> 8);
                            _registers[PcIdx + 1] = (byte)(addr);
                        }
#if (DEBUG)
                        _logger.Log($"JP {JCName(r)}, 0x{addr:X4}");
#endif
                        Wait(10);
                        return;

                    }
                case 0x18:
                    {
                        // order is important here
                        var d = (sbyte)Fetch();
                        var addr = Pc + d;
                        _registers[PcIdx] = (byte)(addr >> 8);
                        _registers[PcIdx + 1] = (byte)(addr);
#if (DEBUG)
                        _logger.Log($"JR 0x{addr:X4}");
#endif
                        Wait(12);
                        return;
                    }
                case 0x20:
                case 0x28:
                case 0x30:
                case 0x38:
                    {
                        // order is important here
                        var d = (sbyte)Fetch();
                        var addr = Pc + d;
                        if (JumpCondition((byte)(r & 3)))
                        {
                            _registers[PcIdx] = (byte)(addr >> 8);
                            _registers[PcIdx + 1] = (byte)(addr);
                            Wait(12);
                        }
                        else
                        {
                            Wait(7);
                        }
#if (DEBUG)
                        _logger.Log($"JR {JCName((byte)(r & 3))}, 0x{addr:X4}");
#endif
                        return;

                    }
                case 0xE9:
                    {
                        var addr = Hl;
                        _registers[PcIdx] = (byte)(addr >> 8);
                        _registers[PcIdx + 1] = (byte)(addr);
#if (DEBUG)
                        _logger.Log("JP HL");
#endif
                        Wait(4);
                        return;
                    }
                case 0x10:
                    {
                        // order is important here
                        var d = (sbyte)Fetch();
                        var addr = Pc + d;
                        var b = _registers[BIdx];
                        _registers[BIdx] = --b;
                        if (b != 0)
                        {
                            _registers[PcIdx] = (byte)(addr >> 8);
                            _registers[PcIdx + 1] = (byte)(addr);
                            Wait(13);
                        }
                        else
                        {
                            Wait(8);
                        }
#if (DEBUG)
                        _logger.Log($"DJNZ 0x{addr:X4}");
#endif
                        return;
                    }
                case 0xCD:
                    {
                        var addr = Fetch16();
                        var stack = Sp;
                        mem[--stack] = (byte)(Pc >> 8);
                        mem[--stack] = (byte)(Pc);
                        _registers[SpIdx] = (byte)(stack >> 8);
                        _registers[SpIdx + 1] = (byte)(stack);
                        _registers[PcIdx] = (byte)(addr >> 8);
                        _registers[PcIdx + 1] = (byte)(addr);
#if (DEBUG)
                        _logger.Log($"CALL 0x{addr:X4}");
#endif
                        Wait(17);
                        return;
                    }
                case 0xC4:
                case 0xCC:
                case 0xD4:
                case 0xDC:
                case 0xE4:
                case 0xEC:
                case 0xF4:
                case 0xFC:
                    {
                        var addr = Fetch16();
                        if (JumpCondition(r))
                        {
                            var stack = Sp;
                            mem[--stack] = (byte)(Pc >> 8);
                            mem[--stack] = (byte)(Pc);
                            _registers[SpIdx] = (byte)(stack >> 8);
                            _registers[SpIdx + 1] = (byte)(stack);
                            _registers[PcIdx] = (byte)(addr >> 8);
                            _registers[PcIdx + 1] = (byte)(addr);
                            Wait(17);
                        }
                        else
                        {
                            Wait(10);
                        }
#if (DEBUG)
                        _logger.Log($"CALL {JCName(r)}, 0x{addr:X4}");
#endif
                        return;

                    }
                case 0xC9:
                    {
                        var stack = Sp;
                        _registers[PcIdx + 1] = mem[stack++];
                        _registers[PcIdx] = mem[stack++];
                        _registers[SpIdx] = (byte)(stack >> 8);
                        _registers[SpIdx + 1] = (byte)(stack);
#if (DEBUG)
                        _logger.Log("RET");
#endif
                        Wait(10);
                        return;
                    }
                case 0xC0:
                case 0xC8:
                case 0xD0:
                case 0xD8:
                case 0xE0:
                case 0xE8:
                case 0xF0:
                case 0xF8:
                    {
                        if (JumpCondition(r))
                        {
                            var stack = Sp;
                            _registers[PcIdx + 1] = mem[stack++];
                            _registers[PcIdx] = mem[stack++];
                            _registers[SpIdx] = (byte)(stack >> 8);
                            _registers[SpIdx + 1] = (byte)(stack);
                            Wait(11);
                        }
                        else
                        {
                            Wait(5);
                        }
#if (DEBUG)
                        _logger.Log($"RET {JCName(r)}");
#endif
                        return;

                    }
                case 0xC7:
                case 0xCF:
                case 0xD7:
                case 0xDF:
                case 0xE7:
                case 0xEF:
                case 0xF7:
                case 0xFF:
                    {
                        var stack = Sp;
                        mem[--stack] = (byte)(Pc >> 8);
                        mem[--stack] = (byte)(Pc);
                        _registers[SpIdx] = (byte)(stack >> 8);
                        _registers[SpIdx + 1] = (byte)(stack);
                        _registers[PcIdx] = 0;
                        _registers[PcIdx + 1] = (byte)(mc & 0x38);
#if (DEBUG)
                        _logger.Log($"RST 0x{mc & 0x38:X4}");
#endif
                        Wait(17);
                        return;
                    }
                case 0xDB:
                    {
                        var port = Fetch() + (_registers[AIdx] << 8);
                        _registers[AIdx] = ports.ReadPort((ushort)port);
#if (DEBUG)
                        _logger.Log($"IN A, (0x{port:X2})");
#endif
                        Wait(11);
                        return;
                    }
                case 0xD3:
                    {
                        var port = Fetch() + (_registers[AIdx] << 8);
                        ports.WritePort((ushort)port, _registers[AIdx]);
#if (DEBUG)
                        _logger.Log($"OUT (0x{port:X2}), A");
#endif
                        Wait(11);
                        return;
                    }
            }

#if(DEBUG)
            _logger.Log($"{mc:X2}: {hi:X} {r:X} {lo:X}");
            //throw new InvalidOperationException("Invalid Opcode: "+mc.ToString("X2"));
#endif
            Halt = true;
        }

        private string JCName(byte condition)
        {
            switch (condition)
            {
                case 0:
                    return "NZ";
                case 1:
                    return "Z";
                case 2:
                    return "NC";
                case 3:
                    return "C";
                case 4:
                    return "PO";
                case 5:
                    return "PE";
                case 6:
                    return "P";
                case 7:
                    return "M";
            }
            return "";
        }

        private void ParseCB(byte mode = 0)
        {
            sbyte d = 0;
            if (mode != 0)
            {
                d = (sbyte)Fetch();
            }
            if (Halt) return;
            var mc = Fetch();
            var hi = (byte)(mc >> 6);
            var lo = (byte)(mc & 0x07);
            var r = (byte)((mc >> 3) & 0x07);
            var useHL = lo == 6;
            var useIX = mode == 0xDD;
            var useIY = mode == 0XFD;
            var reg = useHL ? useIX ? mem[(ushort)(Ix + d)] : useIY ? mem[(ushort)(Iy + d)] : mem[Hl] : _registers[lo];
#if (DEBUG)
            string debug_target;
            if (useHL)
                if (useIX) debug_target = $"(IX{d:+0;-#})";
                else debug_target = useIY ? $"(IY{d:+0;-#})" : "(HL)";
            else
                debug_target = useIX ? $"(IX{d:+0;-#}), {_logger.RName(lo)}" : useIY ? $"(IY{d:+0;-#}), {_logger.RName(lo)}" : _logger.RName(lo);
#endif
            switch (hi)
            {
                case 0:
                    byte c;
                    if ((r & 1) == 1)
                    {
                        c = (byte)(reg & 0x01);
                        reg >>= 1;
                    }
                    else
                    {
                        c = (byte)((reg & 0x80) >> 7);
                        reg <<= 1;
                    }
                    var f = _registers[FIdx];
                    switch (r)
                    {
                        case 0:
                            {
                                reg |= c;
#if (DEBUG)
                                _logger.Log($"RLC {debug_target}");
#endif
                                break;
                            }
                        case 1:
                            {
                                reg |= (byte)(c << 7);
#if (DEBUG)
                                _logger.Log($"RRC {debug_target}");
#endif
                                break;
                            }
                        case 2:
                            {
                                reg |= (byte)(f & (byte)Fl.C);
#if (DEBUG)
                                _logger.Log($"RL {debug_target}");
#endif
                                break;
                            }
                        case 3:
                            {
                                reg |= (byte)((f & (byte)Fl.C) << 7);
#if (DEBUG)
                                _logger.Log($"RR {debug_target}");
#endif
                                break;
                            }
                        case 4:
                            {
#if (DEBUG)
                                _logger.Log($"SLA {debug_target}");
#endif
                                break;
                            }
                        case 5:
                            {
                                reg |= (byte)((reg & 0x40) << 1);
#if (DEBUG)
                                _logger.Log($"SRA {debug_target}");

#endif
                                break;
                            }
                        case 6:
                            {
                                reg |= 1;
#if (DEBUG)
                                _logger.Log($"SLL {debug_target}");
#endif
                                break;
                            }
                        case 7:
                            {
#if (DEBUG)
                                _logger.Log($"SRL {debug_target}");
#endif
                                break;
                            }
                    }
                    f &= (byte)~(Fl.H | Fl.N | Fl.C | Fl.PV | Fl.S | Fl.Z);
                    f |= (byte)(reg & (byte)Fl.S);
                    if (reg == 0) f |= (byte)Fl.Z;
                    if (Parity(reg)) f |= (byte)Fl.PV;
                    f |= c;
                    _registers[FIdx] = f;

                    break;
                case 1:
                    {
                        Bit(r, reg);
#if (DEBUG)
                        _logger.Log($"BIT {r}, {debug_target}");
#endif
                        Wait(useHL ? 12 : 8);
                        return;
                    }
                case 2:
                    reg &= (byte)~(0x01 << r);
#if (DEBUG)
                    _logger.Log($"RES {r}, {debug_target}");
#endif
                    Wait(useHL ? 12 : 8);
                    break;
                case 3:
                    reg |= (byte)(0x01 << r);
#if (DEBUG)
                    _logger.Log($"SET {r}, {debug_target}");
#endif
                    Wait(useHL ? 12 : 8);
                    break;
            }
            if (useHL)
            {
                if (useIX)
                {
                    mem[(ushort)(Ix + d)] = reg;
                    Wait(23);
                }
                else if (useIY)
                {
                    mem[(ushort)(Iy + d)] = reg;
                    Wait(23);
                }
                else
                {
                    mem[Hl] = reg;
                    Wait(15);
                }
            }
            else
            {
                if (useIX)
                {
                    mem[(ushort)(Ix + d)] = reg;
                    Wait(23);
                }
                else if (useIY)
                {
                    mem[(ushort)(Iy + d)] = reg;
                    Wait(23);
                }
                _registers[lo] = reg;
                Wait(8);
            }
        }

        private void Bit(byte bit, byte value)
        {
            var f = (byte)(_registers[FIdx] & (byte)~(Fl.Z | Fl.H | Fl.N));
            if ((value & (0x01 << bit)) == 0) f |= (byte)Fl.Z;
            f |= (byte)Fl.H;
            _registers[FIdx] = f;
        }

        private void AddHl(ushort value)
        {
            var sum = Add(Hl, value);
            _registers[HIdx] = (byte)(sum >> 8);
            _registers[LIdx] = (byte)(sum & 0xFF);
        }

        private void AddIx(ushort value)
        {
            var sum = Add(Ix, value);
            _registers[IxIdx] = (byte)(sum >> 8);
            _registers[IxIdx + 1] = (byte)(sum & 0xFF);
        }

        private void AddIy(ushort value)
        {
            var sum = Add(Iy, value);
            _registers[IyIdx] = (byte)(sum >> 8);
            _registers[IyIdx + 1] = (byte)(sum & 0xFF);
        }

        private ushort Add(ushort value1, ushort value2)
        {
            var sum = value1 + value2;
            var f = (byte)(_registers[FIdx] & (byte)~(Fl.H | Fl.N | Fl.C));
            if ((value1 & 0x0FFF) + (value2 & 0x0FFF) > 0x0FFF)
                f |= (byte)Fl.H;
            if (sum > 0xFFFF)
                f |= (byte)Fl.C;
            _registers[FIdx] = f;
            return (ushort)sum;
        }

        private void AdcHl(ushort value)
        {
            var sum = Adc(Hl, value);
            _registers[HIdx] = (byte)(sum >> 8);
            _registers[LIdx] = (byte)(sum & 0xFF);
        }

        private ushort Adc(ushort value1, ushort value2)
        {
            var sum = value1 + value2 + (_registers[FIdx] & (byte)Fl.C);
            var f = (byte)(_registers[FIdx] & (byte)~(Fl.S | Fl.Z | Fl.H | Fl.PV | Fl.N | Fl.C));
            if ((short)sum < 0)
                f |= (byte)Fl.S;
            if (sum == 0)
                f |= (byte)Fl.Z;
            if ((value1 & 0x0FFF) + (value2 & 0x0FFF) + (byte)Fl.C > 0x0FFF)
                f |= (byte)Fl.H;
            if (sum > 0x7FFF)
                f |= (byte)Fl.PV;
            if (sum > 0xFFFF)
                f |= (byte)Fl.C;
            _registers[FIdx] = f;
            return (ushort)sum;
        }

        private void SbcHl(ushort value)
        {
            var sum = Sbc(Hl, value);
            _registers[HIdx] = (byte)(sum >> 8);
            _registers[LIdx] = (byte)(sum & 0xFF);
        }


        private ushort Sbc(ushort value1, ushort value2)
        {
            var diff = value1 - value2 - (_registers[FIdx] & (byte)Fl.C);
            var f = (byte)(_registers[FIdx] & (byte)~(Fl.S | Fl.Z | Fl.H | Fl.PV | Fl.N | Fl.C));
            if ((short)diff < 0)
                f |= (byte)Fl.S;
            if (diff == 0)
                f |= (byte)Fl.Z;
            if ((value1 & 0xFFF) < (value2 & 0xFFF) + (_registers[FIdx] & (byte)Fl.C))
                f |= (byte)Fl.H;
            if (diff > short.MaxValue || diff < short.MinValue)
                f |= (byte)Fl.PV;
            if ((ushort)diff > value1)
                f |= (byte)Fl.C;
            _registers[FIdx] = f;
            return (ushort)diff;
        }

        private void ParseED()
        {
            if (Halt) return;
            var mc = Fetch();
            var r = (byte)((mc >> 3) & 0x07);

            switch (mc)
            {
                case 0x47:
                    {
                        // LD I, A
                        _registers[IIdx] = _registers[AIdx];
#if (DEBUG)
                        _logger.Log("LD I, A");
#endif
                        Wait(9);
                        return;
                    }
                case 0x4F:
                    {
                        // LD R, A
                        _registers[RIdx] = _registers[AIdx];
#if (DEBUG)
                        _logger.Log("LD R, A");
#endif
                        Wait(9);
                        return;
                    }
                case 0x57:
                    {
                        // LD A, I

                        /*
                                     * Condition Bits Affected
                                     * S is set if the I Register is negative; otherwise, it is reset.
                                     * Z is set if the I Register is 0; otherwise, it is reset.
                                     * H is reset.
                                     * P/V contains contents of IFF2.
                                     * N is reset.
                                     * C is not affected.
                                     * If an interrupt occurs during execution of this instruction, the Parity flag contains a 0.
                                     */
                        var i = _registers[IIdx];
                        _registers[AIdx] = i;
                        var f = (byte)(_registers[FIdx] & (~(byte)(Fl.H | Fl.PV | Fl.N | Fl.S | Fl.Z | Fl.PV)));
                        if (i >= 0x80)
                        {
                            f |= (byte)Fl.S;
                        }
                        else if (i == 0x00)
                        {
                            f |= (byte)Fl.Z;
                        }
                        if (IFF2)
                        {
                            f |= (byte)Fl.PV;
                        }
                        _registers[FIdx] = f;
#if (DEBUG)
                        _logger.Log("LD A, I");
#endif
                        Wait(9);
                        return;
                    }
                case 0x5F:
                    {
                        // LD A, R

                        /*
                                     * Condition Bits Affected
                                     * S is set if, R-Register is negative; otherwise, it is reset.
                                     * Z is set if the R Register is 0; otherwise, it is reset.
                                     * H is reset.
                                     * P/V contains contents of IFF2.
                                     * N is reset.
                                     * C is not affected.
                                     * If an interrupt occurs during execution of this instruction, the parity flag contains a 0. 
                                     */
                        var reg = _registers[RIdx];
                        _registers[AIdx] = reg;
                        var f = (byte)(_registers[FIdx] & (~(byte)(Fl.H | Fl.PV | Fl.N | Fl.S | Fl.Z | Fl.PV)));
                        if (reg >= 0x80)
                        {
                            f |= (byte)Fl.S;
                        }
                        else if (reg == 0x00)
                        {
                            f |= (byte)Fl.Z;
                        }
                        if (IFF2)
                        {
                            f |= (byte)Fl.PV;
                        }
                        _registers[FIdx] = f;
#if (DEBUG)
                        _logger.Log("LD A, R");
#endif
                        Wait(9);
                        return;
                    }
                case 0x4B:
                    {
                        // LD BC, (nn)
                        var addr = Fetch16();
                        _registers[CIdx] = mem[addr++];
                        _registers[BIdx] = mem[addr];
#if (DEBUG)
                        _logger.Log($"LD BC, (0x{--addr:X4})");
#endif
                        Wait(20);
                        return;
                    }
                case 0x5B:
                    {
                        // LD DE, (nn)
                        var addr = Fetch16();
                        _registers[EIdx] = mem[addr++];
                        _registers[DIdx] = mem[addr];
#if (DEBUG)
                        _logger.Log($"LD DE, (0x{--addr:X4})");
#endif
                        Wait(20);
                        return;
                    }
                case 0x6B:
                    {
                        // LD HL, (nn)
                        var addr = Fetch16();
                        _registers[LIdx] = mem[addr++];
                        _registers[HIdx] = mem[addr];
#if (DEBUG)
                        _logger.Log($"LD HL, (0x{--addr:X4})*");
#endif
                        Wait(20);
                        return;
                    }
                case 0x7B:
                    {
                        // LD SP, (nn)
                        var addr = Fetch16();
                        _registers[SpIdx + 1] = mem[addr++];
                        _registers[SpIdx] = mem[addr];
#if (DEBUG)
                        _logger.Log($"LD SP, (0x{--addr:X4})");
#endif
                        Wait(20);
                        return;
                    }
                case 0x43:
                    {
                        // LD (nn), BC
                        var addr = Fetch16();
                        mem[addr++] = _registers[CIdx];
                        mem[addr] = _registers[BIdx];
#if (DEBUG)
                        _logger.Log($"LD (0x{--addr:X4}), BC");
#endif
                        Wait(20);
                        return;
                    }
                case 0x53:
                    {
                        // LD (nn), DE
                        var addr = Fetch16();
                        mem[addr++] = _registers[EIdx];
                        mem[addr] = _registers[DIdx];
#if (DEBUG)
                        _logger.Log($"LD (0x{--addr:X4}), DE");
#endif
                        Wait(20);
                        return;
                    }
                case 0x63:
                    {
                        // LD (nn), HL
                        var addr = Fetch16();
                        mem[addr++] = _registers[LIdx];
                        mem[addr] = _registers[HIdx];
#if (DEBUG)
                        _logger.Log($"LD (0x{--addr:X4}), HL");
#endif
                        Wait(20);
                        return;
                    }
                case 0x73:
                    {
                        // LD (nn), SP
                        var addr = Fetch16();
                        mem[addr++] = _registers[SpIdx + 1];
                        mem[addr] = _registers[SpIdx];
#if (DEBUG)
                        _logger.Log($"LD (0x{--addr:X4}), SP");
#endif
                        Wait(20);
                        return;
                    }
                case 0xA0:
                    {
                        // LDI
                        var bc = Bc;
                        var de = De;
                        var hl = Hl;

                        mem[de] = mem[hl];
                        de++;
                        hl++;
                        bc--;

                        _registers[BIdx] = (byte)(bc >> 8);
                        _registers[CIdx] = (byte)(bc & 0xFF);
                        _registers[DIdx] = (byte)(de >> 8);
                        _registers[EIdx] = (byte)(de & 0xFF);
                        _registers[HIdx] = (byte)(hl >> 8);
                        _registers[LIdx] = (byte)(hl & 0xFF);

                        var f = (byte)(_registers[FIdx] & 0xE9);
                        if (bc != 0) f = (byte)(f | 0x04);
                        _registers[FIdx] = f;
#if (DEBUG)
                        _logger.Log("LDI");
#endif
                        Wait(16);
                        return;
                    }
                case 0xB0:
                    {
                        // LDIR
                        var bc = Bc;
                        var de = De;
                        var hl = Hl;

                        mem[de] = mem[hl];
                        de++;
                        hl++;
                        bc--;

                        _registers[BIdx] = (byte)(bc >> 8);
                        _registers[CIdx] = (byte)(bc & 0xFF);
                        _registers[DIdx] = (byte)(de >> 8);
                        _registers[EIdx] = (byte)(de & 0xFF);
                        _registers[HIdx] = (byte)(hl >> 8);
                        _registers[LIdx] = (byte)(hl & 0xFF);

                        _registers[FIdx] = (byte)(_registers[FIdx] & 0xE9);
                        if (bc != 0)
                        {
                            var pc = (ushort)((_registers[PcIdx] << 8) + _registers[PcIdx + 1]);
                            // jumps back to itself
                            pc -= 2;
                            _registers[PcIdx] = (byte)(pc >> 8);
                            _registers[PcIdx + 1] = (byte)(pc & 0xFF);
                            Wait(21);
                            return;
                        }
#if (DEBUG)
                        _logger.Log("LDIR");
#endif
                        Wait(16);
                        return;
                    }
                case 0xA8:
                    {
                        // LDD
                        var bc = Bc;
                        var de = De;
                        var hl = Hl;

                        mem[de] = mem[hl];
                        de--;
                        hl--;
                        bc--;

                        _registers[BIdx] = (byte)(bc >> 8);
                        _registers[CIdx] = (byte)(bc & 0xFF);
                        _registers[DIdx] = (byte)(de >> 8);
                        _registers[EIdx] = (byte)(de & 0xFF);
                        _registers[HIdx] = (byte)(hl >> 8);
                        _registers[LIdx] = (byte)(hl & 0xFF);

                        var f = (byte)(_registers[FIdx] & 0xE9);
                        if (bc != 0) f = (byte)(f | 0x04);
                        _registers[FIdx] = f;
#if (DEBUG)
                        _logger.Log("LDD");
#endif
                        Wait(16);
                        return;
                    }
                case 0xB8:
                    {
                        // LDDR
                        var bc = Bc;
                        var de = De;
                        var hl = Hl;

                        mem[de] = mem[hl];
                        de--;
                        hl--;
                        bc--;

                        _registers[BIdx] = (byte)(bc >> 8);
                        _registers[CIdx] = (byte)(bc & 0xFF);
                        _registers[DIdx] = (byte)(de >> 8);
                        _registers[EIdx] = (byte)(de & 0xFF);
                        _registers[HIdx] = (byte)(hl >> 8);
                        _registers[LIdx] = (byte)(hl & 0xFF);

                        _registers[FIdx] = (byte)(_registers[FIdx] & 0xE9);
                        if (bc != 0)
                        {
                            var pc = (ushort)((_registers[PcIdx] << 8) + _registers[PcIdx + 1]);
                            // jumps back to itself
                            pc -= 2;
                            _registers[PcIdx] = (byte)(pc >> 8);
                            _registers[PcIdx + 1] = (byte)(pc & 0xFF);
                            Wait(21);
                            return;
                        }
#if (DEBUG)
                        _logger.Log("LDDR");
#endif
                        Wait(16);
                        return;
                    }

                case 0xA1:
                    {
                        // CPI
                        var bc = Bc;
                        var hl = Hl;

                        var a = _registers[AIdx];
                        var b = mem[hl];
                        hl++;
                        bc--;

                        _registers[BIdx] = (byte)(bc >> 8);
                        _registers[CIdx] = (byte)(bc & 0xFF);
                        _registers[HIdx] = (byte)(hl >> 8);
                        _registers[LIdx] = (byte)(hl & 0xFF);

                        var f = (byte)(_registers[FIdx] & 0x2A);
                        if (a < b) f = (byte)(f | 0x80);
                        if (a == b) f = (byte)(f | 0x40);
                        if ((a & 8) < (b & 8)) f = (byte)(f | 0x10);
                        if (bc != 0) f = (byte)(f | 0x04);
                        _registers[FIdx] = (byte)(f | 0x02);
#if (DEBUG)
                        _logger.Log("CPI");
#endif
                        Wait(16);
                        return;
                    }

                case 0xB1:
                    {
                        // CPIR
                        var bc = Bc;
                        var hl = Hl;

                        var a = _registers[AIdx];
                        var b = mem[hl];
                        hl++;
                        bc--;

                        _registers[BIdx] = (byte)(bc >> 8);
                        _registers[CIdx] = (byte)(bc & 0xFF);
                        _registers[HIdx] = (byte)(hl >> 8);
                        _registers[LIdx] = (byte)(hl & 0xFF);

                        if (a == b || bc == 0)
                        {
                            var f = (byte)(_registers[FIdx] & 0x2A);
                            if (a < b) f = (byte)(f | 0x80);
                            if (a == b) f = (byte)(f | 0x40);
                            if ((a & 8) < (b & 8)) f = (byte)(f | 0x10);
                            if (bc != 0) f = (byte)(f | 0x04);
                            _registers[FIdx] = (byte)(f | 0x02);
#if (DEBUG)
                            _logger.Log("CPIR");
#endif
                            Wait(16);
                            return;
                        }

                        var pc = (ushort)((_registers[PcIdx] << 8) + _registers[PcIdx + 1]);
                        // jumps back to itself
                        pc -= 2;
                        _registers[PcIdx] = (byte)(pc >> 8);
                        _registers[PcIdx + 1] = (byte)(pc & 0xFF);
                        Wait(21);
                        return;
                    }

                case 0xA9:
                    {
                        // CPD
                        var bc = Bc;
                        var hl = Hl;

                        var a = _registers[AIdx];
                        var b = mem[hl];
                        hl--;
                        bc--;

                        _registers[BIdx] = (byte)(bc >> 8);
                        _registers[CIdx] = (byte)(bc & 0xFF);
                        _registers[HIdx] = (byte)(hl >> 8);
                        _registers[LIdx] = (byte)(hl & 0xFF);

                        var f = (byte)(_registers[FIdx] & 0x2A);
                        if (a < b) f = (byte)(f | 0x80);
                        if (a == b) f = (byte)(f | 0x40);
                        if ((a & 8) < (b & 8)) f = (byte)(f | 0x10);
                        if (bc != 0) f = (byte)(f | 0x04);
                        _registers[FIdx] = (byte)(f | 0x02);
#if (DEBUG)
                        _logger.Log("CPD");
#endif
                        Wait(16);
                        return;
                    }

                case 0xB9:
                    {
                        // CPDR
                        var bc = Bc;
                        var hl = Hl;

                        var a = _registers[AIdx];
                        var b = mem[hl];
                        hl--;
                        bc--;

                        _registers[BIdx] = (byte)(bc >> 8);
                        _registers[CIdx] = (byte)(bc & 0xFF);
                        _registers[HIdx] = (byte)(hl >> 8);
                        _registers[LIdx] = (byte)(hl & 0xFF);

                        if (a == b || bc == 0)
                        {
                            var f = (byte)(_registers[FIdx] & 0x2A);
                            if (a < b) f = (byte)(f | 0x80);
                            if (a == b) f = (byte)(f | 0x40);
                            if ((a & 8) < (b & 8)) f = (byte)(f | 0x10);
                            if (bc != 0) f = (byte)(f | 0x04);
                            _registers[FIdx] = (byte)(f | 0x02);
#if (DEBUG)
                            _logger.Log("CPDR");
#endif
                            Wait(21);
                            return;
                        }

                        var pc = (ushort)((_registers[PcIdx] << 8) + _registers[PcIdx + 1]);
                        // jumps back to itself
                        pc -= 2;
                        _registers[PcIdx] = (byte)(pc >> 8);
                        _registers[PcIdx + 1] = (byte)(pc & 0xFF);
                        Wait(21);
                        return;
                    }
                case 0x44:
                case 0x54:
                case 0x64:
                case 0x74:
                case 0x4C:
                case 0x5C:
                case 0x6C:
                case 0x7C:
                    {
                        // NEG
                        var a = _registers[AIdx];
                        var diff = -a;
                        _registers[AIdx] = (byte)diff;

                        var f = (byte)(_registers[FIdx] & 0x28);
                        if ((diff & 0x80) > 0) f |= (byte)Fl.S;
                        if (diff == 0) f |= (byte)Fl.Z;
                        if ((a & 0xF) != 0) f |= (byte)Fl.H;
                        if (a == 0x80) f |= (byte)Fl.PV;
                        f |= (byte)Fl.N;
                        if (diff != 0) f |= (byte)Fl.C;
                        _registers[FIdx] = f;


#if (DEBUG)
                        _logger.Log("NEG");
#endif
                        Wait(8);
                        return;
                    }
                case 0x46:
                case 0x66:
                    {
                        // IM 0
                        interruptMode = 0;
#if (DEBUG)
                        _logger.Log("IM 0");
#endif
                        Wait(8);
                        return;
                    }
                case 0x56:
                case 0x76:
                    {
                        // IM 1
                        interruptMode = 1;
#if (DEBUG)
                        _logger.Log("IM 1");
#endif
                        Wait(8);
                        return;
                    }
                case 0x5E:
                case 0x7E:
                    {
                        // IM 2
                        interruptMode = 2;
#if (DEBUG)
                        _logger.Log("IM 2");
#endif
                        Wait(8);
                        return;
                    }
                case 0x4A:
                    {
                        AdcHl(Bc);

#if (DEBUG)
                        _logger.Log("ADC HL, BC");
#endif
                        Wait(15);
                        return;
                    }
                case 0x5A:
                    {
                        AdcHl(De);
#if (DEBUG)
                        _logger.Log("ADC HL, DE");
#endif
                        Wait(15);
                        return;
                    }
                case 0x6A:
                    {
                        AdcHl(Hl);
#if (DEBUG)
                        _logger.Log("ADC HL, HL");
#endif
                        Wait(15);
                        return;
                    }
                case 0x7A:
                    {
                        AdcHl(Sp);
#if (DEBUG)
                        _logger.Log("ADC HL, SP");
#endif
                        Wait(15);
                        return;
                    }
                case 0x42:
                    {
                        SbcHl(Bc);

#if (DEBUG)
                        _logger.Log("SBC HL, BC");
#endif
                        Wait(15);
                        return;
                    }
                case 0x52:
                    {
                        SbcHl(De);
#if (DEBUG)
                        _logger.Log("SBC HL, DE");
#endif
                        Wait(15);
                        return;
                    }
                case 0x62:
                    {
                        SbcHl(Hl);
#if (DEBUG)
                        _logger.Log("SBC HL, HL");
#endif
                        Wait(15);
                        return;
                    }
                case 0x72:
                    {
                        SbcHl(Sp);
#if (DEBUG)
                        _logger.Log("SBC HL, SP");
#endif
                        Wait(15);
                        return;
                    }

                case 0x6F:
                    {
                        var a = _registers[AIdx];
                        var b = mem[Hl];
                        mem[Hl] = (byte)((b << 4) | (a & 0x0F));
                        a = (byte)((a & 0xF0) | (b >> 4));
                        _registers[AIdx] = a;
                        var f = (byte)(_registers[FIdx] & 0x29);
                        if ((a & 0x80) > 0) f |= (byte)Fl.S;
                        if (a == 0) f |= (byte)Fl.Z;
                        if (Parity(a)) f |= (byte)Fl.PV;
                        _registers[FIdx] = f;
#if (DEBUG)
                        _logger.Log("RLD");
#endif
                        Wait(18);
                        return;
                    }
                case 0x67:
                    {
                        var a = _registers[AIdx];
                        var b = mem[Hl];
                        mem[Hl] = (byte)((b >> 4) | (a << 4));
                        a = (byte)((a & 0xF0) | (b & 0x0F));
                        _registers[AIdx] = a;
                        var f = (byte)(_registers[FIdx] & 0x29);
                        if ((a & 0x80) > 0) f |= (byte)Fl.S;
                        if (a == 0) f |= (byte)Fl.Z;
                        if (Parity(a)) f |= (byte)Fl.PV;
                        _registers[FIdx] = f;
#if (DEBUG)
                        _logger.Log("RRD");
#endif
                        Wait(18);
                        return;
                    }
                case 0x45:
                case 0x4D:
                case 0x55:
                case 0x5D:
                case 0x65:
                case 0x6D:
                case 0x75:
                case 0x7D:
                    {
                        var stack = Sp;
                        _registers[PcIdx + 1] = mem[stack++];
                        _registers[PcIdx] = mem[stack++];
                        _registers[SpIdx] = (byte)(stack >> 8);
                        _registers[SpIdx + 1] = (byte)(stack);
                        IFF1 = IFF2;
#if (DEBUG)
                        if (mc == 0x4D)
                            _logger.Log("RETN");
                        else
                            _logger.Log("RETI");
#endif
                        Wait(10);
                        return;
                    }

                case 0x77:
                case 0x7F:
                    {
#if (DEBUG)
                        _logger.Log("NOP");
#endif
                        Wait(8);
                        return;
                    }
                case 0x40:
                case 0x48:
                case 0x50:
                case 0x58:
                case 0x60:
                case 0x68:
                case 0x78:
                    {
                        var a = ports.ReadPort(Bc);
                        _registers[r] = a;
                        var f = (byte)(_registers[FIdx] & 0x29);
                        if ((a & 0x80) > 0) f |= (byte)Fl.S;
                        if (a == 0) f |= (byte)Fl.Z;
                        if (Parity(a)) f |= (byte)Fl.PV;
                        _registers[FIdx] = f;
#if (DEBUG)
                        _logger.Log($"IN {_logger.RName(r)}, (BC)");
#endif
                        Wait(8);
                        return;
                    }
                case 0xA2:
                    {
                        var a = ports.ReadPort(Bc);
                        var hl = Hl;
                        mem[hl++] = a;
                        _registers[HIdx] = (byte)(hl >> 8);
                        _registers[LIdx] = (byte)hl;
                        var b = (byte)(_registers[BIdx] - 1);
                        _registers[BIdx] = b;
                        var f = (byte)(_registers[FIdx] & (byte)~(Fl.N | Fl.Z));
                        if (b == 0) f |= (byte)Fl.Z;
                        f |= (byte)Fl.N;
                        _registers[FIdx] = f;

#if (DEBUG)
                        _logger.Log("INI");
#endif
                        Wait(16);
                        return;
                    }
                case 0xB2:
                    {
                        var a = ports.ReadPort(Bc);
                        var hl = Hl;
                        mem[hl++] = a;
                        _registers[HIdx] = (byte)(hl >> 8);
                        _registers[LIdx] = (byte)hl;
                        var b = (byte)(_registers[BIdx] - 1);
                        _registers[BIdx] = b;
                        if (b != 0)
                        {
                            var pc = Pc - 2;
                            _registers[PcIdx] = (byte)(pc >> 8);
                            _registers[PcIdx + 1] = (byte)pc;
#if (DEBUG)
                            _logger.Log("(INIR)");
#endif
                            Wait(21);
                        }
                        else
                        {
                            _registers[FIdx] = (byte)(_registers[FIdx] | (byte)(Fl.N | Fl.Z));
#if (DEBUG)
                            _logger.Log("INIR");
#endif
                            Wait(16);
                        }
                        return;
                    }
                case 0xAA:
                    {
                        var a = ports.ReadPort(Bc);
                        var hl = Hl;
                        mem[hl--] = a;
                        _registers[HIdx] = (byte)(hl >> 8);
                        _registers[LIdx] = (byte)hl;
                        var b = (byte)(_registers[BIdx] - 1);
                        _registers[BIdx] = b;
                        var f = (byte)(_registers[FIdx] & (byte)~(Fl.N | Fl.Z));
                        if (b == 0) f |= (byte)Fl.Z;
                        f |= (byte)Fl.N;
                        _registers[FIdx] = f;
#if (DEBUG)
                        _logger.Log("IND");
#endif
                        Wait(16);
                        return;
                    }
                case 0xBA:
                    {
                        var a = ports.ReadPort(Bc);
                        var hl = Hl;
                        mem[hl--] = a;
                        _registers[HIdx] = (byte)(hl >> 8);
                        _registers[LIdx] = (byte)hl;
                        var b = (byte)(_registers[BIdx] - 1);
                        _registers[BIdx] = b;
                        if (b != 0)
                        {
                            var pc = Pc - 2;
                            _registers[PcIdx] = (byte)(pc >> 8);
                            _registers[PcIdx + 1] = (byte)pc;
#if (DEBUG)
                            _logger.Log("(INDR)");
#endif
                            Wait(21);
                        }
                        else
                        {
                            _registers[FIdx] = (byte)(_registers[FIdx] | (byte)(Fl.N | Fl.Z));
#if (DEBUG)
                            _logger.Log("INDR");
#endif
                            Wait(16);
                        }
                        return;
                    }
                case 0x41:
                case 0x49:
                case 0x51:
                case 0x59:
                case 0x61:
                case 0x69:
                case 0x79:
                    {
                        var a = _registers[r];
                        ports.WritePort(Bc, a);
                        var f = (byte)(_registers[FIdx] & 0x29);
                        if ((a & 0x80) > 0) f |= (byte)Fl.S;
                        if (a == 0) f |= (byte)Fl.Z;
                        if (Parity(a)) f |= (byte)Fl.PV;
                        _registers[FIdx] = f;
#if (DEBUG)
                        _logger.Log($"OUT (BC), {_logger.RName(r)}");
#endif
                        Wait(8);
                        return;
                    }
                case 0xA3:
                    {
                        var hl = Hl;
                        var a = mem[hl++];
                        ports.WritePort(Bc, a);
                        _registers[HIdx] = (byte)(hl >> 8);
                        _registers[LIdx] = (byte)hl;
                        var b = (byte)(_registers[BIdx] - 1);
                        _registers[BIdx] = b;
                        var f = (byte)(_registers[FIdx] & (byte)~(Fl.N | Fl.Z));
                        if (b == 0) f |= (byte)Fl.Z;
                        f |= (byte)Fl.N;
                        _registers[FIdx] = f;

#if (DEBUG)
                        _logger.Log("OUTI");
#endif
                        Wait(16);
                        return;
                    }
                case 0xB3:
                    {
                        var hl = Hl;
                        var a = mem[hl++];
                        ports.WritePort(Bc, a);
                        _registers[HIdx] = (byte)(hl >> 8);
                        _registers[LIdx] = (byte)hl;
                        var b = (byte)(_registers[BIdx] - 1);
                        _registers[BIdx] = b;
                        if (b != 0)
                        {
                            var pc = Pc - 2;
                            _registers[PcIdx] = (byte)(pc >> 8);
                            _registers[PcIdx + 1] = (byte)pc;
#if (DEBUG)
                            _logger.Log("(OUTIR)");
#endif
                            Wait(21);
                        }
                        else
                        {
                            _registers[FIdx] = (byte)(_registers[FIdx] | (byte)(Fl.N | Fl.Z));
#if (DEBUG)
                            _logger.Log("OUTIR");
#endif
                            Wait(16);
                        }
                        return;
                    }
                case 0xAB:
                    {
                        var hl = Hl;
                        var a = mem[hl--];
                        ports.WritePort(Bc, a);
                        _registers[HIdx] = (byte)(hl >> 8);
                        _registers[LIdx] = (byte)hl;
                        var b = (byte)(_registers[BIdx] - 1);
                        _registers[BIdx] = b;
                        var f = (byte)(_registers[FIdx] & (byte)~(Fl.N | Fl.Z));
                        if (b == 0) f |= (byte)Fl.Z;
                        f |= (byte)Fl.N;
                        _registers[FIdx] = f;
#if (DEBUG)
                        _logger.Log("OUTD");
#endif
                        Wait(16);
                        return;
                    }
                case 0xBB:
                    {
                        var hl = Hl;
                        var a = mem[hl--];
                        ports.WritePort(Bc, a);
                        _registers[HIdx] = (byte)(hl >> 8);
                        _registers[LIdx] = (byte)hl;
                        var b = (byte)(_registers[BIdx] - 1);
                        _registers[BIdx] = b;
                        if (b != 0)
                        {
                            var pc = Pc - 2;
                            _registers[PcIdx] = (byte)(pc >> 8);
                            _registers[PcIdx + 1] = (byte)pc;
#if (DEBUG)
                            _logger.Log("(OUTDR)");
#endif
                            Wait(21);
                        }
                        else
                        {
                            _registers[FIdx] = (byte)(_registers[FIdx] | (byte)(Fl.N | Fl.Z));
#if (DEBUG)
                            _logger.Log("OUTDR");
#endif
                            Wait(16);
                        }
                        return;
                    }
            }
#if (DEBUG)
            _logger.Log($"ED {mc:X2}: {r:X2}");
#endif
            Halt = true;
        }

        private void ParseDD()
        {
            if (Halt) return;
            var mc = Fetch();
            var hi = (byte)(mc >> 6);
            var lo = (byte)(mc & 0x07);
            var mid = (byte)((mc >> 3) & 0x07);

            switch (mc)
            {
                case 0xCB:
                    {
                        ParseCB(0xDD);
                        return;
                    }
                case 0x21:
                    {
                        // LD IX, nn
                        _registers[IxIdx + 1] = Fetch();
                        _registers[IxIdx] = Fetch();
#if (DEBUG)
                        _logger.Log($"LD IX, 0x{Ix:X4}");
#endif
                        Wait(14);
                        return;
                    }
                case 0x46:
                case 0x4e:
                case 0x56:
                case 0x5e:
                case 0x66:
                case 0x6e:
                case 0x7e:
                    {
                        // LD r, (IX+d)
                        var d = (sbyte)Fetch();
                        _registers[mid] = mem[(ushort)(Ix + d)];
#if (DEBUG)
                        _logger.Log($"LD {_logger.RName(mid)}, (IX{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0x70:
                case 0x71:
                case 0x72:
                case 0x73:
                case 0x74:
                case 0x75:
                case 0x77:
                    {
                        // LD (IX+d), r
                        var d = (sbyte)Fetch();
                        mem[(ushort)(Ix + d)] = _registers[lo];
#if (DEBUG)
                        _logger.Log($"LD (IX{d:+0;-#}), {_logger.RName(lo)}");
#endif
                        Wait(19);
                        return;
                    }
                case 0x36:
                    {
                        // LD (IX+d), n
                        var d = (sbyte)Fetch();
                        var n = Fetch();
                        mem[(ushort)(Ix + d)] = n;
#if (DEBUG)
                        _logger.Log($"LD (IX{d:+0;-#}), {n}");
#endif
                        Wait(19);
                        return;
                    }
                case 0x2A:
                    {
                        // LD IX, (nn)
                        var addr = Fetch16();
                        _registers[IxIdx + 1] = mem[addr++];
                        _registers[IxIdx] = mem[addr];
#if (DEBUG)
                        _logger.Log($"LD IX, (0x{addr:X4})*");
#endif
                        Wait(20);
                        return;
                    }
                case 0x22:
                    {
                        // LD (nn), IX
                        var addr = Fetch16();
                        mem[addr++] = _registers[IxIdx + 1];
                        mem[addr] = _registers[IxIdx];
#if (DEBUG)
                        _logger.Log($"LD (0x{addr:X4}), IX");
#endif
                        Wait(20);
                        return;
                    }

                case 0xF9:
                    {
                        // LD SP, IX
                        _registers[SpIdx] = _registers[IxIdx];
                        _registers[SpIdx + 1] = _registers[IxIdx + 1];
#if (DEBUG)
                        _logger.Log("LD SP, IX");
#endif
                        Wait(10);
                        return;
                    }
                case 0xE5:
                    {
                        // PUSH IX
                        var addr = Sp;
                        addr--;
                        mem[addr] = _registers[IxIdx];
                        addr--;
                        mem[addr] = _registers[IxIdx + 1];
                        _registers[SpIdx + 1] = (byte)(addr & 0xFF);
                        _registers[SpIdx] = (byte)(addr >> 8);
#if (DEBUG)
                        _logger.Log("PUSH IX");
#endif
                        Wait(15);
                        return;
                    }
                case 0xE1:
                    {
                        // POP IX
                        var addr = Sp;
                        _registers[IxIdx + 1] = mem[addr++];
                        _registers[IxIdx] = mem[addr++];
                        _registers[SpIdx + 1] = (byte)(addr & 0xFF);
                        _registers[SpIdx] = (byte)(addr >> 8);
#if (DEBUG)
                        _logger.Log("POP IX");
#endif
                        Wait(14);
                        return;
                    }
                case 0xE3:
                    {
                        // EX (SP), IX
                        var h = _registers[IxIdx];
                        var l = _registers[IxIdx + 1];
                        var addr = Sp;
                        _registers[IxIdx + 1] = mem[addr++];
                        _registers[IxIdx] = mem[addr];
                        mem[addr--] = h;
                        mem[addr] = l;

#if (DEBUG)
                        _logger.Log("EX (SP), IX");
#endif
                        Wait(24);
                        return;
                    }

                case 0x86:
                    {
                        // ADD A, (IX+d)
                        var d = (sbyte)Fetch();

                        Add(mem[(ushort)(Ix + d)]);
#if (DEBUG)
                        _logger.Log($"ADD A, (IX{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0x8E:
                    {
                        // ADC A, (IX+d)
                        var d = (sbyte)Fetch();
                        var a = _registers[AIdx];
                        Adc(mem[(ushort)(Ix + d)]);
#if (DEBUG)
                        _logger.Log($"ADC A, (IX{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0x96:
                    {
                        // SUB A, (IX+d)
                        var d = (sbyte)Fetch();
                        var b = mem[(ushort)(Ix + d)];

                        Sub(b);
#if (DEBUG)
                        _logger.Log($"SUB A, (IX{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0x9E:
                    {
                        // SBC A, (IX+d)
                        var d = (sbyte)Fetch();

                        Sbc(mem[(ushort)(Ix + d)]);
#if (DEBUG)
                        _logger.Log($"SBC A, (IX{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0xA6:
                    {
                        // AND A, (IX+d)
                        var d = (sbyte)Fetch();
                        var b = mem[(ushort)(Ix + d)];

                        And(b);
#if (DEBUG)
                        _logger.Log($"AND A, (IX{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0xB6:
                    {
                        // OR A, (IX+d)
                        var d = (sbyte)Fetch();
                        var b = mem[(ushort)(Ix + d)];

                        Or(b);
#if (DEBUG)
                        _logger.Log($"OR A, (IX{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0xAE:
                    {
                        // OR A, (IX+d)
                        var d = (sbyte)Fetch();
                        var b = mem[(ushort)(Ix + d)];

                        Xor(b);
#if (DEBUG)
                        _logger.Log($"XOR A, (IX{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0xBE:
                    {
                        // CP A, (IX+d)
                        var d = (sbyte)Fetch();
                        var b = mem[(ushort)(Ix + d)];

                        Cmp(b);
#if (DEBUG)
                        _logger.Log($"CP A, (IX{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0x34:
                    {
                        // INC (IX+d)
                        var d = (sbyte)Fetch();
                        mem[(ushort)(Ix + d)] = Inc(mem[(ushort)(Ix + d)]);
#if (DEBUG)
                        _logger.Log($"INC (IX{d:+0;-#})");
#endif
                        Wait(7);
                        return;
                    }
                case 0x35:
                    {
                        // DEC (IX+d)
                        var d = (sbyte)Fetch();
                        mem[(ushort)(Ix + d)] = Dec(mem[(ushort)(Ix + d)]);
#if (DEBUG)
                        _logger.Log($"DEC (IX{d:+0;-#})");
#endif
                        Wait(7);
                        return;
                    }
                case 0x09:
                    {
                        AddIx(Bc);
#if (DEBUG)
                        _logger.Log("ADD IX, BC");
#endif
                        Wait(4);
                        return;
                    }
                case 0x19:
                    {
                        AddIx(De);
#if (DEBUG)
                        _logger.Log("ADD IX, DE");
#endif
                        Wait(4);
                        return;
                    }
                case 0x29:
                    {
                        AddIx(Ix);
#if (DEBUG)
                        _logger.Log("ADD IX, IX");
#endif
                        Wait(4);
                        return;
                    }
                case 0x39:
                    {
                        AddIx(Sp);
#if (DEBUG)
                        _logger.Log("ADD IX, SP");
#endif
                        Wait(4);
                        return;
                    }
                case 0x23:
                    {
                        var val = Ix + 1;
                        _registers[IxIdx] = (byte)(val >> 8);
                        _registers[IxIdx + 1] = (byte)(val & 0xFF);
#if (DEBUG)
                        _logger.Log("INC IX");
#endif
                        Wait(4);
                        return;
                    }
                case 0x2B:
                    {
                        var val = Ix - 1;
                        _registers[IxIdx] = (byte)(val >> 8);
                        _registers[IxIdx + 1] = (byte)(val & 0xFF);
#if (DEBUG)
                        _logger.Log("DEC IX");
#endif
                        Wait(4);
                        return;
                    }
                case 0xE9:
                    {
                        var addr = Ix;
                        _registers[PcIdx] = (byte)(addr >> 8);
                        _registers[PcIdx + 1] = (byte)(addr);
#if (DEBUG)
                        _logger.Log("JP IX");
#endif
                        Wait(8);
                        return;
                    }

            }
#if (DEBUG)
            _logger.Log($"DD {mc:X2}: {hi:X} {mid:X} {lo:X}");
#endif
            Halt = true;
        }

        private void ParseFD()
        {
            if (Halt) return;
            var mc = Fetch();
            var hi = (byte)(mc >> 6);
            var lo = (byte)(mc & 0x07);
            var r = (byte)((mc >> 3) & 0x07);

            switch (mc)
            {
                case 0xCB:
                    {
                        ParseCB(0xFD);
                        return;
                    }
                case 0x21:
                    {
                        // LD IY, nn
                        _registers[IyIdx + 1] = Fetch();
                        _registers[IyIdx] = Fetch();
#if (DEBUG)
                        _logger.Log($"LD IY, 0x{Iy:X4}");
#endif
                        Wait(14);
                        return;
                    }

                case 0x46:
                case 0x4e:
                case 0x56:
                case 0x5e:
                case 0x66:
                case 0x6e:
                case 0x7e:
                    {
                        // LD r, (IY+d)
                        var d = (sbyte)Fetch();
                        _registers[r] = mem[(ushort)(Iy + d)];
#if (DEBUG)
                        _logger.Log($"LD {_logger.RName(r)}, (IY{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0x70:
                case 0x71:
                case 0x72:
                case 0x73:
                case 0x74:
                case 0x75:
                case 0x77:
                    {
                        // LD (IY+d), r
                        var d = (sbyte)Fetch();
                        mem[(ushort)(Iy + d)] = _registers[lo];
#if (DEBUG)
                        _logger.Log($"LD (IY{d:+0;-#}), {_logger.RName(lo)}");
#endif
                        Wait(19);
                        return;
                    }
                case 0x36:
                    {
                        // LD (IY+d), n
                        var d = (sbyte)Fetch();
                        var n = Fetch();
                        mem[(ushort)(Iy + d)] = n;
#if (DEBUG)
                        _logger.Log($"LD (IY{d:+0;-#}), {n}");
#endif
                        Wait(19);
                        return;
                    }
                case 0x2A:
                    {
                        // LD IY, (nn)
                        var addr = Fetch16();
                        _registers[IyIdx + 1] = mem[addr++];
                        _registers[IyIdx] = mem[addr];
#if (DEBUG)
                        _logger.Log($"LD IY, (0x{--addr:X4})*");
#endif
                        Wait(20);
                        return;
                    }

                case 0x22:
                    {
                        // LD (nn), IY
                        var addr = Fetch16();
                        mem[addr++] = _registers[IyIdx + 1];
                        mem[addr] = _registers[IyIdx];
#if (DEBUG)
                        _logger.Log($"LD (0x{--addr:X4}), IY");
#endif
                        Wait(20);
                        return;
                    }
                case 0xF9:
                    {
                        // LD SP, IY
                        _registers[SpIdx] = _registers[IyIdx];
                        _registers[SpIdx + 1] = _registers[IyIdx + 1];
#if (DEBUG)
                        _logger.Log("LD SP, IY");
#endif
                        Wait(10);
                        return;
                    }
                case 0xE5:
                    {
                        // PUSH IY
                        var addr = Sp;
                        mem[--addr] = _registers[IyIdx];
                        mem[--addr] = _registers[IyIdx + 1];
                        _registers[SpIdx + 1] = (byte)(addr & 0xFF);
                        _registers[SpIdx] = (byte)(addr >> 8);
#if (DEBUG)
                        _logger.Log("PUSH IY");
#endif
                        Wait(15);
                        return;
                    }
                case 0xE1:
                    {
                        // POP IY
                        var addr = Sp;
                        _registers[IyIdx + 1] = mem[addr++];
                        _registers[IyIdx] = mem[addr++];
                        _registers[SpIdx + 1] = (byte)(addr & 0xFF);
                        _registers[SpIdx] = (byte)(addr >> 8);
#if (DEBUG)
                        _logger.Log("POP IY");
#endif
                        Wait(14);
                        return;
                    }
                case 0xE3:
                    {
                        // EX (SP), IY
                        var h = _registers[IyIdx];
                        var l = _registers[IyIdx + 1];
                        var addr = Sp;
                        _registers[IyIdx + 1] = mem[addr];
                        mem[addr++] = l;
                        _registers[IyIdx] = mem[addr];
                        mem[addr] = h;

#if (DEBUG)
                        _logger.Log("EX (SP), IY");
#endif
                        Wait(24);
                        return;
                    }
                case 0x86:
                    {
                        // ADD A, (IY+d)
                        var d = (sbyte)Fetch();

                        Add(mem[(ushort)(Iy + d)]);
#if (DEBUG)
                        _logger.Log($"ADD A, (IY{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0x8E:
                    {
                        // ADC A, (IY+d)
                        var d = (sbyte)Fetch();
                        var a = _registers[AIdx];
                        Adc(mem[(ushort)(Iy + d)]);
#if (DEBUG)
                        _logger.Log($"ADC A, (IY{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0x96:
                    {
                        // SUB A, (IY+d)
                        var d = (sbyte)Fetch();

                        Sub(mem[(ushort)(Iy + d)]);
#if (DEBUG)
                        _logger.Log($"SUB A, (IY{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0x9E:
                    {
                        // SBC A, (IY+d)
                        var d = (sbyte)Fetch();

                        Sbc(mem[(ushort)(Iy + d)]);
#if (DEBUG)
                        _logger.Log($"SBC A, (IY{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0xA6:
                    {
                        // AND A, (IY+d)
                        var d = (sbyte)Fetch();
                        var b = mem[(ushort)(Iy + d)];

                        And(b);
#if (DEBUG)
                        _logger.Log($"AND A, (IY{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0xB6:
                    {
                        // OR A, (IY+d)
                        var d = (sbyte)Fetch();
                        var b = mem[(ushort)(Iy + d)];

                        Or(b);
#if (DEBUG)
                        _logger.Log($"OR A, (IY{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0xAE:
                    {
                        // XOR A, (IY+d)
                        var d = (sbyte)Fetch();
                        var b = mem[(ushort)(Iy + d)];

                        Xor(b);
#if (DEBUG)
                        _logger.Log($"XOR A, (IY{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0xBE:
                    {
                        // CP A, (IY+d)
                        var d = (sbyte)Fetch();

                        Cmp(mem[(ushort)(Iy + d)]);
#if (DEBUG)
                        _logger.Log($"CP A, (IY{d:+0;-#})");
#endif
                        Wait(19);
                        return;
                    }
                case 0x34:
                    {
                        // INC (IY+d)
                        var d = (sbyte)Fetch();
                        mem[(ushort)(Iy + d)] = Inc(mem[(ushort)(Iy + d)]);
#if (DEBUG)
                        _logger.Log($"INC (IY{d:+0;-#})");
#endif
                        Wait(7);
                        return;
                    }
                case 0x35:
                    {
                        // DEC (IY+d)
                        var d = (sbyte)Fetch();
                        mem[(ushort)(Iy + d)] = Dec(mem[(ushort)(Iy + d)]);
#if (DEBUG)
                        _logger.Log($"DEC (IY{d:+0;-#})");
#endif
                        Wait(7);
                        return;
                    }
                case 0x09:
                    {
                        AddIy(Bc);
#if (DEBUG)
                        _logger.Log("ADD IY, BC");
#endif
                        Wait(4);
                        return;
                    }
                case 0x19:
                    {
                        AddIy(De);
#if (DEBUG)
                        _logger.Log("ADD IY, DE");
#endif
                        Wait(4);
                        return;
                    }
                case 0x29:
                    {
                        AddIy(Iy);
#if (DEBUG)
                        _logger.Log("ADD IY, IY");
#endif
                        Wait(4);
                        return;
                    }
                case 0x39:
                    {
                        AddIy(Sp);
#if (DEBUG)
                        _logger.Log("ADD IY, SP");
#endif
                        Wait(4);
                        return;
                    }
                case 0x23:
                    {
                        var val = Iy + 1;
                        _registers[IyIdx] = (byte)(val >> 8);
                        _registers[IyIdx + 1] = (byte)(val & 0xFF);
#if (DEBUG)
                        _logger.Log("INC IY");
#endif
                        Wait(4);
                        return;
                    }
                case 0x2B:
                    {
                        var val = Iy - 1;
                        _registers[IyIdx] = (byte)(val >> 8);
                        _registers[IyIdx + 1] = (byte)(val & 0xFF);
#if (DEBUG)
                        _logger.Log("DEC IY");
#endif
                        Wait(4);
                        return;
                    }
                case 0xE9:
                    {
                        var addr = Iy;
                        _registers[PcIdx] = (byte)(addr >> 8);
                        _registers[PcIdx + 1] = (byte)(addr);
#if (DEBUG)
                        _logger.Log("JP IY");
#endif
                        Wait(8);
                        return;
                    }
            }
#if (DEBUG)
            _logger.Log($"FD {mc:X2}: {hi:X2} {lo:X2} {r:X2}");
#endif
            Halt = true;
        }

        private void Add(byte b)
        {
            var a = _registers[AIdx];
            var sum = a + b;
            _registers[AIdx] = (byte)sum;
            var f = (byte)(_registers[FIdx] & 0x28);
            if ((sum & 0x80) > 0)
                f |= (byte)Fl.S;
            if ((byte)sum == 0)
                f |= (byte)Fl.Z;
            if ((a & 0xF + b & 0xF) > 0xF)
                f |= (byte)Fl.H;
            if ((a >= 0x80 && b >= 0x80 && (sbyte)sum > 0) || (a < 0x80 && b < 0x80 && (sbyte)sum < 0))
                f |= (byte)Fl.PV;
            if (sum > 0xFF)
                f |= (byte)Fl.C;
            _registers[FIdx] = f;
        }

        private void Adc(byte b)
        {
            var a = _registers[AIdx];
            var c = (byte)(_registers[FIdx] & (byte)Fl.C);
            var sum = a + b + c;
            _registers[AIdx] = (byte)sum;
            var f = (byte)(_registers[FIdx] & 0x28);
            if ((sum & 0x80) > 0)
                f |= (byte)Fl.S;
            if ((byte)sum == 0)
                f |= (byte)Fl.Z;
            if ((a & 0xF + b & 0xF) > 0xF)
                f |= (byte)Fl.H;
            if ((a >= 0x80 && b >= 0x80 && (sbyte)sum > 0) || (a < 0x80 && b < 0x80 && (sbyte)sum < 0))
                f |= (byte)Fl.PV;
            f = (byte)(f & ~(byte)Fl.N);
            if (sum > 0xFF) f |= (byte)Fl.C;
            _registers[FIdx] = f;
        }

        private void Sub(byte b)
        {
            var a = _registers[AIdx];
            var diff = a - b;
            _registers[AIdx] = (byte)diff;
            var f = (byte)(_registers[FIdx] & 0x28);
            if ((diff & 0x80) > 0)
                f |= (byte)Fl.S;
            if (diff == 0)
                f |= (byte)Fl.Z;
            if ((a & 0xF) < (b & 0xF))
                f |= (byte)Fl.H;
            if ((a >= 0x80 && b >= 0x80 && (sbyte)diff > 0) || (a < 0x80 && b < 0x80 && (sbyte)diff < 0))
                f |= (byte)Fl.PV;
            f |= (byte)Fl.N;
            if (diff < 0)
                f |= (byte)Fl.C;
            _registers[FIdx] = f;
        }

        private void Sbc(byte b)
        {
            var a = _registers[AIdx];
            var c = (byte)(_registers[FIdx] & 0x01);
            var diff = a - b - c;
            _registers[AIdx] = (byte)diff;
            var f = (byte)(_registers[FIdx] & 0x28);
            if ((diff & 0x80) > 0) f |= (byte)Fl.S;
            if (diff == 0) f |= (byte)Fl.Z;
            if ((a & 0xF) < (b & 0xF) + c) f |= (byte)Fl.H;
            if ((a >= 0x80 && b >= 0x80 && (sbyte)diff > 0) || (a < 0x80 && b < 0x80 && (sbyte)diff < 0))
                f |= (byte)Fl.PV;
            f |= (byte)Fl.N;
            if (diff > 0xFF) f |= (byte)Fl.C;
            _registers[FIdx] = f;
        }

        private void And(byte b)
        {
            var a = _registers[AIdx];
            var res = (byte)(a & b);
            _registers[AIdx] = res;
            var f = (byte)(_registers[FIdx] & 0x28);
            if ((res & 0x80) > 0) f |= (byte)Fl.S;
            if (res == 0) f |= (byte)Fl.Z;
            f |= (byte)Fl.H;
            if (Parity(res)) f |= (byte)Fl.PV;
            _registers[FIdx] = f;
        }

        private void Or(byte b)
        {
            var a = _registers[AIdx];
            var res = (byte)(a | b);
            _registers[AIdx] = res;
            var f = (byte)(_registers[FIdx] & 0x28);
            if ((res & 0x80) > 0)
                f |= (byte)Fl.S;
            if (res == 0)
                f |= (byte)Fl.Z;
            if (Parity(res))
                f |= (byte)Fl.PV;
            _registers[FIdx] = f;
        }

        private void Xor(byte b)
        {
            var a = _registers[AIdx];
            var res = (byte)(a ^ b);
            _registers[AIdx] = res;
            var f = (byte)(_registers[FIdx] & 0x28);
            if ((res & 0x80) > 0)
                f |= (byte)Fl.S;
            if (res == 0)
                f |= (byte)Fl.Z;
            if (Parity(res))
                f |= (byte)Fl.PV;
            _registers[FIdx] = f;
        }

        private void Cmp(byte b)
        {
            var a = _registers[AIdx];
            var diff = a - b;
            var f = (byte)(_registers[FIdx] & 0x28);
            if ((diff & 0x80) > 0)
                f = (byte)(f | 0x80);
            if (diff == 0)
                f = (byte)(f | 0x40);
            if ((a & 0xF) < (b & 0xF))
                f = (byte)(f | 0x10);
            if ((a > 0x80 && b > 0x80 && (sbyte)diff > 0) || (a < 0x80 && b < 0x80 && (sbyte)diff < 0))
                f = (byte)(f | 0x04);
            f = (byte)(f | 0x02);
            if (diff > 0xFF)
                f = (byte)(f | 0x01);
            _registers[FIdx] = f;
        }

        private byte Inc(byte b)
        {
            var sum = b + 1;
            var f = (byte)(_registers[FIdx] & 0x28);
            if ((sum & 0x80) > 0)
                f = (byte)(f | 0x80);
            if (sum == 0)
                f = (byte)(f | 0x40);
            if ((b & 0xF) == 0xF)
                f = (byte)(f | 0x10);
            if ((b < 0x80 && (sbyte)sum < 0))
                f = (byte)(f | 0x04);
            f = (byte)(f | 0x02);
            if (sum > 0xFF) f = (byte)(f | 0x01);
            _registers[FIdx] = f;

            return (byte)sum;
        }

        private byte Dec(byte b)
        {
            var sum = b - 1;
            var f = (byte)(_registers[FIdx] & 0x28);
            if ((sum & 0x80) > 0)
                f = (byte)(f | 0x80);
            if (sum == 0)
                f = (byte)(f | 0x40);
            if ((b & 0x0F) == 0)
                f = (byte)(f | 0x10);
            if (b == 0x80)
                f = (byte)(f | 0x04);
            f = (byte)(f | 0x02);
            _registers[FIdx] = f;

            return (byte)sum;
        }

        private static bool Parity(ushort value)
        {
            var parity = true;
            while (value > 0)
            {
                if ((value & 1) == 1) parity = !parity;
                value = (byte)(value >> 1);
            }
            return parity;
        }

        private bool JumpCondition(byte condition)
        {
            Fl mask;
            switch (condition & 0xFE)
            {
                case 0:
                    mask = Fl.Z;
                    break;
                case 2:
                    mask = Fl.C;
                    break;
                case 4:
                    mask = Fl.PV;
                    break;
                case 6:
                    mask = Fl.S;
                    break;
                default:
                    return false;
            }
            return ((_registers[FIdx] & (byte)mask) > 0) == ((condition & 1) == 1);

        }

        /// <summary>
        ///     Fetches from [PC] and increments PC
        /// </summary>
        /// <returns></returns>
        private byte Fetch()
        {
            var pc = Pc;
            var ret = mem[pc];
#if (DEBUG)
            _logger.LogMemRead(pc, ret);
#endif
            pc++;
            _registers[PcIdx] = (byte)(pc >> 8);
            _registers[PcIdx + 1] = (byte)(pc & 0xFF);
            return ret;
        }

        private ushort Fetch16()
        {
            return (ushort)(Fetch() + (Fetch() << 8));
        }

        public void Reset()
        {
            Array.Clear(_registers, 0, _registers.Length);

            _registers[AIdx] = 0xFF;
            _registers[FIdx] = 0xFF;
            _registers[SpIdx] = 0xFF;
            _registers[SpIdx + 1] = 0xFF;

            //A CPU reset forces both the IFF1 and IFF2 to the reset state, which disables interrupts
            IFF1 = false;
            IFF2 = false;

            _clock = DateTime.UtcNow;
        }

        public byte[] GetState()
        {
            var length = _registers.Length;
            var ret = new byte[length + 2];
            Array.Copy(_registers, ret, length);
            ret[length] = (byte)(IFF1 ? 1 : 0);
            ret[length + 1] = (byte)(IFF2 ? 1 : 0);
            return ret;
        }

        public string DumpState()
        {
            var ret = " BC   DE   HL  SZ-H-PNC A" + Environment.NewLine;
            ret +=
                $"{_registers[BIdx]:X2}{_registers[CIdx]:X2} {_registers[DIdx]:X2}{_registers[EIdx]:X2} {_registers[HIdx]:X2}{_registers[LIdx]:X2} {(_registers[FIdx] & 0x80) >> 7}{(_registers[FIdx] & 0x40) >> 6}{(_registers[FIdx] & 0x20) >> 5}{(_registers[FIdx] & 0x10) >> 4}{(_registers[FIdx] & 0x08) >> 3}{(_registers[FIdx] & 0x04) >> 2}{(_registers[FIdx] & 0x02) >> 1}{(_registers[FIdx] & 0x01)} {_registers[AIdx]:X2}";
            ret +=
                $"\n{_registers[BpIdx]:X2}{_registers[CpIdx]:X2} {_registers[DpIdx]:X2}{_registers[EpIdx]:X2} {_registers[HpIdx]:X2}{_registers[LpIdx]:X2} {(_registers[FpIdx] & 0x80) >> 7}{(_registers[FpIdx] & 0x40) >> 6}{(_registers[FpIdx] & 0x20) >> 5}{(_registers[FpIdx] & 0x10) >> 4}{(_registers[FpIdx] & 0x08) >> 3}{(_registers[FpIdx] & 0x04) >> 2}{(_registers[FpIdx] & 0x02) >> 1}{_registers[FpIdx] & 0x01} {_registers[ApIdx]:X2}";
            ret += Environment.NewLine + Environment.NewLine + "I  R   IX   IY   SP   PC" + Environment.NewLine;
            ret +=
                $"{_registers[IIdx]:X2} {_registers[RIdx]:X2} {_registers[IxIdx]:X2}{_registers[IxIdx + 1]:X2} {_registers[IyIdx]:X2}{_registers[IyIdx + 1]:X2} {_registers[SpIdx]:X2}{_registers[SpIdx + 1]:X2} {_registers[PcIdx]:X2}{_registers[PcIdx + 1]:X2} ";

            ret += Environment.NewLine;
            return ret;
        }

        private void Wait(int t)
        {
            _registers[RIdx] += (byte)((t + 3) / 4);
            const int realTicksPerTick = 250; // 4MHz
            var ticks = t * realTicksPerTick;
            var elapsed = (DateTime.UtcNow - _clock).Ticks;
            var sleep = ticks - elapsed;
            if (sleep > 0)
            {
                Thread.Sleep((int)sleep / 1000000);
                _clock = _clock + new TimeSpan(ticks);
            }
            else
            {
                // #if (DEBUG)
                //                 _logger.Log($"Clock expected {((double)ticks) / realTicksPerTick:0.00} but was {((double)elapsed) / realTicksPerTick:0.00}");
                // #endif
                _clock = DateTime.UtcNow;
            }
        }

        private void SwapReg8(byte r1, byte r2)
        {
            var t = _registers[r1];
            _registers[r1] = _registers[r2];
            _registers[r2] = t;
        }

        [Flags]
        private enum Fl : byte
        {
            C = 0x01,
            N = 0x02,
            PV = 0x04,
            H = 0x10,
            Z = 0x40,
            S = 0x80
        }

    }
}

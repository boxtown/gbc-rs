#![allow(dead_code)]

struct GPU {
    vram: [u8; 0x2000],
    oam: [u8; 0x100],
}

impl std::default::Default for GPU {
    fn default() -> Self {
        GPU {
            vram: [0; 0x2000],
            oam: [0; 0x100],
        }
    }
}

struct MMU {
    rom: [u8; 0x8000],
    sram: [u8; 0x2000],
    wram: [u8; 0x2000],
    io: [u8; 0x100],
    hram: [u8; 0x80],
}

impl std::default::Default for MMU {
    fn default() -> Self {
        MMU {
            rom: [0; 0x8000],
            sram: [0; 0x2000],
            wram: [0; 0x2000],
            io: [0; 0x100],
            hram: [0; 0x80],
        }
    }
}

trait Read<T> {
    fn read(self, emu: &Emulator) -> T;
}

trait Write<T> {
    fn write(self, emu: &mut Emulator, value: T);
}

#[derive(Debug, Copy, Clone)]
struct Registers {
    pc: u16,
    sp: u16,
    a: u8,
    f: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    h: u8,
    l: u8,
}

#[derive(Debug, Copy, Clone)]
enum Flag {
    Zero,
    Sub,
    HalfCarry,
    Carry,
}

impl Flag {
    fn mask(&self) -> u8 {
        match self {
            Flag::Zero => 0x80,
            Flag::Sub => 0x40,
            Flag::HalfCarry => 0x20,
            Flag::Carry => 0x10,
        }
    }
}

#[derive(Debug, Copy, Clone)]
enum Register {
    A,
    B,
    C,
    D,
    E,
    H,
    L,
}

impl Read<u8> for Register {
    fn read(self, emu: &Emulator) -> u8 {
        match self {
            Register::A => emu.registers.a,
            Register::B => emu.registers.b,
            Register::C => emu.registers.c,
            Register::D => emu.registers.d,
            Register::E => emu.registers.e,
            Register::H => emu.registers.h,
            Register::L => emu.registers.l,
        }
    }
}

impl Write<u8> for Register {
    fn write(self, emu: &mut Emulator, value: u8) {
        match self {
            Register::A => emu.registers.a = value,
            Register::B => emu.registers.b = value,
            Register::C => emu.registers.c = value,
            Register::D => emu.registers.d = value,
            Register::E => emu.registers.e = value,
            Register::H => emu.registers.h = value,
            Register::L => emu.registers.l = value,
        }
    }
}

#[derive(Debug, Copy, Clone)]
enum Reg16 {
    AF,
    BC,
    DE,
    HL,
    SP,
}

impl std::convert::From<Addr> for Reg16 {
    fn from(addr: Addr) -> Self {
        match addr {
            Addr::BC => Reg16::BC,
            Addr::DE => Reg16::DE,
            Addr::HL => Reg16::HL,
            Addr::SP => Reg16::SP,
        }
    }
}

impl Read<u16> for Reg16 {
    fn read(self, emu: &Emulator) -> u16 {
        match self {
            Reg16::AF => ((emu.registers.a as u16) << 8) | (emu.registers.f as u16),
            Reg16::BC => ((emu.registers.b as u16) << 8) | (emu.registers.c as u16),
            Reg16::DE => ((emu.registers.d as u16) << 8) | (emu.registers.e as u16),
            Reg16::HL => ((emu.registers.h as u16) << 8) | (emu.registers.l as u16),
            Reg16::SP => emu.registers.sp,
        }
    }
}

impl Write<u16> for Reg16 {
    fn write(self, emu: &mut Emulator, value: u16) {
        match self {
            Reg16::AF => {
                emu.registers.a = (value >> 8) as u8;
                emu.registers.f = value as u8;
            }
            Reg16::BC => {
                emu.registers.b = (value >> 8) as u8;
                emu.registers.c = value as u8;
            }
            Reg16::DE => {
                emu.registers.d = (value >> 8) as u8;
                emu.registers.e = value as u8;
            }
            Reg16::HL => {
                emu.registers.h = (value >> 8) as u8;
                emu.registers.l = value as u8;
            }
            Reg16::SP => emu.registers.sp = value,
        }
    }
}

enum Addr {
    BC,
    DE,
    HL,
    SP,
}

impl Read<u8> for Addr {
    fn read(self, emu: &Emulator) -> u8 {
        let reg = Reg16::from(self);
        let addr = reg.read(emu);
        emu.read_byte(addr)
    }
}

impl Write<u8> for Addr {
    fn write(self, emu: &mut Emulator, value: u8) {
        let reg = Reg16::from(self);
        let addr = reg.read(emu);
        emu.write_byte(addr, value);
    }
}

impl Registers {
    fn update_flag(&mut self, flag: Flag, flag_set: bool) {
        if flag_set {
            self.f |= flag.mask();
        } else {
            self.f &= !flag.mask();
        }
    }

    fn flag(&self, flag: Flag) -> bool {
        let mask = flag.mask();
        self.f & mask == mask
    }
}

impl std::default::Default for Registers {
    fn default() -> Registers {
        Registers {
            pc: 0,
            sp: 0,
            a: 0,
            f: 0,
            b: 0,
            c: 0,
            d: 0,
            e: 0,
            h: 0,
            l: 0,
        }
    }
}

struct Interrupts {}

impl std::default::Default for Interrupts {
    fn default() -> Interrupts {
        Interrupts {}
    }
}

enum JumpCond {
    NZ,
    Z,
    NC,
    C,
}

#[derive(Default)]
struct Emulator {
    gpu: GPU,
    mmu: MMU,
    registers: Registers,
    interrupts: Interrupts,
}

impl Emulator {
    fn read_byte(&self, addr: u16) -> u8 {
        match addr {
            0x0000..=0x7fff => self.mmu.rom[addr as usize],
            0x8000..=0x9fff => self.gpu.vram[(addr - 0x8000) as usize],
            0xa000..=0xbfff => self.mmu.sram[(addr - 0xa000) as usize],
            0xc000..=0xdfff => self.mmu.wram[(addr - 0xc000) as usize],
            0xe000..=0xfdff => self.mmu.wram[(addr - 0xe000) as usize],
            0xfe00..=0xfe9f => self.gpu.oam[(addr - 0xfe00) as usize],
            0xfea0..=0xfeff => 0, // GPU OAM is only 160 bytes
            0xff00..=0xff7f => self.mmu.io[(addr - 0xff00) as usize],
            0xff80..=0xfffe => self.mmu.hram[(addr - 0xff80) as usize],
            0xffff => 0,
        }
    }

    fn read_word(&self, addr: u16) -> u16 {
        let lo = self.read_byte(addr);
        let hi = self.read_byte(addr.wrapping_add(1));
        u16::from_le_bytes([lo, hi])
    }

    fn write_byte(&mut self, addr: u16, value: u8) {
        match addr {
            0x0000..=0x7fff => self.mmu.rom[addr as usize] = value,
            0x8000..=0x9fff => self.gpu.vram[(addr - 0x8000) as usize] = value,
            0xa000..=0xbfff => self.mmu.sram[(addr - 0xa000) as usize] = value,
            0xc000..=0xdfff => self.mmu.wram[(addr - 0xc000) as usize] = value,
            0xe000..=0xfdff => self.mmu.wram[(addr - 0xe000) as usize] = value,
            0xfe00..=0xfe9f => self.gpu.oam[(addr - 0xfe00) as usize] = value,
            0xfea0..=0xfeff => (), // GPU OAM is only 160 bytes
            0xff00..=0xff7f => self.mmu.io[(addr - 0xff00) as usize] = value,
            0xff80..=0xfffe => self.mmu.hram[(addr - 0xff80) as usize] = value,
            0xffff => (),
        }
    }

    fn write_word(&mut self, addr: u16, value: u16) {
        let bytes = value.to_le_bytes();
        self.write_byte(addr, bytes[0]);
        self.write_byte(addr.wrapping_add(1), bytes[1]);
    }

    fn fetch_next_byte(&mut self) -> u8 {
        let byte = self.read_byte(self.registers.pc);
        self.registers.pc = self.registers.pc.wrapping_add(1);
        byte
    }

    fn fetch_next_word(&mut self) -> u16 {
        let word = self.read_word(self.registers.pc);
        self.registers.pc = self.registers.pc.wrapping_add(2);
        word
    }

    fn cpu_step(&mut self) {
        let byte = self.fetch_next_byte();

        match byte {
            0x00 => (), // nop
            0x01 => self.load_immediate16(Reg16::BC),
            0x02 => self.load(Addr::BC, Register::A),
            0x03 => self.inc16(Reg16::BC),
            0x04 => self.inc(Register::B),
            0x05 => self.dec(Register::B),
            0x06 => self.load_immediate(Register::B),
            0x07 => self.rlc(Register::A),
            0x08 => self.ld_nn_sp(),
            0x09 => self.add16(Reg16::HL, Reg16::BC),
            0x0a => self.load(Register::A, Addr::BC),
            0x0b => self.dec16(Reg16::BC),
            0x0c => self.inc(Register::C),
            0x0d => self.dec(Register::C),
            0x0e => self.load_immediate(Register::C),
            0x0f => self.rrc(Register::A),
            0x10 => self.stop(),
            0x11 => self.load_immediate16(Reg16::DE),
            0x12 => self.load(Addr::DE, Register::A),
            0x13 => self.inc16(Reg16::DE),
            0x14 => self.inc(Register::D),
            0x15 => self.dec(Register::D),
            0x16 => self.load_immediate(Register::D),
            0x17 => self.rl(Register::A),
            0x18 => self.jr(),
            0x19 => self.add16(Reg16::HL, Reg16::DE),
            0x1a => self.load(Register::A, Addr::DE),
            0x1b => self.dec16(Reg16::DE),
            0x1c => self.inc(Register::E),
            0x1d => self.dec(Register::E),
            0x1e => self.load_immediate(Register::E),
            0x1f => self.rr(Register::A),
            0x20 => self.jr_cond(JumpCond::NZ),
            0x21 => self.load_immediate16(Reg16::HL),
            0x22 => self.ldi_hl_a(),
            0x23 => self.inc16(Reg16::HL),
            0x24 => self.inc(Register::H),
            0x25 => self.dec(Register::H),
            0x26 => self.load_immediate(Register::H),
            0x2f => self.cpl(),
            0x31 => self.load_immediate16(Reg16::SP),
            0xaf => self.xor(Register::A, Register::A),
            _ => unimplemented!(),
        }
    }

    // special case instructions

    fn ld_nn_sp(&mut self) {
        let nn = self.fetch_next_word();
        self.write_word(nn, self.registers.sp);
    }

    fn stop(&self) {
        panic!("STOP!");
    }

    fn ldi_hl_a(&mut self) {
        self.load(Addr::HL, Register::A);
        self.inc16(Reg16::HL);
    }

    // generalized instructions

    fn load_immediate(&mut self, reg: Register) {
        let n = self.fetch_next_byte();
        reg.write(self, n);
    }

    fn load_immediate16(&mut self, reg: Reg16) {
        let nn = self.fetch_next_word();
        reg.write(self, nn);
    }

    fn load<R, W>(&mut self, into: W, from: R)
    where
        R: Read<u8>,
        W: Write<u8>,
    {
        into.write(self, from.read(self));
    }

    fn jr(&mut self) {
        let n = self.fetch_next_byte();
        self.registers.pc = self.registers.pc.wrapping_add(n as u16);
    }

    fn jr_cond(&mut self, cond: JumpCond) {
        match cond {
            JumpCond::NZ => {
                if !self.registers.flag(Flag::Zero) {
                    self.jr()
                }
            }
            JumpCond::Z => {
                if self.registers.flag(Flag::Zero) {
                    self.jr()
                }
            }
            JumpCond::NC => {
                if !self.registers.flag(Flag::Carry) {
                    self.jr()
                }
            }
            JumpCond::C => {
                if self.registers.flag(Flag::Carry) {
                    self.jr()
                }
            }
        }
    }

    fn add16(&mut self, target: Reg16, source: Reg16) {
        let source_value = source.read(self);
        let target_value = target.read(self);
        let new_target_value = target_value.wrapping_add(source_value);

        self.registers.update_flag(Flag::Sub, false);
        self.registers.update_flag(
            Flag::HalfCarry,
            (target_value & 0x07ff) + (source_value & 0x07ff) > 0x07ff,
        );
        self.registers
            .update_flag(Flag::Carry, target_value > 0xffff - source_value);

        target.write(self, new_target_value);
    }

    fn inc(&mut self, reg: Register) {
        let value = reg.read(self);
        let new_value = value.wrapping_add(1);

        self.registers.update_flag(Flag::Zero, new_value == 0);
        self.registers.update_flag(Flag::Sub, false);
        self.registers
            .update_flag(Flag::HalfCarry, value & 0x0f == 0x0f);

        reg.write(self, new_value);
    }

    fn inc16(&mut self, reg: Reg16) {
        reg.write(self, reg.read(self).wrapping_add(1));
    }

    fn dec(&mut self, reg: Register) {
        let value = reg.read(self);
        let new_value = value.wrapping_sub(1);

        self.registers.update_flag(Flag::Zero, new_value == 0);
        self.registers.update_flag(Flag::Sub, true);
        self.registers
            .update_flag(Flag::HalfCarry, value & 0xf == 0);

        reg.write(self, new_value);
    }

    fn dec16(&mut self, reg: Reg16) {
        reg.write(self, reg.read(self).wrapping_sub(1));
    }

    fn rl(&mut self, reg: Register) {
        let value = reg.read(self);
        let carry = if self.registers.flag(Flag::Carry) {
            1
        } else {
            0
        };
        let new_value = (value << 1) | carry;

        self.registers.update_flag(Flag::Zero, new_value == 0);
        self.registers.update_flag(Flag::Sub, false);
        self.registers.update_flag(Flag::HalfCarry, false);
        self.registers.update_flag(Flag::Carry, value & 0x80 != 0);

        reg.write(self, new_value);
    }

    fn rlc(&mut self, reg: Register) {
        let value = reg.read(self);
        let new_value = value.rotate_left(1);

        self.registers.update_flag(Flag::Zero, new_value == 0);
        self.registers.update_flag(Flag::Sub, false);
        self.registers.update_flag(Flag::HalfCarry, false);
        self.registers.update_flag(Flag::Carry, value & 0x80 != 0);

        reg.write(self, new_value);
    }

    fn rr(&mut self, reg: Register) {
        let value = reg.read(self);
        let carry = if self.registers.flag(Flag::Carry) {
            1
        } else {
            0
        };
        let new_value = (carry << 7) | (value >> 1);

        self.registers.update_flag(Flag::Zero, new_value == 0);
        self.registers.update_flag(Flag::Sub, false);
        self.registers.update_flag(Flag::HalfCarry, false);
        self.registers.update_flag(Flag::Carry, value & 0x01 != 0);

        reg.write(self, new_value);
    }

    fn rrc(&mut self, reg: Register) {
        let value = reg.read(self);
        let new_value = value.rotate_right(1);

        self.registers.update_flag(Flag::Zero, new_value == 0);
        self.registers.update_flag(Flag::Sub, false);
        self.registers.update_flag(Flag::HalfCarry, false);
        self.registers.update_flag(Flag::Carry, value & 0x01 != 0);

        reg.write(self, new_value);
    }

    fn xor(&mut self, target: Register, source: Register) {
        let source_value = source.read(self);
        let target_value = target.read(self);
        let new_target_value = target_value ^ source_value;

        self.registers
            .update_flag(Flag::Zero, new_target_value == 0);
        self.registers.update_flag(Flag::Sub, false);
        self.registers.update_flag(Flag::HalfCarry, false);
        self.registers.update_flag(Flag::Carry, false);

        target.write(self, new_target_value);
    }

    fn cpl(&mut self) {
        self.registers.update_flag(Flag::Sub, true);
        self.registers.update_flag(Flag::HalfCarry, true);
        self.registers.a = !self.registers.a;
    }
}

use std::fs;
use std::io::stdin;

fn main() {
    let v = fs::read("res/bootstrap.bin").unwrap();
    let mut emu = Emulator::default();
    for (dst, src) in emu.mmu.rom.iter_mut().zip(v) {
        *dst = src
    }

    let mut s = String::new();
    loop {
        emu.cpu_step();
        println!("{:?}", emu.registers);
        stdin().read_line(&mut s).unwrap();
        if s == "exit" {
            break;
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_register_update_flag() {
        let mut registers = Registers::default();
        registers.update_flag(Flag::Zero, true);
        assert_eq!(registers.f, 0b10000000);
        registers.update_flag(Flag::Sub, true);
        assert_eq!(registers.f, 0b11000000);
        registers.update_flag(Flag::HalfCarry, true);
        assert_eq!(registers.f, 0b11100000);
        registers.update_flag(Flag::Carry, true);
        assert_eq!(registers.f, 0b11110000);

        registers = Registers {
            f: 0x00f0,
            ..Default::default()
        };
        registers.update_flag(Flag::Zero, false);
        assert_eq!(registers.f, 0b01110000);
        registers.update_flag(Flag::Sub, false);
        assert_eq!(registers.f, 0b00110000);
        registers.update_flag(Flag::HalfCarry, false);
        assert_eq!(registers.f, 0b00010000);
        registers.update_flag(Flag::Carry, false);
        assert_eq!(registers.f, 0);
    }

    #[test]
    fn test_emu_ld_nn_sp() {
        let mut emu = Emulator::default();
        emu.write_word(0x0000, 0x1234);
        Reg16::SP.write(&mut emu, 0x4321);
        emu.ld_nn_sp();

        assert_eq!(emu.registers.pc, 2);
        assert_eq!(emu.read_word(0x1234), 0x4321);
    }

    #[test]
    fn test_emu_ldi_hl_a() {
        let mut emu = Emulator::default();
        emu.registers.a = 0xff;
        emu.ldi_hl_a();

        assert_eq!(emu.read_byte(0x0000), 0xff);
        assert_eq!(Reg16::HL.read(&emu), 0x0001);
    }

    #[test]
    fn test_emu_load_imm() {
        let mut emu = Emulator::default();
        emu.write_word(0x0000, 0xfeff);
        emu.load_immediate(Register::A);
        assert_eq!(emu.registers.a, 0xff);
        emu.load_immediate(Register::A);
        assert_eq!(emu.registers.a, 0xfe);
    }

    #[test]
    fn test_emu_load_immediate16() {
        let mut emu = Emulator::default();
        emu.write_word(0x0000, 0xfeff);
        emu.write_word(0x0002, 0xfcfd);
        emu.load_immediate16(Reg16::BC);
        assert_eq!(Reg16::BC.read(&emu), 0xfeff);
        emu.load_immediate16(Reg16::BC);
        assert_eq!(Reg16::BC.read(&emu), 0xfcfd);
    }

    #[test]
    fn test_load_register_from_address() {
        let mut emu = Emulator::default();
        emu.write_byte(0x1234, 0xff);
        Reg16::BC.write(&mut emu, 0x1234);
        emu.load(Register::A, Addr::BC);
        assert_eq!(emu.registers.a, 0xff);
    }

    #[test]
    fn test_store_register_to_address() {
        let mut emu = Emulator::default();
        Reg16::BC.write(&mut emu, 0x1234);
        emu.registers.a = 0xff;
        emu.load(Addr::BC, Register::A);
        assert_eq!(emu.read_byte(0x1234), 0xff);
    }

    #[test]
    fn test_emu_jr() {
        let mut emu = Emulator::default();
        emu.write_byte(0x0000, 0x12);
        emu.jr();
        assert_eq!(emu.registers.pc, 0x0013);
    }

    #[test]
    fn test_emu_jr_nz() {
        let mut emu = Emulator::default();
        emu.write_byte(0x0000, 0x12);

        emu.registers.update_flag(Flag::Zero, true);
        emu.jr_cond(JumpCond::NZ);
        assert_eq!(emu.registers.pc, 0);

        emu.registers.update_flag(Flag::Zero, false);
        emu.jr_cond(JumpCond::NZ);
        assert_eq!(emu.registers.pc, 0x0013);
    }

    #[test]
    fn test_emu_jr_z() {
        let mut emu = Emulator::default();
        emu.write_byte(0x0000, 0x12);

        emu.jr_cond(JumpCond::Z);
        assert_eq!(emu.registers.pc, 0);

        emu.registers.update_flag(Flag::Zero, true);
        emu.jr_cond(JumpCond::Z);
        assert_eq!(emu.registers.pc, 0x0013);
    }

    #[test]
    fn test_emu_jr_nc() {
        let mut emu = Emulator::default();
        emu.write_byte(0x0000, 0x12);

        emu.registers.update_flag(Flag::Carry, true);
        emu.jr_cond(JumpCond::NC);
        assert_eq!(emu.registers.pc, 0);

        emu.registers.update_flag(Flag::Carry, false);
        emu.jr_cond(JumpCond::NC);
        assert_eq!(emu.registers.pc, 0x0013);
    }

    #[test]
    fn test_emu_jr_c() {
        let mut emu = Emulator::default();
        emu.write_byte(0x0000, 0x12);

        emu.jr_cond(JumpCond::C);
        assert_eq!(emu.registers.pc, 0);

        emu.registers.update_flag(Flag::Carry, true);
        emu.jr_cond(JumpCond::C);
        assert_eq!(emu.registers.pc, 0x0013);
    }

    #[test]
    fn test_emu_add16() {
        let mut emu = Emulator::default();
        Reg16::BC.write(&mut emu, 0x1234);
        Reg16::HL.write(&mut emu, 0x2345);
        emu.add16(Reg16::HL, Reg16::BC);
        assert_eq!(Reg16::HL.read(&emu), 0x3579);
    }

    #[test]
    fn test_emu_add16_wraps() {
        let mut emu = Emulator::default();
        Reg16::BC.write(&mut emu, 0xffff);
        Reg16::HL.write(&mut emu, 0x0001);
        emu.add16(Reg16::HL, Reg16::BC);
        assert_eq!(Reg16::HL.read(&emu), 0x0000);
    }

    #[test]
    fn test_emu_add16_clears_sub_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Sub, true);
        emu.add16(Reg16::HL, Reg16::BC);
        assert_eq!(emu.registers.flag(Flag::Sub), false);
    }

    #[test]
    fn test_emu_add16_clears_half_carry_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::HalfCarry, true);
        emu.add16(Reg16::HL, Reg16::BC);
        assert_eq!(emu.registers.flag(Flag::HalfCarry), false);
    }

    #[test]
    fn test_emu_add16_sets_half_carry_flag() {
        let mut emu = Emulator::default();
        Reg16::BC.write(&mut emu, 0xffff);
        Reg16::HL.write(&mut emu, 0x0001);
        emu.add16(Reg16::HL, Reg16::BC);
        assert_eq!(emu.registers.flag(Flag::HalfCarry), true);
    }

    #[test]
    fn test_emu_add16_clears_carry_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Carry, true);
        emu.add16(Reg16::HL, Reg16::BC);
        assert_eq!(emu.registers.flag(Flag::Carry), false);
    }

    #[test]
    fn test_emu_add16_sets_carry_flag() {
        let mut emu = Emulator::default();
        Reg16::BC.write(&mut emu, 0xffff);
        Reg16::HL.write(&mut emu, 0x0001);
        emu.add16(Reg16::HL, Reg16::BC);
        assert_eq!(emu.registers.flag(Flag::Carry), true);
    }

    #[test]
    fn test_emu_inc_register() {
        let mut emu = Emulator::default();
        emu.inc(Register::A);
        assert_eq!(emu.registers.a, 1);
    }

    #[test]
    fn test_emu_inc_wraps() {
        let mut emu = Emulator::default();
        emu.registers.a = 0xff;
        emu.inc(Register::A);
        assert_eq!(emu.registers.a, 0);
    }

    #[test]
    fn test_emu_inc_clears_zero_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Zero, true);
        emu.inc(Register::A);
        assert_eq!(emu.registers.flag(Flag::Zero), false);
    }

    #[test]
    fn test_emu_inc_sets_zero_flag() {
        let mut emu = Emulator::default();
        emu.registers.a = 0xff;
        emu.inc(Register::A);
        assert_eq!(emu.registers.flag(Flag::Zero), true);
    }

    #[test]
    fn test_emu_inc_clears_sub_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Sub, true);
        emu.inc(Register::A);
        assert_eq!(emu.registers.flag(Flag::Sub), false);
    }

    #[test]
    fn test_emu_inc_sets_half_carry_flag() {
        let mut emu = Emulator::default();
        emu.registers.a = 0xff;
        emu.inc(Register::A);
        assert_eq!(emu.registers.flag(Flag::HalfCarry), true);
    }

    #[test]
    fn test_emu_inc16_register() {
        let mut emu = Emulator::default();
        Reg16::BC.write(&mut emu, 0x1234);
        emu.inc16(Reg16::BC);
        assert_eq!(Reg16::BC.read(&emu), 0x1235);
    }

    #[test]
    fn test_emu_inc16_wraps() {
        let mut emu = Emulator::default();
        Reg16::BC.write(&mut emu, 0xffff);
        emu.inc16(Reg16::BC);
        assert_eq!(Reg16::BC.read(&emu), 0);
    }

    #[test]
    fn test_emu_dec_register() {
        let mut emu = Emulator::default();
        emu.registers.a = 0xff;
        emu.dec(Register::A);
        assert_eq!(emu.registers.a, 0xfe);
    }

    #[test]
    fn test_emu_dec_wraps() {
        let mut emu = Emulator::default();
        emu.dec(Register::A);
        assert_eq!(emu.registers.a, 0xff);
    }

    #[test]
    fn test_emu_dec_clears_zero_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Zero, true);
        emu.dec(Register::A);
        assert_eq!(emu.registers.flag(Flag::Zero), false);
    }

    #[test]
    fn test_emu_dec_sets_zero_flag() {
        let mut emu = Emulator::default();
        emu.registers.a = 0x01;
        emu.dec(Register::A);
        assert_eq!(emu.registers.flag(Flag::Zero), true);
    }

    #[test]
    fn test_emu_dec_sets_sub_flag() {
        let mut emu = Emulator::default();
        emu.dec(Register::A);
        assert_eq!(emu.registers.flag(Flag::Sub), true);
    }

    #[test]
    fn test_emu_dec_sets_half_carry_flag() {
        let mut emu = Emulator::default();
        emu.dec(Register::A);
        assert_eq!(emu.registers.flag(Flag::HalfCarry), true);
    }

    #[test]
    fn test_emu_dec16_register() {
        let mut emu = Emulator::default();
        Reg16::BC.write(&mut emu, 0x1234);
        emu.dec16(Reg16::BC);
        assert_eq!(Reg16::BC.read(&emu), 0x1233);
    }

    #[test]
    fn test_emu_dec16_wraps() {
        let mut emu = Emulator::default();
        emu.dec16(Reg16::BC);
        assert_eq!(Reg16::BC.read(&emu), 0xffff);
    }

    #[test]
    fn test_emu_rl_register() {
        let mut emu = Emulator::default();
        emu.registers.a = 0x01;
        emu.rl(Register::A);
        assert_eq!(emu.registers.a, 0x02);
    }

    #[test]
    fn test_emu_rl_wraps_carry() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Carry, true);
        emu.registers.a = 0x01;
        emu.rl(Register::A);
        assert_eq!(emu.registers.a, 0x03);
    }

    #[test]
    fn test_emu_rl_clears_zero_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Zero, true);
        emu.registers.a = 0x01;
        emu.rl(Register::A);
        assert_eq!(emu.registers.flag(Flag::Zero), false);
    }

    #[test]
    fn test_emu_rl_sets_zero_flag() {
        let mut emu = Emulator::default();
        emu.rlc(Register::A);
        assert_eq!(emu.registers.flag(Flag::Zero), true);
    }

    #[test]
    fn test_emu_rl_clears_sub_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Sub, true);
        emu.rl(Register::A);
        assert_eq!(emu.registers.flag(Flag::Sub), false);
    }

    #[test]
    fn test_emu_rl_clears_half_carry_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::HalfCarry, true);
        emu.rl(Register::A);
        assert_eq!(emu.registers.flag(Flag::HalfCarry), false);
    }

    #[test]
    fn test_emu_rl_clears_carry_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Carry, true);
        emu.rl(Register::A);
        assert_eq!(emu.registers.flag(Flag::Carry), false);
    }

    #[test]
    fn test_emu_rl_sets_carry_flag() {
        let mut emu = Emulator::default();
        emu.registers.a = 0xff;
        emu.rlc(Register::A);
        assert_eq!(emu.registers.flag(Flag::Carry), true);
    }

    #[test]
    fn test_emu_rlc_register() {
        let mut emu = Emulator::default();
        emu.registers.a = 0x01;
        emu.rlc(Register::A);
        assert_eq!(emu.registers.a, 0x02);
    }

    #[test]
    fn test_emu_rlc_wraps() {
        let mut emu = Emulator::default();
        emu.registers.a = 0xfe;
        emu.rlc(Register::A);
        assert_eq!(emu.registers.a, 0xfd);
    }

    #[test]
    fn test_emu_rlc_clears_zero_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Zero, true);
        emu.registers.a = 0x01;
        emu.rlc(Register::A);
        assert_eq!(emu.registers.flag(Flag::Zero), false);
    }

    #[test]
    fn test_emu_rlc_sets_zero_flag() {
        let mut emu = Emulator::default();
        emu.rlc(Register::A);
        assert_eq!(emu.registers.flag(Flag::Zero), true);
    }

    #[test]
    fn test_emu_rlc_clears_sub_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Sub, true);
        emu.rlc(Register::A);
        assert_eq!(emu.registers.flag(Flag::Sub), false);
    }

    #[test]
    fn test_emu_rlc_clears_half_carry_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::HalfCarry, true);
        emu.rlc(Register::A);
        assert_eq!(emu.registers.flag(Flag::HalfCarry), false);
    }

    #[test]
    fn test_emu_rlc_clears_carry_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Carry, true);
        emu.rlc(Register::A);
        assert_eq!(emu.registers.flag(Flag::Carry), false);
    }

    #[test]
    fn test_emu_rlc_sets_carry_flag() {
        let mut emu = Emulator::default();
        emu.registers.a = 0xff;
        emu.rlc(Register::A);
        assert_eq!(emu.registers.flag(Flag::Carry), true);
    }

    #[test]
    fn test_emu_rr_register() {
        let mut emu = Emulator::default();
        emu.registers.a = 0x7f;
        emu.rr(Register::A);
        assert_eq!(emu.registers.a, 0x3f);
    }

    #[test]
    fn test_emu_rr_wraps_carry() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Carry, true);
        emu.registers.a = 0x02;
        emu.rr(Register::A);
        assert_eq!(emu.registers.a, 0x81);
    }

    #[test]
    fn test_emu_rr_clears_zero_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Zero, true);
        emu.registers.a = 0x02;
        emu.rr(Register::A);
        assert_eq!(emu.registers.flag(Flag::Zero), false);
    }

    #[test]
    fn test_emu_rr_sets_zero_flag() {
        let mut emu = Emulator::default();
        emu.rr(Register::A);
        assert_eq!(emu.registers.flag(Flag::Zero), true);
    }

    #[test]
    fn test_emu_rr_clears_sub_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Sub, true);
        emu.rr(Register::A);
        assert_eq!(emu.registers.flag(Flag::Sub), false);
    }

    #[test]
    fn test_emu_rr_clears_half_carry_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::HalfCarry, true);
        emu.rr(Register::A);
        assert_eq!(emu.registers.flag(Flag::HalfCarry), false);
    }

    #[test]
    fn test_emu_rr_clears_carry_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Carry, true);
        emu.rr(Register::A);
        assert_eq!(emu.registers.flag(Flag::Carry), false);
    }

    #[test]
    fn test_emu_rr_sets_carry_flag() {
        let mut emu = Emulator::default();
        emu.registers.a = 0x01;
        emu.rr(Register::A);
        assert_eq!(emu.registers.flag(Flag::Carry), true);
    }

    #[test]
    fn test_emu_rrc_register() {
        let mut emu = Emulator::default();
        emu.registers.a = 0x7f;
        emu.rrc(Register::A);
        assert_eq!(emu.registers.a, 0xbf);
    }

    #[test]
    fn test_emu_rrc_wraps() {
        let mut emu = Emulator::default();
        emu.registers.a = 0x01;
        emu.rrc(Register::A);
        assert_eq!(emu.registers.a, 0x80);
    }

    #[test]
    fn test_emu_rrc_clears_zero_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Zero, true);
        emu.registers.a = 0x02;
        emu.rrc(Register::A);
        assert_eq!(emu.registers.flag(Flag::Zero), false);
    }

    #[test]
    fn test_emu_rrc_sets_zero_flag() {
        let mut emu = Emulator::default();
        emu.rrc(Register::A);
        assert_eq!(emu.registers.flag(Flag::Zero), true);
    }

    #[test]
    fn test_emu_rrc_clears_sub_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Sub, true);
        emu.rrc(Register::A);
        assert_eq!(emu.registers.flag(Flag::Sub), false);
    }

    #[test]
    fn test_emu_rrc_clears_half_carry_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::HalfCarry, true);
        emu.rrc(Register::A);
        assert_eq!(emu.registers.flag(Flag::HalfCarry), false);
    }

    #[test]
    fn test_emu_rrc_clears_carry_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Carry, true);
        emu.rrc(Register::A);
        assert_eq!(emu.registers.flag(Flag::Carry), false);
    }

    #[test]
    fn test_emu_rrc_sets_carry_flag() {
        let mut emu = Emulator::default();
        emu.registers.a = 0x01;
        emu.rrc(Register::A);
        assert_eq!(emu.registers.flag(Flag::Carry), true);
    }

    #[test]
    fn test_emu_xor_register() {
        let mut emu = Emulator::default();
        emu.registers.a = 0x12;
        emu.xor(Register::A, Register::A);
        assert_eq!(emu.registers.a, 0x00);
    }

    #[test]
    fn test_emu_xor_clears_zero_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Zero, true);
        emu.registers.a = 0x12;
        emu.xor(Register::A, Register::B);
        assert_eq!(emu.registers.flag(Flag::Zero), false);
    }

    #[test]
    fn test_emu_xor_sets_zero_flag() {
        let mut emu = Emulator::default();
        emu.xor(Register::A, Register::A);
        assert_eq!(emu.registers.flag(Flag::Zero), true);
    }

    #[test]
    fn test_emu_xor_clears_sub_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Sub, true);
        emu.xor(Register::A, Register::A);
        assert_eq!(emu.registers.flag(Flag::Sub), false);
    }

    #[test]
    fn test_emu_xor_clears_half_carry_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::HalfCarry, true);
        emu.xor(Register::A, Register::A);
        assert_eq!(emu.registers.flag(Flag::HalfCarry), false);
    }

    #[test]
    fn test_emu_xor_clears_carry_flag() {
        let mut emu = Emulator::default();
        emu.registers.update_flag(Flag::Carry, true);
        emu.xor(Register::A, Register::A);
        assert_eq!(emu.registers.flag(Flag::Carry), false);
    }

    #[test]
    fn test_emu_cpl() {
        let mut emu = Emulator::default();
        Register::A.write(&mut emu, 0b10101010);
        emu.cpl();
        assert_eq!(emu.registers.a, 0b01010101);
    }

    #[test]
    fn test_emu_cpl_sets_sub_flag() {
        let mut emu = Emulator::default();
        emu.cpl();
        assert_eq!(emu.registers.flag(Flag::Sub), true);
    }

    #[test]
    fn test_emu_cpl_sets_half_carry_flag() {
        let mut emu = Emulator::default();
        emu.cpl();
        assert_eq!(emu.registers.flag(Flag::HalfCarry), true);
    }
}

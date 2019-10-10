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

#[derive(Copy, Clone)]
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

#[derive(Copy, Clone)]
enum Flag {
    Zero,
    Sub,
    HalfCarry,
    Carry,
}

#[derive(Copy, Clone)]
enum Register {
    A,
    B,
    C,
    D,
    E,
    H,
    L,
}

#[derive(Copy, Clone)]
enum RegisterPair {
    AF,
    BC,
    DE,
    HL,
}

impl Registers {
    fn read_register(&self, reg: Register) -> u8 {
        match reg {
            Register::A => self.a,
            Register::B => self.b,
            Register::C => self.c,
            Register::D => self.d,
            Register::E => self.e,
            Register::H => self.h,
            Register::L => self.l,
        }
    }

    fn write_register(&mut self, reg: Register, value: u8) {
        match reg {
            Register::A => self.a = value,
            Register::B => self.b = value,
            Register::C => self.c = value,
            Register::D => self.d = value,
            Register::E => self.e = value,
            Register::H => self.h = value,
            Register::L => self.l = value,
        }
    }

    fn read_register_pair(&self, reg: RegisterPair) -> u16 {
        match reg {
            RegisterPair::AF => ((self.a as u16) << 8) | (self.f as u16),
            RegisterPair::BC => ((self.b as u16) << 8) | (self.c as u16),
            RegisterPair::DE => ((self.d as u16) << 8) | (self.e as u16),
            RegisterPair::HL => ((self.h as u16) << 8) | (self.l as u16),
        }
    }

    fn write_register_pair(&mut self, reg: RegisterPair, value: u16) {
        match reg {
            RegisterPair::AF => {
                self.a = (value >> 8) as u8;
                self.f = value as u8;
            }
            RegisterPair::BC => {
                self.b = (value >> 8) as u8;
                self.c = value as u8;
            }
            RegisterPair::DE => {
                self.d = (value >> 8) as u8;
                self.e = value as u8;
            }
            RegisterPair::HL => {
                self.h = (value >> 8) as u8;
                self.l = value as u8;
            }
        }
    }

    fn update_flag(&mut self, flag: Flag, flag_set: bool) {
        let value = match flag {
            Flag::Zero => 0x80,
            Flag::Sub => 0x40,
            Flag::HalfCarry => 0x20,
            Flag::Carry => 0x10,
        };

        if flag_set {
            self.f |= value;
        } else {
            self.f &= !value;
        }
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

#[derive(Default)]
struct Emulator {
    gpu: GPU,
    mmu: MMU,
    registers: Registers,
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
        (self.read_byte(addr + 1) as u16) << 8 + self.read_byte(addr) as u16
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
            0x01 => self.ld_bc_nn(),
            0x02 => self.ld_bc_a(),
            0x03 => self.inc_bc(),
            0x04 => self.inc_b(),
            0x05 => self.dec_b(),
            0x06 => self.ld_b_n(),
            0x07 => self.rlc_a(),
            _ => unimplemented!(),
        }
    }

    fn ld_bc_nn(&mut self) {
        let nn = self.fetch_next_word();
        self.registers.write_register_pair(RegisterPair::BC, nn);
    }

    fn ld_bc_a(&mut self) {
        self.write_byte(
            self.registers.read_register_pair(RegisterPair::BC),
            self.registers.a,
        );
    }

    fn inc_bc(&mut self) {
        self.registers.write_register_pair(
            RegisterPair::BC,
            self.registers
                .read_register_pair(RegisterPair::BC)
                .wrapping_add(1),
        );
    }

    fn inc_b(&mut self) {
        self.inc(Register::B);
    }

    fn dec_b(&mut self) {
        self.dec(Register::B);
    }

    fn ld_b_n(&mut self) {
        let n = self.fetch_next_byte();
        self.registers.write_register(Register::B, n);
    }

    fn rlc_a(&mut self) {
        self.rlc(Register::A);
    }

    fn inc(&mut self, reg: Register) {
        let value = self.registers.read_register(reg);
        let new_value = value.wrapping_add(1);

        self.registers.update_flag(Flag::Zero, new_value == 0);
        self.registers.update_flag(Flag::Sub, false);
        self.registers
            .update_flag(Flag::HalfCarry, value & 0x0f == 0x0f);

        self.registers.write_register(reg, new_value);
    }

    fn dec(&mut self, reg: Register) {
        let value = self.registers.read_register(reg);
        let new_value = value.wrapping_sub(1);

        self.registers.update_flag(Flag::Zero, new_value == 0);
        self.registers.update_flag(Flag::Sub, true);
        self.registers
            .update_flag(Flag::HalfCarry, value & 0xf == 0);

        self.registers.write_register(reg, new_value);
    }

    fn rlc(&mut self, reg: Register) {
        let value = self.registers.read_register(reg);
        let new_value = value.rotate_left(1);

        self.registers.update_flag(Flag::Zero, new_value == 0);
        self.registers.update_flag(Flag::Sub, false);
        self.registers.update_flag(Flag::HalfCarry, false);
        self.registers.update_flag(Flag::Carry, value & 0x80 != 0);

        self.registers.write_register(reg, new_value);
    }
}

use std::fs;

fn main() {
    let v = fs::read("res/bootstrap.bin").unwrap();
    let mut emu = Emulator::default();
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
}

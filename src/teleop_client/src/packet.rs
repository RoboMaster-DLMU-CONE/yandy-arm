use bytemuck::{Pod, Zeroable};

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum YandyControlCmd {
    None = 0x00,
    // System
    Error = 0x01,
    SwitchEnable = 0x02,
    Reset = 0x03,
    // Mode
    SwitchFetch = 0x10,
    SwitchStore = 0x11,
    // Manual
    SwitchGrip = 0x20,
    // Debug
    ToggleHeld = 0x80,
    IncStore = 0x81,
    DecStore = 0x82,
}

impl YandyControlCmd {
    pub fn to_u8(self) -> u8 {
        self as u8
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Pod, Zeroable, Debug)]
pub struct YandyControlPack {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub cmd: u8,
}

impl Default for YandyControlPack {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            cmd: YandyControlCmd::None as u8,
        }
    }
}

pub struct PacketSerializer {
    sequence: u8,
}

impl PacketSerializer {
    pub fn new() -> Self {
        Self { sequence: 0 }
    }

    pub fn serialize(&mut self, packet: &YandyControlPack) -> Vec<u8> {
        let cmd_id: u16 = 0x0604;
        let data_bytes = bytemuck::bytes_of(packet);
        let data_size = data_bytes.len();
        
        let frame_header_size = 7;
        let frame_tail_size = 2;
        let total_size = frame_header_size + data_size + frame_tail_size;
        
        let mut buffer = vec![0u8; total_size];
        
        // Header
        buffer[0] = 0xA5; // SOF
        
        // CMD (Little Endian)
        buffer[1] = (cmd_id & 0xFF) as u8;
        buffer[2] = ((cmd_id >> 8) & 0xFF) as u8;
        
        // Data Size (Little Endian)
        let data_size_u16 = data_size as u16;
        buffer[3] = (data_size_u16 & 0xFF) as u8;
        buffer[4] = ((data_size_u16 >> 8) & 0xFF) as u8;
        
        // Seq
        buffer[5] = self.sequence;
        
        // Header CRC8 (over first 6 bytes)
        buffer[6] = crc8::calc(&buffer[0..6]);
        
        // Data
        buffer[7..7+data_size].copy_from_slice(data_bytes);
        
        // Frame CRC16 (over header + data)
        let crc16 = crc16::calc(&buffer[0..7+data_size]);
        let tail_idx = 7 + data_size;
        buffer[tail_idx] = (crc16 & 0xFF) as u8;
        buffer[tail_idx+1] = ((crc16 >> 8) & 0xFF) as u8;
        
        self.sequence = self.sequence.wrapping_add(1);
        
        buffer
    }
}

mod crc8 {
    const CRC8_INIT: u8 = 0x00;
    const CRC8_POLY: u8 = 0x07;

    pub fn calc(data: &[u8]) -> u8 {
        let mut crc = CRC8_INIT;
        for &byte in data {
            crc ^= byte;
            for _ in 0..8 {
                if (crc & 0x80) != 0 {
                    crc = (crc << 1) ^ CRC8_POLY;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc
    }
}

mod crc16 {
    const CRC16_INIT: u16 = 0xFFFF;
    const CRC16_POLY: u16 = 0x1021;

    pub fn calc(data: &[u8]) -> u16 {
        let mut crc = CRC16_INIT;
        for &byte in data {
            crc ^= (byte as u16) << 8;
            for _ in 0..8 {
                if (crc & 0x8000) != 0 {
                    crc = (crc << 1) ^ CRC16_POLY;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc
    }
}

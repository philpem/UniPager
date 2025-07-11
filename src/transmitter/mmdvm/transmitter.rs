use crate::config::Config;
use crate::transmitter::Transmitter;
use std::fmt;
use std::time::Duration;
use std::thread;
use serial::{self, SerialPort};
use std::str;

// Frame start byte
const MMDVM_FRAME_START: u8 = 0xE0;

// MMDVM command codes
const MMDVM_GET_VERSION: u8 = 0x00;
const MMDVM_SET_CONFIG:  u8 = 0x02;
const MMDVM_SET_MODE:    u8 = 0x03;
const MMDVM_SET_FREQ:    u8 = 0x04;
const MMDVM_POCSAG_DATA: u8 = 0x50;
const MMDVM_MODE_IDLE:   u8 = 0x00;

// MMDVM command response codes
const MMDVM_ACK:         u8 = 0x70;
const MMDVM_NACK:        u8 = 0x7F;

// Number of POCSAG codewords in each complete batch
const POCSAG_CWS_PER_BATCH: usize = 17;

// NACK reason codes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MmdvmNackReason {
    BadCommand,
    WrongMode,
    CommandTooLong,
    DataIncorrect,
    BufferFull,
    Unknown(u8)
}

impl From<u8> for MmdvmNackReason {
    fn from(value: u8) -> MmdvmNackReason {
        match value {
            1 => MmdvmNackReason::BadCommand,
            2 => MmdvmNackReason::WrongMode,
            3 => MmdvmNackReason::CommandTooLong,
            4 => MmdvmNackReason::DataIncorrect,
            5 => MmdvmNackReason::BufferFull,
            _ => MmdvmNackReason::Unknown(value)
        }
    }
}

impl fmt::Display for MmdvmNackReason {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let name = match *self {
            MmdvmNackReason::BadCommand => "Bad command".to_owned(),
            MmdvmNackReason::WrongMode => "Wrong mode".to_owned(),
            MmdvmNackReason::CommandTooLong => "Command too long".to_owned(),
            MmdvmNackReason::DataIncorrect => "Data incorrect".to_owned(),
            MmdvmNackReason::BufferFull => "Buffer full, enhance your calm".to_owned(),
            MmdvmNackReason::Unknown(val) => format!("Unknown NACK reason, value {:?}", val)
        };
        write!(f, "{}", name)
    }
}
impl std::error::Error for MmdvmNackReason {}


pub struct MMDVMTransmitter {
    serial: Box<serial::SerialPort>
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ResponseCode {
    Ack(u8),
    VersionString{desc: String, version: u8}
}

#[derive(Debug)]
pub enum ResponseFailure {
    Nack(u8, MmdvmNackReason),
    InvalidData(Vec<u8>),
    IoError(std::io::Error),
    Timeout
}


impl MMDVMTransmitter {
    pub fn new(config: &Config) -> MMDVMTransmitter {
        info!("Initializing MMDVM transmitter...");

        let mut serial = serial::open(&config.mmdvm.port).expect(
            "Unable to open serial port"
        );

        serial
            .configure(&serial::PortSettings {
                baud_rate: serial::BaudRate::Baud115200,
                char_size: serial::CharSize::Bits8,
                parity: serial::Parity::ParityNone,
                stop_bits: serial::StopBits::Stop1,
                flow_control: serial::FlowControl::FlowNone
            })
            .expect("Unable to configure serial port");

        serial.set_timeout(Duration::from_millis(500))
            .expect("Unable to set serial port timeout");

        let mut tx = MMDVMTransmitter { serial: Box::new(serial) };
        tx.init(config);
        tx
    }

    pub fn init(&mut self, config: &Config) {
        let inverted = config.mmdvm.inverted;
        let mut level = config.mmdvm.level;

        level = if level > 100.0 { 100.0 } else { level };
        level *= 2.55 + 0.5;

        self.send_cmd(MMDVM_GET_VERSION, &[]);

        if let Ok(ResponseCode::VersionString { desc, version }) = self.read_result() {
            info!("Connected to MMDVM with protocol version {:?}: '{:?}'", version, desc);
        } else {
            error!("Error when reading from the MMDVM");
        }

        self.send_cmd(MMDVM_SET_CONFIG, &[
            // Invert, deviation and duplex settings
            (inverted as u8) << 4 | 0x80,
            // Enable POCSAG and disable all other modes
            0x20,
            // TXdelay in 10ms units
            10,
            // Idle mode
            MMDVM_MODE_IDLE,
            // RXLevel (not needed)
            0,
            // CW ID TX level (not needed)
            0,
            // DMR color code (not needed)
            0,
            // DMR delay (not needed)
            0,
            // Was OscOffset (not needed)
            128,
            // DStar TX Level (not needed)
            0,
            // DMR TX Level (not needed)
            0,
            // YSF TX Level (not needed)
            0,
            // P25 TX Level (not needed)
            0,
            // TX DC offset (not needed)
            0,
            // RX DC offset (not needed)
            0,
            // NXDN TX Level (not needed)
            0,
            // YSF TX hang time (not needed)
            0,
            // POCSAG TX level
            level as u8
        ]);

        let _ = self.read_result();

        self.send_cmd(MMDVM_SET_FREQ, &[
            0x0,
            // freq_rx
            0x2C, 0xAD, 0x39, 0x1A,
            // freq_tx
            0x2C, 0xAD, 0x39, 0x1A,
            // rf_power,
            0xFF,
            // pocsag_freq_tx
            0x2C, 0xAD, 0x39, 0x1A,
        ]);

        let _ = self.read_result();
    }

    pub fn send_cmd(&mut self, cmd: u8, data: &[u8]) {
        let header = [
            MMDVM_FRAME_START,
            (data.len() + 3) as u8,
            cmd
        ];

        if self.serial.write_all(&header).is_err() {
            error!("Failed to write to MMDVM.");
            return;
        }

        if self.serial.write_all(&data).is_err() {
            error!("Failed to write to MMDVM.");
            return;
        }

        if self.serial.flush().is_err() {
            error!("Unable to flush serial port");
        }
    }

    pub fn read_result(&mut self) -> Result<ResponseCode, ResponseFailure> {
        let mut buffer = [0; 256];

        let mut bytes_received = 0;

        while bytes_received < 3 { 
            match self.serial.read(&mut buffer[bytes_received..]) {
                // Probably a serial port timeout
                Ok(0) => return Err(ResponseFailure::Timeout),
                // One or more bytes received
                Ok(n) => bytes_received += n,
                // I/O error
                Err(e) => { error!("Error when reading from the MMDVM (during packet header)"); return Err(ResponseFailure::IoError(e)) }
            }
        } 
    
        // Now buffer[0..3] (at least) is filled in:
        //  Check the magic number and packet type are valid
        //  If they are, then the length ought to be too.

        let final_length_needed = match &buffer[0..3] {
            // It's an Ack, this is 4 bytes length, and the payload (buffer[4]) is the command code
            [MMDVM_FRAME_START, length, MMDVM_ACK] |
            // It's a Nack, we need another 2 bytes: Command Code and Nack Reason
            [MMDVM_FRAME_START, length, MMDVM_NACK] |
            // It's a Version String, we accept this
            [MMDVM_FRAME_START, length, MMDVM_GET_VERSION] => *length as usize,
            // It's not valid  
            _ => {
                let mut b: Vec<u8> = Vec::new();
                b.extend_from_slice(&buffer);
                return Err(ResponseFailure::InvalidData(b))
            }
        };
               
        // Check the length is valid: should be at least 3 bytes (FRAME_START, length, code)
        if final_length_needed < 3 {
            let mut b: Vec<u8> = Vec::new();
            b.extend_from_slice(&buffer);
            return Err(ResponseFailure::InvalidData(b));
        }

        // Packet header (and thus the length) seems to be valid, read the payload
        while bytes_received < final_length_needed { 
            match self.serial.read(&mut buffer[bytes_received..]) {
                // Probably a serial port timeout
                Ok(0) => return Err(ResponseFailure::Timeout),
                // One or more bytes received
                Ok(n) => bytes_received += n,
                // I/O error
                Err(e) => { error!("Error when reading from the MMDVM"); return Err(ResponseFailure::IoError(e)) }
            }
        }     

        // === 

        // We assume we have the complete packet now (assuming the MMDVM is following the protocol), so process it.

    
        match &buffer[0..final_length_needed] {
            [MMDVM_FRAME_START, _, MMDVM_ACK, ftype] => {
                info!("Received ACK ({:?})", ftype);
                Ok(ResponseCode::Ack(*ftype))
            }
            [MMDVM_FRAME_START, _, MMDVM_NACK, ftype, nack_code] => {
                let nack = MmdvmNackReason::from(*nack_code);
                warn!("Received NACK ({:?}): {:?} => {:?}", ftype, nack_code, nack);
                Err(ResponseFailure::Nack(*ftype, nack))
                
            }
            [MMDVM_FRAME_START, _, MMDVM_NACK, ftype] => {
                warn!("Received NACK ({:?}) without a reason code?! (Protocol Violation!)", ftype);
                let mut b: Vec<u8> = Vec::new();
                b.extend_from_slice(&buffer);
                Err(ResponseFailure::InvalidData(b))
            }
            [MMDVM_FRAME_START, _, MMDVM_GET_VERSION, version] => {

                let text: &str = str::from_utf8(&buffer[4..]).unwrap();
                let desc: String  = text.to_owned();
                
                Ok(ResponseCode::VersionString{version: *version, desc: desc})
            }
            _ => {
                warn!("READ_RESULT: Unknown frame received, buffer is {}", String::from_utf8_lossy(&buffer));
                let mut b: Vec<u8> = Vec::new();
                b.extend_from_slice(&buffer);
                Err(ResponseFailure::InvalidData(b))
            }
        }  
    }
}

impl Transmitter for MMDVMTransmitter {
    fn send(&mut self, data: &mut dyn Iterator<Item = u32>) {
        let mut buffer: Vec<u8> = Vec::with_capacity(252);

        loop {
            buffer.clear();

            // Send incoming codewords to the MMDVM in complete POCSAG batches (see ITU-R M.584-2 Annex 1)
            for word in data.take(POCSAG_CWS_PER_BATCH) {
                let bytes = word.to_be_bytes();
                buffer.extend_from_slice(&bytes);
            }

            if !buffer.is_empty() {
                let packet_header = [
                    MMDVM_FRAME_START,
                    (buffer.len() + 3) as u8,   // packet length includes the frame start, length and command bytes
                    MMDVM_POCSAG_DATA as u8,
                ];

                let mut result = Ok(ResponseCode::Ack(0));
                while let Err(ResponseFailure::Nack(_, MmdvmNackReason::BufferFull)) =  result {
                    if self.serial.write_all(&packet_header).is_err() {
                        error!("Unable to intialize MMDVM!");
                    }

                    if self.serial.write_all(&buffer[..]).is_err() {
                        error!("Unable to intialize MMDVM!");
                    }

                    if self.serial.flush().is_err() {
                        error!("Unable to flush serial port");
                    }

                    result = self.read_result();
                }
            }
            else {
                break;
            }
        }

        self.send_cmd(MMDVM_SET_MODE, &[MMDVM_MODE_IDLE]);

        let _ = self.read_result();
    }
}

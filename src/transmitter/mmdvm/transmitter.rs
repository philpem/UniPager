use crate::config::Config;
use crate::transmitter::Transmitter;
use std::io::ErrorKind;
use std::fmt;
use std::fmt::Write;
use std::time::Duration;
use serial::{self, SerialPort};
use std::str;

// Frame start byte
const MMDVM_FRAME_START: u8 = 0xE0;

// MMDVM command codes
const MMDVM_GET_VERSION: u8 = 0x00;
const MMDVM_GET_STATUS:  u8 = 0x01;
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
    SuppressedTimeout,
    VersionString{desc: String, version: u8},
    Status{protocols: u8, modem_state: u8, flags: u8, space: Vec<u8>}
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

        serial.set_timeout(Duration::from_millis(100))
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

        if let Ok(ResponseCode::VersionString { desc, version }) = self.read_result(false) {
            info!("Connected to MMDVM with protocol version {:?}: {:?}", version, desc);
        } else {
            error!("Error when reading from the MMDVM");
        }

        self.send_cmd(MMDVM_SET_CONFIG, &[
            // Invert, deviation and duplex settings
            (inverted as u8) << 4 | 0x80,
            // Enable POCSAG and disable all other modes
            0x20,
            // TXdelay in 10ms units - delay between PTT and start of preamble
            50,
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
        let _ = self.read_result(false);

        self.send_cmd(MMDVM_SET_FREQ, &[
            0x0,
            // freq_rx - Little endian = 439987500
            0x2C, 0xAD, 0x39, 0x1A,
            // freq_tx - Little endian = 439987500
            0x2C, 0xAD, 0x39, 0x1A,
            // rf_power,
            0xFF,
            // pocsag_freq_tx - Little endian = 439987500
            0x2C, 0xAD, 0x39, 0x1A,
        ]);
        let _ = self.read_result(false);

        self.send_cmd(MMDVM_GET_STATUS, &[]);
        info!("Get Status response: {:?}", self.read_result(false));
    }

    pub fn send_cmd(&mut self, cmd: u8, data: &[u8]) {
        let header = [
            MMDVM_FRAME_START,
            (data.len() + 3) as u8,
            cmd
        ];

        let mut s = String::new();
        for byte in header.iter() {
            write!(&mut s, "{:02X} ", byte).expect("Unable to write");
        }
        write!(&mut s, "// ").expect("Unable to write");
        for byte in data.iter() {
            write!(&mut s, "{:02X} ", byte).expect("Unable to write");
        }
        warn!("SEND_CMD: buffer is {}", s);


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

    pub fn read_result(&mut self, timeout_ok: bool) -> Result<ResponseCode, ResponseFailure> {
        let mut buffer = [0; 256];

        let mut bytes_received = 0;

        while bytes_received < 3 { 
            match self.serial.read(&mut buffer[bytes_received..]) {
                // Probably a serial port timeout
                Ok(0) => return Err(ResponseFailure::Timeout),
                // One or more bytes received
                Ok(n) => bytes_received += n,
                // I/O error
                Err(e) => {
                    if e.kind() == ErrorKind::TimedOut {
                        if timeout_ok {
                            // Suppress the timeout and pretend everything is OK
                            // Used for sending data to the MMDVM
                            return Ok(ResponseCode::SuppressedTimeout)
                        } else {
                            error!("Timeout reading from the MMDVM (during packet header)");
                            return Err(ResponseFailure::Timeout)
                        }
                    } else {
                        error!("Error when reading from the MMDVM (during packet header)");
                        return Err(ResponseFailure::IoError(e))
                    }
                }
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
            // It's a Get Status response
            [MMDVM_FRAME_START, length, MMDVM_GET_STATUS] |
            // It's a Version String
            [MMDVM_FRAME_START, length, MMDVM_GET_VERSION] => *length as usize,
            // It's not valid  
            _ => {
                let mut b: Vec<u8> = Vec::new();
                b.extend_from_slice(&buffer);
                return Err(ResponseFailure::InvalidData(b[..bytes_received].to_vec()))
            }
        };
               
        // Check the length is valid: should be at least 3 bytes (FRAME_START, length, code)
        if final_length_needed < 3 {
            let mut b: Vec<u8> = Vec::new();
            b.extend_from_slice(&buffer);
            return Err(ResponseFailure::InvalidData(b[..bytes_received].to_vec()));
        }

        // Packet header (and thus the length) seems to be valid, read the payload
        while bytes_received < final_length_needed { 
            match self.serial.read(&mut buffer[bytes_received..]) {
                // Probably a serial port timeout
                Ok(0) => return Err(ResponseFailure::Timeout),
                // One or more bytes received
                Ok(n) => bytes_received += n,
                // I/O error
                Err(e) => {
                    if e.kind() == ErrorKind::TimedOut {
                        error!("Timeout reading from the MMDVM");
                        return Err(ResponseFailure::Timeout)
                    } else {
                        error!("Error when reading from the MMDVM");
                        return Err(ResponseFailure::IoError(e))
                    }
                }
            }
        }     

        // === 

        // We assume we have the complete packet now (assuming the MMDVM is following the protocol), so process it.

    
        match &buffer[0..4] {
            [MMDVM_FRAME_START, _, MMDVM_ACK, ftype] => {
                info!("Received ACK ({:?})", ftype);
                Ok(ResponseCode::Ack(*ftype))
            }
            [MMDVM_FRAME_START, _, MMDVM_NACK, ftype] => {
                if bytes_received < 5 {
                    warn!("Received NACK ({:?}) without a reason code?! (Protocol Violation!)", ftype);
                    let mut b: Vec<u8> = Vec::new();
                    b.extend_from_slice(&buffer);
                    return Err(ResponseFailure::InvalidData(b[..bytes_received].to_vec()))
                }

                let nack_code = &buffer[4];
                let nack = MmdvmNackReason::from(*nack_code);
                warn!("Received NACK ({:?}): {:?} => {:?}", ftype, nack_code, nack);
                Err(ResponseFailure::Nack(*ftype, nack))
                
            }
            [MMDVM_FRAME_START, _, MMDVM_GET_VERSION, version] => {
                let text: &str = str::from_utf8(&buffer[4..bytes_received]).unwrap();
                let desc: String  = text.to_owned();
                Ok(ResponseCode::VersionString{version: *version, desc: desc})
            }
            [MMDVM_FRAME_START, length, MMDVM_GET_STATUS, proto] => {
                Ok(ResponseCode::Status{protocols: *proto, modem_state: buffer[4], flags: buffer[5], space: buffer[6..bytes_received].to_vec()})
            }
            _ => {
                let mut s = String::new();
                for byte in buffer[..bytes_received].iter() {
                    write!(&mut s, "{:02X} ", byte).expect("Unable to write");
                }
                warn!("READ_RESULT: Unknown frame received, buffer is {}", s);

                let mut b: Vec<u8> = Vec::new();
                b.extend_from_slice(&buffer);
                Err(ResponseFailure::InvalidData(b[..bytes_received].to_vec()))
            }
        }  
    }
}

impl Transmitter for MMDVMTransmitter {
    fn send(&mut self, data: &mut dyn Iterator<Item = u32>) {
        let mut buffer: Vec<u8> = Vec::with_capacity(252);

        // Eat the preamble words and save the first non-preamble word
        let mut sw = 0xAAAAAAAA;
        let mut n = POCSAG_CWS_PER_BATCH - 1;
        while sw == 0xAAAAAAAA {
            for word in data.take(1) { sw = word; }
        }
        
        loop {
            buffer.clear();

            // Send the non-preamble word
            if sw != 0xAAAAAAAA {
                //let bytes = sw.to_be_bytes();
                let bytes = [
                    ((sw & 0xff000000) >> 24) as u8,
                    ((sw & 0x00ff0000) >> 16) as u8,
                    ((sw & 0x0000ff00) >> 8) as u8,
                    (sw & 0x000000ff) as u8,
                ];
                buffer.extend_from_slice(&bytes);
                sw = 0xAAAAAAAA;
            }

            // Send incoming codewords to the MMDVM in complete POCSAG batches (see ITU-R M.584-2 Annex 1)
            for word in data.take(n) {
                //let bytes = word.to_be_bytes();
                let bytes = [
                    ((word & 0xff000000) >> 24) as u8,
                    ((word & 0x00ff0000) >> 16) as u8,
                    ((word & 0x0000ff00) >> 8) as u8,
                    (word & 0x000000ff) as u8,
                ];
                buffer.extend_from_slice(&bytes);
            }
            n = POCSAG_CWS_PER_BATCH;

            if !buffer.is_empty() {
                // Send a GET STATUS command, we need one every 2 seconds to clear the watchdog timer on the MMDVM
                self.send_cmd(MMDVM_GET_STATUS, &[]);
                info!("Get Status response: {:?}", self.read_result(false));

                // TODO: Check if we have buffer space on the MMDVM and if not, wait

                // send the buffer
                let mut result = Err(ResponseFailure::Nack(0, MmdvmNackReason::BufferFull));
                while let Err(ResponseFailure::Nack(_, MmdvmNackReason::BufferFull)) = result {
                    info!("tx");
                    self.send_cmd(MMDVM_POCSAG_DATA, &buffer);

                    // We expect either nothing, or a NACK (buffer full)
                    result = self.read_result(true);
                    // TODO: Error check
                    info!("tx resp {:?}", result);
                }

                self.send_cmd(MMDVM_GET_STATUS, &[]);
                info!("Get Status response: {:?}", self.read_result(false));
            }
            else {
                break;
            }
        }
    }
}

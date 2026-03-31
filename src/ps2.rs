use core::mem;

use defmt::{error, info};
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Input, InputConfig, InputPin, Level, Output, OutputConfig, OutputPin, Pull},
};

use crate::ps2_controller_task::ps2_controller_task;

// ============================================================================
// PS2 Controller Driver (Library Code)
// ============================================================================

/// Button Constants (Active Low in protocol, but handled as boolean flags here)
pub mod buttons {
    pub const SELECT: u16 = 0x0001;
    pub const L3: u16 = 0x0002;
    pub const R3: u16 = 0x0004;
    pub const START: u16 = 0x0008;
    pub const UP: u16 = 0x0010;
    pub const RIGHT: u16 = 0x0020;
    pub const DOWN: u16 = 0x0040;
    pub const LEFT: u16 = 0x0080;
    pub const L2: u16 = 0x0100;
    pub const R2: u16 = 0x0200;
    pub const L1: u16 = 0x0400;
    pub const R1: u16 = 0x0800;
    pub const Y: u16 = 0x1000;
    pub const B: u16 = 0x2000;
    pub const A: u16 = 0x4000;
    pub const X: u16 = 0x8000;
}

#[derive(Clone, Copy, defmt::Format)]
pub enum Button {
    Select,
    L3,
    R3,
    Start,
    Up,
    Right,
    Down,
    Left,
    L2,
    R2,
    L1,
    R1,
    Y,
    B,
    A,
    X,
}
impl Button {
    // A helpful array to iterate over all possible buttons
    pub const ALL: [Button; 16] = [
        Button::Select,
        Button::L3,
        Button::R3,
        Button::Start,
        Button::Up,
        Button::Right,
        Button::Down,
        Button::Left,
        Button::L2,
        Button::R2,
        Button::L1,
        Button::R1,
        Button::Y,
        Button::B,
        Button::A,
        Button::X,
    ];
}

impl Into<u16> for Button {
    fn into(self) -> u16 {
        match self {
            Button::Select => buttons::SELECT,
            Button::L3 => buttons::L3,
            Button::R3 => buttons::R3,
            Button::Start => buttons::START,
            Button::Up => buttons::UP,
            Button::Right => buttons::RIGHT,
            Button::Down => buttons::DOWN,
            Button::Left => buttons::LEFT,
            Button::L2 => buttons::L2,
            Button::R2 => buttons::R2,
            Button::L1 => buttons::L1,
            Button::R1 => buttons::R1,
            Button::Y => buttons::Y,
            Button::B => buttons::B,
            Button::A => buttons::A,
            Button::X => buttons::X,
        }
    }
}

// Commands
const CMD_ENTER_CONFIG: [u8; 5] = [0x01, 0x43, 0x00, 0x01, 0x00];
const CMD_SET_MODE: [u8; 9] = [0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00];
const CMD_EXIT_CONFIG: [u8; 9] = [0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A];
const CMD_READ_DATA: [u8; 9] = [0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];

// Timing (Microseconds) - Matched to C++ #ifdef ESP32
const CTRL_CLK_DELAY: u32 = 5;
const CTRL_BYTE_DELAY: u32 = 18;

pub struct Ps2Controller<'a> {
    clk: Output<'a>,
    cmd: Output<'a>,
    att: Output<'a>,
    dat: Input<'a>,
    delay: Delay,
    pub raw_buttons: u16,
    analog_data: [u8; 4], // LX, LY, RX, RY
}

impl<'a: 'static> Ps2Controller<'a> {
    pub fn spawn(
        clk: impl OutputPin + 'a,
        cs: impl OutputPin + 'a,
        cmd: impl OutputPin + 'a,
        dat: impl InputPin + 'a,
        delay: Delay,
        spawner: &Spawner,
    ) {
        let clk = Output::new(clk, Level::High, OutputConfig::default());

        // CS (Attention)
        let att = Output::new(cs, Level::High, OutputConfig::default());

        // CMD (MOSI)
        let cmd = Output::new(cmd, Level::High, OutputConfig::default());

        // DAT (MISO)
        let dat = Input::new(dat, InputConfig::default().with_pull(Pull::Up));
        let mut controller = Self {
            clk,
            cmd,
            att,
            dat,
            delay,
            raw_buttons: 0xFFFF,
            analog_data: [128; 4], // Center stick default
        };

        // Initial pin state
        controller.cmd.set_high();
        controller.clk.set_high();
        controller.att.set_high();

        info!("Configuring Gamepad...");
        match controller.config_gamepad() {
            Ok(_) => info!("Success! Gamepad configured."),
            Err(_) => error!("Failed to configure gamepad."),
        }

        spawner.spawn(ps2_controller_task(controller)).unwrap();
    }

    /// Initializes the controller into Analog Mode (Red Light)
    pub fn config_gamepad(&mut self) -> Result<(), ()> {
        // Try a few times to sync
        self.read_gamepad();
        self.read_gamepad();

        // Enter Config Mode
        self.send_command_string(&CMD_ENTER_CONFIG);
        self.delay.delay_micros(CTRL_BYTE_DELAY);

        // Set Mode to Analog (Lock)
        self.send_command_string(&CMD_SET_MODE);
        self.delay.delay_micros(CTRL_BYTE_DELAY);

        // Exit Config Mode
        self.send_command_string(&CMD_EXIT_CONFIG);
        self.delay.delay_micros(CTRL_BYTE_DELAY);

        self.read_gamepad();

        Ok(())
    }

    /// Reads the current state of the gamepad
    pub fn read_gamepad(&mut self) -> bool {
        let cmd = CMD_READ_DATA; // Copy command to mutable buffer
        let mut response = [0u8; 9]; // Standard polling is 9 bytes usually

        self.cmd.set_high();
        self.clk.set_high();
        self.att.set_low(); // Enable joystick

        self.delay.delay_micros(CTRL_BYTE_DELAY);

        for i in 0..9 {
            response[i] = self.shift_in_out(cmd[i]);
        }

        // info!("Raw Response: {}", response);

        self.att.set_high(); // Disable joystick

        // Check header (0x73 = Analog Red LED, 0x41 = Digital)
        // 0x73, 0x79, etc are valid analog modes.
        let mode = response[1];

        // Combine button bytes (Byte 3 and 4)
        // Note: PS2 buttons are Active LOW.
        // Byte 3: Select, L3, R3, Start, Up, Right, Down, Left
        // Byte 4: L2, R2, L1, R1, Triangle, Circle, Cross, Square
        self.raw_buttons = (response[4] as u16) << 8 | (response[3] as u16);

        // Parse Analog Sticks (Bytes 5, 6, 7, 8)
        self.analog_data[0] = response[7]; // LX
        self.analog_data[1] = response[8]; // LY
        self.analog_data[2] = response[5]; // RX
        self.analog_data[3] = response[6]; // RY

        // Return true if we detected a valid analog mode header (0x7X)
        (mode & 0xF0) == 0x70
    }

    pub fn button(&self, button: Button) -> bool {
        let mask: u16 = button.into();

        // Buttons are active LOW, so we invert raw_buttons.
        (!self.raw_buttons & mask) != 0
    }

    // d[x,y] origin point top up left, with 0-255 in each direction
    pub fn analog_lx(&self) -> u8 {
        self.analog_data[0]
    }
    pub fn analog_ly(&self) -> u8 {
        self.analog_data[1]
    }
    pub fn analog_rx(&self) -> u8 {
        self.analog_data[2]
    }
    pub fn analog_ry(&self) -> u8 {
        self.analog_data[3]
    }

    /// Helper to send specific command strings
    fn send_command_string(&mut self, data: &[u8]) {
        self.att.set_low();
        self.delay.delay_micros(CTRL_BYTE_DELAY);
        for &byte in data {
            self.shift_in_out(byte);
        }
        self.att.set_high();
        self.delay.delay_micros(CTRL_BYTE_DELAY);
    }

    /// LSB First SPI-like bit-banging
    fn shift_in_out(&mut self, byte: u8) -> u8 {
        let mut received: u8 = 0;

        for i in 0..8 {
            // 1. Setup Data (LSB First)
            if (byte & (1 << i)) != 0 {
                self.cmd.set_high();
            } else {
                self.cmd.set_low();
            }

            // 2. Clock Low (Active)
            self.clk.set_low();
            self.delay.delay_micros(CTRL_CLK_DELAY);

            // 3. Read Data
            if self.dat.is_high() {
                received |= 1 << i;
            }

            // 4. Clock High (Idle)
            self.clk.set_high();
            self.delay.delay_micros(CTRL_CLK_DELAY);
        }

        self.cmd.set_high();
        self.delay.delay_micros(CTRL_BYTE_DELAY);
        received
    }
}

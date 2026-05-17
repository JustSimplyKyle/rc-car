#![no_std]
#![feature(impl_trait_in_assoc_type, type_alias_impl_trait)]
#![no_main]

extern crate alloc;
extern crate rc_car;

use cobs::CobsDecoder;
use core::fmt::Write;
use defmt::error;
use defmt::warn;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::watch::Watch;
use embassy_time::Duration;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::system::Stack;
use esp_hal::Async;
use esp_radio::esp_now::BROADCAST_ADDRESS;
use heapless::String;

use embedded_hal_compat::ReverseCompat;

use defmt::info;
use esp_hal::i2c::master::Config as I2cConfig;
use esp_hal::i2c::master::I2c;
use esp_hal::time::Rate;

use sh1106::{prelude::*, Builder};

use embassy_executor::Spawner;
use embassy_futures::select::{self, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Timer;
use embedded_graphics::{
    mono_font::{ascii::FONT_5X8, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::Point,
    prelude::*,
    text::{Baseline, Text},
};
use esp_backtrace as _;
use esp_hal::ledc::{self, Ledc};
use esp_hal::{clock::CpuClock, delay::Delay, timer::timg::TimerGroup};
use esp_println as _;
use esp_rtos::start_second_core_with_stack_guard_offset;
use rc_car::motor::{self, ServoCmd};
use rc_car::ps2::Ps2Controller;
use rc_car::ps2_controller_task::PS2_GAMEPAD;
use static_cell::{make_static, StaticCell};

use defmt::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[derive(Clone, Copy, Default)]
struct DisplayState {
    angles: [u32; 5],
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct uni_gamepad_t {
    pub dpad: u8,
    pub axis_x: i32,
    pub axis_y: i32,
    pub axis_rx: i32,
    pub axis_ry: i32,
    pub brake: i32,
    pub throttle: i32,
    pub buttons: u16,
    pub misc_buttons: u8,
    pub gyro: [i32; 3usize],
    pub accel: [i32; 3usize],
}

impl uni_gamepad_t {
    pub fn pressed(&self, btn: Button) -> bool {
        match btn {
            Button::Dpad(dpad_direction) => {
                let id = match dpad_direction {
                    DpadDirection::Up => DPAD_UP,
                    DpadDirection::Down => DPAD_DOWN,
                    DpadDirection::Right => DPAD_RIGHT,
                    DpadDirection::Left => DPAD_LEFT,
                };
                (self.dpad & id) != 0
            }
            Button::Regular(btn) => {
                let id = match btn {
                    RegularButton::A => BUTTON_A,
                    RegularButton::B => BUTTON_B,
                    RegularButton::X => BUTTON_X,
                    RegularButton::Y => BUTTON_Y,
                    RegularButton::L1 => BUTTON_SHOULDER_L,
                    RegularButton::R1 => BUTTON_SHOULDER_R,
                    RegularButton::L2 => BUTTON_TRIGGER_L,
                    RegularButton::R2 => BUTTON_TRIGGER_R,
                    RegularButton::ThumbL => BUTTON_THUMB_L,
                    RegularButton::ThumbR => BUTTON_THUMB_R,
                };

                (self.buttons & id) != 0
            }
        }
    }
    pub fn status(&self) -> heapless::String<128> {
        use core::fmt::Write;
        use heapless::String;

        let mut out: String<128> = String::new();

        // D-Pad
        if self.pressed(Button::Dpad(DpadDirection::Up)) {
            let _ = write!(out, "UP ");
        }
        if self.pressed(Button::Dpad(DpadDirection::Down)) {
            let _ = write!(out, "DOWN ");
        }
        if self.pressed(Button::Dpad(DpadDirection::Left)) {
            let _ = write!(out, "LEFT ");
        }
        if self.pressed(Button::Dpad(DpadDirection::Right)) {
            let _ = write!(out, "RIGHT ");
        }

        // Face buttons + shoulders
        let checks = [
            (Button::Regular(RegularButton::A), "A"),
            (Button::Regular(RegularButton::B), "B"),
            (Button::Regular(RegularButton::X), "X"),
            (Button::Regular(RegularButton::Y), "Y"),
            (Button::Regular(RegularButton::L1), "L1"),
            (Button::Regular(RegularButton::R1), "R1"),
            (Button::Regular(RegularButton::L2), "L2"),
            (Button::Regular(RegularButton::R2), "R2"),
            (Button::Regular(RegularButton::ThumbL), "L3"),
            (Button::Regular(RegularButton::ThumbR), "R3"),
        ];

        for (btn, name) in checks {
            if self.pressed(btn) {
                let _ = write!(out, "{} ", name);
            }
        }

        // Optional: include analog hints (only if meaningful)
        if self.axis_x != 0 || self.axis_y != 0 {
            let _ = write!(out, "| LX:{} LY:{} ", self.axis_x, self.axis_y);
        }
        if self.axis_rx != 0 || self.axis_ry != 0 {
            let _ = write!(out, "| RX:{} RY:{} ", self.axis_rx, self.axis_ry);
        }
        if self.throttle != 0 || self.brake != 0 {
            let _ = write!(out, "| RT:{} LT:{} ", self.throttle, self.brake);
        }

        // Trim trailing space
        let _ = out.pop();

        out
    }
}

enum Button {
    Dpad(DpadDirection),
    Regular(RegularButton),
}

enum DpadDirection {
    Up,
    Down,
    Right,
    Left,
}

enum RegularButton {
    A,
    B,
    X,
    Y,
    L1,
    R1,
    L2,
    R2,
    ThumbL,
    ThumbR,
}

pub const DPAD_UP: u8 = 1;
pub const DPAD_DOWN: u8 = 2;
pub const DPAD_RIGHT: u8 = 4;
pub const DPAD_LEFT: u8 = 8;
pub const BUTTON_A: u16 = 1;
pub const BUTTON_B: u16 = 2;
pub const BUTTON_X: u16 = 4;
pub const BUTTON_Y: u16 = 8;
pub const BUTTON_SHOULDER_L: u16 = 16;
pub const BUTTON_SHOULDER_R: u16 = 32;
pub const BUTTON_TRIGGER_L: u16 = 64;
pub const BUTTON_TRIGGER_R: u16 = 128;
pub const BUTTON_THUMB_L: u16 = 256;
pub const BUTTON_THUMB_R: u16 = 512;

use esp_hal::uart::Uart;

// Capacity of 1: display core always gets the latest state, older frames are dropped.
static DISPLAY_WATCH: Watch<CriticalSectionRawMutex, DisplayState, 1> = Watch::new();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer1);

    Timer::after_secs(1).await;

    let wifi = peripherals.WIFI;
    let esp_radio_ctrl = make_static!(esp_radio::init().unwrap());
    let (mut controller, interfaces) =
        esp_radio::wifi::new(&esp_radio_ctrl, wifi, Default::default()).unwrap();
    controller.set_mode(esp_radio::wifi::WifiMode::Sta).unwrap();
    controller.start().unwrap();

    let mut esp_now = interfaces.esp_now;
    esp_now.set_channel(11).unwrap();

    let ledc = make_static!(Ledc::new(peripherals.LEDC));
    ledc.set_global_slow_clock(ledc::LSGlobalClkSource::APBClk);

    rc_car::timer_config!(Servo, 50, Duty12Bit);
    rc_car::timer_config!(Dc, 20000, Duty8Bit);

    let (mut dc, mut m1, mut m2, mut m3, mut m4, mut m5) =
        motor::ledc_motor::MotorSpawner::new(ledc)
            .spawn_dc_new(
                peripherals.GPIO10,
                peripherals.GPIO42,
                peripherals.GPIO40,
                Dc,
            )
            .spawn_pwm_new(spawner, peripherals.GPIO18, 60, 45, 85, Servo)
            .spawn_pwm_reuse(spawner, peripherals.GPIO15, 0, 10, 180, Servo)
            .spawn_pwm_reuse(spawner, peripherals.GPIO16, 40, 0, 180, Servo)
            .spawn_pwm_reuse(spawner, peripherals.GPIO17, 0, 0, 180, Servo)
            .spawn_pwm_reuse(spawner, peripherals.GPIO7, 80, 80 - 35, 80 + 35, Servo)
            .finish();

    dc.set_duty_percent(50);

    // Ps2Controller::spawn(
    //     peripherals.GPIO14,
    //     peripherals.GPIO13,
    //     peripherals.GPIO12,
    //     peripherals.GPIO11,
    //     Delay::new(),
    //     &spawner,
    // );

    let uart = Uart::new(peripherals.UART0, esp_hal::uart::Config::default())
        .unwrap()
        .with_rx(peripherals.GPIO14)
        .with_tx(peripherals.GPIO13)
        .into_async();

    let gamepad_channel = make_static!(Watch::new());

    spawner
        .spawn(uart_task(uart, gamepad_channel.sender()))
        .unwrap();

    let interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    // start_second_core_with_stack_guard_offset(
    //     peripherals.CPU_CTRL,
    //     interrupt.software_interrupt0,
    //     interrupt.software_interrupt1,
    //     make_static!(Stack::<32768>::new()),
    //     None,
    //     move || {
    //         let i2c_bus = I2c::new(
    //             peripherals.I2C0,
    //             I2cConfig::default().with_frequency(Rate::from_khz(400)),
    //         )
    //         .unwrap()
    //         .with_scl(peripherals.GPIO1)
    //         .with_sda(peripherals.GPIO2);

    //         let mut display: GraphicsMode<_> = Builder::new()
    //             .with_size(DisplaySize::Display128x32)
    //             .connect_i2c(i2c_bus.reverse())
    //             .into();
    //         display.init().unwrap();

    //         let text_style = MonoTextStyleBuilder::new()
    //             .font(&FONT_5X8)
    //             .text_color(BinaryColor::On)
    //             .build();

    //         let mut buf: String<32> = String::new();
    //         let mut state = DisplayState::default();
    //         let mut recv = DISPLAY_WATCH.receiver().unwrap();

    //         loop {
    //             if let Some(new_state) = recv.try_get() {
    //                 state = new_state;
    //             }

    //             display.clear();

    //             const POSITIONS: [(i32, i32); 5] = [(0, 0), (0, 9), (0, 18), (65, 0), (65, 9)];

    //             for (i, (x, y)) in POSITIONS.iter().enumerate() {
    //                 buf.clear();
    //                 write!(buf, "Servo {}: {}", i + 1, state.angles[i]).unwrap();
    //                 Text::with_baseline(&buf, Point::new(*x, *y), text_style, Baseline::Top)
    //                     .draw(&mut display)
    //                     .unwrap();
    //             }

    //             display.flush().unwrap();

    //             // Natural rate limit: the I2C flush above already takes ~10ms at 400kHz.
    //             // Add a small yield to avoid hammering the bus faster than needed.
    //             // This is a blocking spin on core 1 so we use a simple counter loop
    //             // rather than an async timer.
    //             for _ in 0..10_000u32 {
    //                 core::hint::spin_loop();
    //             }
    //         }
    //     },
    // );

    let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_millis(5));

    let mut recv = gamepad_channel.receiver().unwrap();

    loop {
        // let data = esp_now.receive_async().await;
        // let data_slice = data.data();

        // if data_slice.len() >= core::mem::size_of::<uni_gamepad_t>() {
        //     let gp =
        //         unsafe { core::ptr::read_unaligned(data_slice.as_ptr() as *const uni_gamepad_t) };

        //     info!("{}", gp.status());
        // } else {
        //     info!("Received packet too short!");
        // }

        let gp = recv.get().await;
        info!("{}", gp.status());

        let display_state = DisplayState {
            angles: [
                m1.angle().await,
                m4.angle().await,
                m3.angle().await,
                m2.angle().await,
                m5.angle().await,
            ],
        };

        DISPLAY_WATCH.sender().send(display_state);

        Timer::after(Duration::from_millis(5)).await;
    }
}

#[embassy_executor::task]
async fn uart_task(
    mut uart: Uart<'static, Async>,
    sender: embassy_sync::watch::Sender<'static, NoopRawMutex, uni_gamepad_t, 4>,
) {
    const GAMEPAD_SIZE: usize = 56;

    let mut out_buf = [0u8; GAMEPAD_SIZE];
    let mut byte = [0u8; 1];
    let mut decoder = CobsDecoder::new(&mut out_buf);

    loop {
        match uart.read_async(&mut byte).await {
            Err(e) => {
                error!("UART read error: {:?}", e);
                decoder = CobsDecoder::new(&mut out_buf);
                continue;
            }
            Ok(_) => {}
        }

        match decoder.feed(byte[0]) {
            Ok(None) => {
                // info!("state machine");
            } // still accumulating
            Ok(Some(len)) if len == GAMEPAD_SIZE => {
                let gp = unsafe { core::ptr::read(out_buf.as_ptr() as *const uni_gamepad_t) };
                sender.send(gp);

                decoder = CobsDecoder::new(&mut out_buf);
            }
            Ok(Some(len)) => {
                warn!("wrong frame size: {} (expected {})", len, GAMEPAD_SIZE);
                decoder = CobsDecoder::new(&mut out_buf);
            }
            Err(e) => {
                error!("failed to decode error, reason: {}", e);

                decoder = CobsDecoder::new(&mut out_buf);
            }
        }
    }
}

#![no_std]
#![feature(impl_trait_in_assoc_type, type_alias_impl_trait)]
#![no_main]

extern crate alloc;
extern crate rc_car;

use core::fmt::Write;
use embassy_sync::watch::Watch;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::system::Stack;
use heapless::String;
use rc_car::ps2::Button;

use embedded_hal_compat::ReverseCompat;

use defmt::info;
use esp_hal::i2c::master::Config as I2cConfig;
use esp_hal::i2c::master::I2c;
use esp_hal::time::Rate;

use sh1106::{prelude::*, Builder};

use embassy_executor::Spawner;
use embassy_futures::select::{self, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
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

esp_bootloader_esp_idf::esp_app_desc!();

#[derive(Clone, Copy, Default)]
struct DisplayState {
    angles: [u32; 5],
}

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

    Ps2Controller::spawn(
        peripherals.GPIO14,
        peripherals.GPIO13,
        peripherals.GPIO12,
        peripherals.GPIO11,
        Delay::new(),
        &spawner,
    );

    let interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    start_second_core_with_stack_guard_offset(
        peripherals.CPU_CTRL,
        interrupt.software_interrupt0,
        interrupt.software_interrupt1,
        make_static!(Stack::<32768>::new()),
        None,
        move || {
            let i2c_bus = I2c::new(
                peripherals.I2C0,
                I2cConfig::default().with_frequency(Rate::from_khz(400)),
            )
            .unwrap()
            .with_scl(peripherals.GPIO1)
            .with_sda(peripherals.GPIO2);

            let mut display: GraphicsMode<_> = Builder::new()
                .with_size(DisplaySize::Display128x32)
                .connect_i2c(i2c_bus.reverse())
                .into();
            display.init().unwrap();

            let text_style = MonoTextStyleBuilder::new()
                .font(&FONT_5X8)
                .text_color(BinaryColor::On)
                .build();

            let mut buf: String<32> = String::new();
            let mut state = DisplayState::default();
            let mut recv = DISPLAY_WATCH.receiver().unwrap();

            loop {
                if let Some(new_state) = recv.try_get() {
                    state = new_state;
                }

                display.clear();

                const POSITIONS: [(i32, i32); 5] = [(0, 0), (0, 9), (0, 18), (65, 0), (65, 9)];

                for (i, (x, y)) in POSITIONS.iter().enumerate() {
                    buf.clear();
                    write!(buf, "Servo {}: {}", i + 1, state.angles[i]).unwrap();
                    Text::with_baseline(&buf, Point::new(*x, *y), text_style, Baseline::Top)
                        .draw(&mut display)
                        .unwrap();
                }

                display.flush().unwrap();

                // Natural rate limit: the I2C flush above already takes ~10ms at 400kHz.
                // Add a small yield to avoid hammering the bus faster than needed.
                // This is a blocking spin on core 1 so we use a simple counter loop
                // rather than an async timer.
                for _ in 0..10_000u32 {
                    core::hint::spin_loop();
                }
            }
        },
    );

    let mut ps2 = PS2_GAMEPAD.receiver().unwrap();
    // dc.set_duty_percent(100);
    // dc.stop();

    loop {
        let ps2 = ps2.get().await;

        info!("{}", ps2.active_buttons());

        if ps2.pressed(Button::Up) {
            m1.send_cmd(ServoCmd::IncrementBy(2)).await;
        }
        if ps2.pressed(Button::Down) {
            m1.send_cmd(ServoCmd::DecrementBy(2)).await;
        }
        if ps2.pressed(Button::R1) {
            m2.send_cmd(ServoCmd::IncrementBy(2)).await;
        }
        if ps2.pressed(Button::R2) {
            m2.send_cmd(ServoCmd::DecrementBy(2)).await;
        }
        if ps2.pressed(Button::L1) {
            m3.send_cmd(ServoCmd::IncrementBy(2)).await;
        }
        if ps2.pressed(Button::L2) {
            m3.send_cmd(ServoCmd::DecrementBy(2)).await;
        }
        if ps2.pressed(Button::Left) {
            m4.send_cmd(ServoCmd::IncrementBy(2)).await;
        }
        if ps2.pressed(Button::Right) {
            m4.send_cmd(ServoCmd::DecrementBy(2)).await;
        }

        if ps2.right_analog_stick.x < 127 - 60 {
            m5.send_cmd(ServoCmd::TurnToAngle(80 - 35)).await;
        } else if ps2.right_analog_stick.x > 127 + 60 {
            m5.send_cmd(ServoCmd::TurnToAngle(80 + 35)).await;
        } else {
            m5.send_cmd(ServoCmd::TurnToAngle(80)).await;
        }

        // ── Reset to home position ────────────────────────────────────
        if ps2.pressed(Button::Start) {
            m2.send_cmd(ServoCmd::TurnToAngle(10)).await;
            m3.send_cmd(ServoCmd::TurnToAngle(40)).await;
            m4.send_cmd(ServoCmd::TurnToAngle(0)).await;
        }

        if ps2.left_analog_stick.y < 127 - 60 {
            dc.go_front();
        } else if ps2.left_analog_stick.y > 127 + 60 {
            dc.go_back();
        } else {
            dc.stop();
        }

        info!("{}", ps2.left_analog_stick);

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

        Timer::after_millis(50).await;
    }
}

#![no_std]
#![feature(impl_trait_in_assoc_type, type_alias_impl_trait)]
#![no_main]

extern crate alloc;
extern crate rc_car;

use core::fmt::Write;
use heapless::String;
use rc_car::ps2::Button;

use core::time::Duration;
use embedded_hal_compat::ForwardCompat;
use embedded_hal_compat::ReverseCompat;

use defmt::info;
// I2C
use esp_hal::i2c::master::Config as I2cConfig;
use esp_hal::i2c::master::I2c;
use esp_hal::time::Rate;

// OLED
// use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306Async};
use sh1106::{prelude::*, Builder};

// Embedded Graphics
use embassy_executor::Spawner;
use embassy_futures::select::{self, select};
use embassy_time::Timer;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::Point,
    prelude::*,
    text::{Baseline, Text},
};
use esp_backtrace as _;
use esp_hal::ledc::{self, Ledc};
use esp_hal::{clock::CpuClock, delay::Delay, timer::timg::TimerGroup};
use esp_println as _;
use rc_car::motor::{self, ServoCmd, Speed};
use rc_car::ps2::Ps2Controller;
use rc_car::ps2_controller_task::PS2_GAMEPAD;
use static_cell::make_static;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main(stack_size = 32768)]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    esp_alloc::heap_allocator!(size: 64 * 1024);
    // esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);

    // Initialize timers and RNG
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
            .spawn_pwm_new(spawner, peripherals.GPIO5, 0, 45, 85, Servo)
            .spawn_pwm_reuse(spawner, peripherals.GPIO6, 0, 0, 180, Servo)
            .spawn_pwm_reuse(spawner, peripherals.GPIO7, 65, 0, 180, Servo)
            .spawn_pwm_reuse(spawner, peripherals.GPIO15, 0, 0, 180, Servo)
            .spawn_pwm_reuse(spawner, peripherals.GPIO16, 102, 76, 128, Servo)
            .finish();

    Ps2Controller::spawn(
        peripherals.GPIO14,
        peripherals.GPIO13,
        peripherals.GPIO12,
        peripherals.GPIO11,
        Delay::new(),
        &spawner,
    );

    let mut ps2 = PS2_GAMEPAD.receiver().unwrap();

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
    let mut dirty = false;

    let text_style = MonoTextStyleBuilder::new()
        .font(&embedded_graphics::mono_font::ascii::FONT_5X8)
        .text_color(BinaryColor::On)
        .build();

    dc.set_duty_percent(100);
    dc.stop();

    let mut buf: String<64> = String::new();
    loop {
        buf.clear();
        let s = select(ps2.get(), Timer::after_millis(10)).await;
        let increment = async |motor: &motor::ledc_motor::PwmMotor<'_>| {
            motor.send_cmd(ServoCmd::IncrementBy(2)).await;
        };
        let decrement = async |motor: &motor::ledc_motor::PwmMotor<'_>| {
            motor.send_cmd(ServoCmd::DecrementBy(2)).await;
        };
        match s {
            select::Either::First(ps2) => {
                info!("{}", ps2.active_buttons());
                if ps2.pressed(Button::Up) {
                    increment(&m1).await;
                }
                if ps2.pressed(Button::Down) {
                    decrement(&m1).await;
                }
                if ps2.pressed(Button::Y) {
                    increment(&m2).await;
                }
                if ps2.pressed(Button::A) {
                    decrement(&m2).await;
                }
                if ps2.pressed(Button::X) {
                    increment(&m3).await;
                }
                if ps2.pressed(Button::B) {
                    decrement(&m3).await;
                }
                if ps2.pressed(Button::Left) {
                    increment(&m4).await;
                }
                if ps2.pressed(Button::Right) {
                    decrement(&m4).await;
                }
                if ps2.pressed(Button::L2) {
                    increment(&m5).await;
                }
                if ps2.pressed(Button::R2) {
                    decrement(&m5).await;
                }
                if ps2.pressed(Button::Start) {
                    m2.send_cmd(ServoCmd::TurnToAngle(0)).await;
                    m3.send_cmd(ServoCmd::TurnToAngle(65)).await;
                    m4.send_cmd(ServoCmd::TurnToAngle(0)).await;
                }
                if ps2.left_analog_stick.y < 127 - 60 {
                    dc.go_back();
                } else if ps2.left_analog_stick.y > 127 + 60 {
                    dc.go_front();
                } else {
                    dc.stop();
                }
                info!("{}", ps2.left_analog_stick);
                dirty = true;
            }
            select::Either::Second(()) => {}
        }

        if dirty {
            display.clear();

            write!(buf, "Servo 1: {}", m1.angle().await).unwrap();

            Text::with_baseline(&buf, Point::new(0, 0), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();

            buf.clear();

            write!(buf, "Servo 2: {}", m2.angle().await).unwrap();

            Text::with_baseline(&buf, Point::new(0, 9), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();

            buf.clear();

            write!(buf, "Servo 3: {}", m3.angle().await).unwrap();

            Text::with_baseline(&buf, Point::new(0, 18), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();

            buf.clear();

            write!(buf, "Servo 4: {}", m4.angle().await).unwrap();

            Text::with_baseline(&buf, Point::new(52 + 13, 0), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();

            buf.clear();

            write!(buf, "Servo 5: {}", m5.angle().await).unwrap();

            Text::with_baseline(&buf, Point::new(52 + 13, 9), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();

            display.flush().unwrap();
            dirty = false;
        }
    }
}

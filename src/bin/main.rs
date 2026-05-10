#![no_std]
#![feature(impl_trait_in_assoc_type, type_alias_impl_trait)]
#![no_main]

extern crate alloc;
extern crate rc_car;

use core::time::Duration;

use defmt::info;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::gpio::{Level, Output, OutputPin};
use esp_hal::ledc::{
    self,
    timer::{self, LSClockSource, TimerIFace},
    Ledc, LowSpeed,
};
use esp_hal::mcpwm::PeripheralClockConfig;
use esp_hal::time::Rate;
use esp_hal::{clock::CpuClock, delay::Delay, timer::timg::TimerGroup};
use esp_println as _;
use num::{clamp, FromPrimitive, ToPrimitive, Unsigned};
use rc_car::motor::{self, ServoCmd, Speed};
use rc_car::ps2::Ps2Controller;
use rc_car::ps2_controller_task::PS2_GAMEPAD;
use static_cell::make_static;

esp_bootloader_esp_idf::esp_app_desc!();

use esp_hal::ledc::channel::ChannelIFace;
#[esp_rtos::main]
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

    // let step_motor = motor::step_motor::StepMotor::new(
    //     spawner,
    //     peripherals.GPIO9,
    //     peripherals.GPIO10,
    //     make_static!(Signal::new()),
    // );

    rc_car::timer_config!(Servo, 50, Duty12Bit);
    rc_car::timer_config!(Dc, 20000, Duty8Bit);
    // rc_car::timer_config!(MotorPower, 20_000, Duty10Bit);

    // let s = motor::MotorSpawner::new_servo(peripherals.MCPWM0, spawner);

    // let (m1) = s.spawn(peripherals.GPIO12, 90).finish();

    let (pwm0,) = motor::ledc_motor::MotorSpawner::new(ledc)
        .spawn_pwm_new(
            spawner,
            peripherals.GPIO12,
            90,
            make_static!(embassy_sync::channel::Channel::new()),
            Servo,
        )
        // .spawn_dc_new(
        //     peripherals.GPIO1,
        //     peripherals.GPIO19,
        //     peripherals.GPIO20,
        //     Dc,
        // )
        .finish();

    // pwm0.send_cmd(ServoCmd::TurnToAngle(45)).await;

    // let s = motor::MotorSpawner::new_servo(peripherals.MCPWM0, spawner);

    // let (m1) = s
    //     .spawn(peripherals.GPIO7, 90)
    //     .spawn(peripherals.GPIO15, 90)
    //     // .spawn(peripherals.GPIO16, 90)
    //     .spawn(peripherals.GPIO17, 45) // [45,90] claw
    // .finish();

    // servo1.send_cmd(ServoCmd::TurnToAngle(30)).await;

    pwm0.send_cmd(ServoCmd::SetSpeed(Speed::Instant)).await;

    loop {
        pwm0.send_cmd(ServoCmd::TurnToAngle(180)).await;
        Timer::after_millis(3000).await;
        pwm0.send_cmd(ServoCmd::TurnToAngle(0)).await;
        Timer::after_millis(3000).await;
    }
}

pub struct StatefulAngleManager {
    pub current_angle: u32,
    pub min_angle: u32,
    pub max_angle: u32,
    pub step_size: u32,
}

impl StatefulAngleManager {
    pub fn new() -> Self {
        Self {
            current_angle: 0,
            min_angle: 0,
            max_angle: 180,
            step_size: 5,
        }
    }
    pub fn new_centered() -> Self {
        Self {
            current_angle: 90,
            min_angle: 0,
            max_angle: 180,
            step_size: 5,
        }
    }

    pub fn new_with_angle(current_angle: u32) -> Self {
        Self {
            current_angle,
            min_angle: 0,
            max_angle: 180,
            step_size: 5,
        }
    }

    fn increment(&mut self) {
        self.current_angle = (self.current_angle + self.step_size).min(self.max_angle);
    }

    fn decrement(&mut self) {
        self.current_angle = self
            .current_angle
            .saturating_sub(self.step_size)
            .max(self.min_angle);
    }
}

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
use esp_hal::ledc::{self, Ledc};
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

    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(40u32)).unwrap();
    let mut mcpwm = esp_hal::mcpwm::McPwm::new(peripherals.MCPWM0, clock_cfg);
    mcpwm.operator0.set_timer(&mcpwm.timer0);

    let step_motor = motor::step_motor::StepMotor::new(
        spawner,
        peripherals.GPIO12,
        peripherals.GPIO11,
        make_static!(Signal::new()),
    );

    use rc_car::motor::dc_motor::TimerConfigTrait;
    rc_car::timer_config!(MotorN20, 50_000, Duty8Bit);
    rc_car::timer_config!(MotorPower, 20_000, Duty10Bit);
    let ledc = make_static!(Ledc::new(peripherals.LEDC));

    let [mut servo1, mut servo2] = motor::dc_motor::MotorSpawner::new_pwm(ledc)
        .spawn_new(spawner, peripherals.GPIO18, 0, MotorN20)
        .spawn_new(spawner, peripherals.GPIO20, 0, MotorN20)
        .finish();

    servo1.send_cmd(ServoCmd::TurnToAngle(30)).await;

    loop {
        step_motor.rpm(300.0);
        Timer::after_millis(5000).await;

        step_motor.rpm(-300.0);
        Timer::after_millis(5000).await;
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

fn analog_to_servo(raw_value: u8) -> u8 {
    let mapped = map_range_int(raw_value, 255, 180);

    clamp(mapped, 0, 180)
}

fn map_range_int<T>(val: T, in_max: T, out_max: T) -> T
where
    T: ToPrimitive + FromPrimitive + Unsigned + Copy,
{
    let v = val.to_u32().unwrap();
    let im = in_max.to_u32().unwrap();
    let om = out_max.to_u32().unwrap();

    // Perform the calculation in u32 space with rounding
    let result = (v * om + im / 2) / im;

    T::from_u32(result).unwrap()
}

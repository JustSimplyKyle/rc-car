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
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::gpio::{Level, Output, OutputPin};
use esp_hal::ledc::{self};
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

// Stepper config
const TIMER_TOP: u16 = 19_999; // 20 000 counts → 50 Hz base clock
const STEP_PULSE_TICKS: u16 = 100; // pulse width (~100 counts high)

// Speed expressed as Hz = steps per second
const MIN_HZ: u32 = 50; // start speed
const MAX_HZ: u32 = 2_000; // top speed
const ACCEL_STEPS: u32 = 200;

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

    let step_motor = StepMotorBuilder::new(spawner, peripherals.GPIO12, peripherals.GPIO11);

    loop {
        // RPM_CHANNEL.send(200.0).await;
        // Timer::after_millis(500).await;

        step_motor.send(300.0).await;
        Timer::after_millis(5000).await;

        step_motor.send(-300.0).await;
        Timer::after_millis(5000).await;
    }
}

pub struct StepMotorBuilder;

impl StepMotorBuilder {
    pub fn new(
        spawner: Spawner,
        step_pin: impl OutputPin + 'static,
        dir_pin: impl OutputPin + 'static,
    ) -> Sender<'static, CriticalSectionRawMutex, f32, 4> {
        let step_pin = Output::new(step_pin, esp_hal::gpio::Level::Low, Default::default());
        let dir_pin = Output::new(dir_pin, esp_hal::gpio::Level::Low, Default::default());
        let channel = make_static!(Channel::new());

        spawner
            .spawn(motor_task(step_pin, dir_pin, channel.receiver()))
            .unwrap();

        channel.sender()
    }
}

// 17HS4401S: 1.8°/step = 200 steps/rev
const STEPS_PER_REV: u32 = 200;

/// Convert RPM to microsecond delay between steps
fn rpm_to_step_delay(rpm: f32) -> Duration {
    // steps/sec = rpm * steps_per_rev / 60
    let steps_per_sec = rpm * STEPS_PER_REV as f32 / 60.0;
    Duration::from_secs_f32(1.0 / steps_per_sec)
}

#[embassy_executor::task]
async fn motor_task(
    mut step_pin: Output<'static>,
    mut dir_pin: Output<'static>,
    receiver: Receiver<'static, CriticalSectionRawMutex, f32, 4>,
) {
    const ACCEL_RPM_PER_SEC: f32 = 200.0;
    const MIN_RPM: f32 = 10.0;
    let mut current_rpm: f32 = MIN_RPM;
    let mut target_rpm: f32 = MIN_RPM;

    loop {
        if let Ok(new_rpm) = receiver.try_receive() {
            target_rpm = new_rpm.abs().max(MIN_RPM) * new_rpm.signum();
        }

        info!("Current RPM: {}", current_rpm);

        let step_duration_secs = rpm_to_step_delay(current_rpm.abs()).as_secs_f32();
        let max_delta = ACCEL_RPM_PER_SEC * step_duration_secs;

        current_rpm = if target_rpm > current_rpm {
            (current_rpm + max_delta).min(target_rpm)
        } else {
            (current_rpm - max_delta).max(target_rpm)
        };

        dir_pin.set_level(if current_rpm.signum() >= 0.0 {
            Level::High
        } else {
            Level::Low
        });

        let half_delay = rpm_to_step_delay(current_rpm.abs()).as_micros() as u64 / 2;
        step_pin.set_high();
        Timer::after_micros(half_delay).await;
        step_pin.set_low();
        Timer::after_micros(half_delay).await;
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

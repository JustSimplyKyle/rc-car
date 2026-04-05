#![no_std]
#![feature(impl_trait_in_assoc_type, type_alias_impl_trait)]
#![no_main]

extern crate alloc;
extern crate rc_car;

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::ledc::{self};
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

    // let wifi_controller = make_static!(esp_radio::init().unwrap());

    // let stack = rc_car::web::start_wifi(
    //     wifi_controller,
    //     peripherals.WIFI,
    //     esp_hal::rng::Rng::new(),
    //     &spawner,
    // )
    // .await;

    // start_web_server(spawner, stack).await;

    let (servo_motor1, servo_motor2, servo_motor3) =
        motor::MotorSpawner::new_servo(peripherals.MCPWM0, spawner.clone())
            .spawn(peripherals.GPIO13, 30)
            .spawn(peripherals.GPIO14, None)
            .spawn(peripherals.GPIO15, 90)
            .finish();

    let (servo_motor4, servo_motor5) =
        motor::MotorSpawner::new_servo(peripherals.MCPWM1, spawner.clone())
            .spawn(peripherals.GPIO40, 90)
            .spawn(peripherals.GPIO41, None)
            .finish();

    let delay = Delay::new();

    Ps2Controller::spawn(
        peripherals.GPIO7,
        peripherals.GPIO6,
        peripherals.GPIO5,
        peripherals.GPIO4,
        delay,
        &spawner,
    );
    let mut receiver = PS2_GAMEPAD.receiver().unwrap();

    let mut speed_select = [Speed::Fast, Speed::Instant, Speed::Slow, Speed::Normal]
        .iter()
        .cycle();

    speed_select.next();

    use esp_hal::ledc::timer;
    use esp_hal::time::Rate;

    let ledc = make_static!(ledc::Ledc::new(peripherals.LEDC));
    ledc.set_global_slow_clock(ledc::LSGlobalClkSource::APBClk);

    let config_power_motor = ledc::timer::config::Config {
        frequency: esp_hal::time::Rate::from_hz(1000),
        duty: timer::config::Duty::Duty8Bit,
        clock_source: timer::LSClockSource::APBClk,
    };

    let config_n20 = timer::config::Config {
        frequency: Rate::from_khz(1),
        duty: timer::config::Duty::Duty8Bit,
        clock_source: timer::LSClockSource::APBClk,
    };

    use rc_car::motor::dc_motor::TimerConfigTrait;

    rc_car::timer_config!(MotorN20, 50_000, timer::config::Duty::Duty8Bit);
    rc_car::timer_config!(MotorPower, 20_000, timer::config::Duty::Duty10Bit);

    let [mut motor_n20, mut motor_power] = motor::dc_motor::MotorSpawner::new(ledc)
        .spawn_new(
            peripherals.GPIO16,
            peripherals.GPIO17,
            peripherals.GPIO18,
            MotorN20,
        )
        .spawn_new(
            peripherals.GPIO10,
            peripherals.GPIO11,
            peripherals.GPIO12,
            MotorPower,
        )
        .finish();

    let mut servo1_angle = StatefulAngleManager {
        current_angle: 30,
        min_angle: 5,
        max_angle: 70,
        step_size: 2,
    };

    let mut servo2_angle = StatefulAngleManager::new();
    let mut servo3_angle = StatefulAngleManager::new_centered();
    let mut servo4_angle = StatefulAngleManager::new_centered();

    motor_n20.set_duty_percent(60);
    motor_power.set_duty_percent(100);

    Timer::after_secs(1).await;

    loop {
        use defmt::info;
        use esp_hal::ledc::channel::ChannelIFace;
        use rc_car::ps2::Button;

        let ps2 = receiver.get().await;

        let buttons = ps2.active_buttons();

        let power_motor_speed = (ps2.left_analog_stick.y as i16 - 128);
        if power_motor_speed.abs() < 5 {
            motor_power.stop();
        } else {
            if power_motor_speed < 0 {
                info!("go front");
                motor_power.go_front();
            } else {
                info!("go back");
                motor_power.go_back();
            }

            let target_duty = map_range_int(power_motor_speed.abs() as u8, 128, 100);
            motor_power.set_duty_percent(target_duty);
            info!("Updated duty to: {}", target_duty);
        }

        for btn in buttons.iter().filter_map(|x| x.as_ref()) {
            info!("{} pressed", btn);
        }

        if ps2.right_analog_stick.x > 128 + 5 {
            servo1_angle.increment()
        }
        if ps2.right_analog_stick.x < 128 - 5 {
            servo1_angle.decrement();
        }
        if ps2.pressed(Button::L1) {
            servo2_angle.decrement();
        }
        if ps2.pressed(Button::R1) {
            servo2_angle.increment();
        }
        if ps2.pressed(Button::L2) {
            servo3_angle.decrement();
        }
        if ps2.pressed(Button::R2) {
            servo3_angle.increment();
        }
        if ps2.pressed(Button::Up) {
            servo4_angle.decrement();
        }
        if ps2.pressed(Button::Down) {
            servo4_angle.increment();
        }
        if ps2.any([Button::X, Button::B]) {
            if ps2.pressed(Button::X) {
                motor_n20.go_front();
            }
            if ps2.pressed(Button::B) {
                motor_n20.go_back();
            }
        } else {
            motor_n20.stop();
        }
        if ps2.pressed(Button::Y) {
            let speed = speed_select.next().unwrap();
            let s = match speed {
                Speed::Slow => 40,
                Speed::Normal => 60,
                Speed::Fast => 80,
                Speed::Instant => 100,
                Speed::Custom(duration) => unimplemented!(),
            };
            info!("{}", s);
            motor_n20.set_duty_percent(s);
        }

        info!("1: {}", servo1_angle.current_angle);
        info!("2: {}", servo2_angle.current_angle);
        info!("3: {}", servo3_angle.current_angle);
        info!("4: {}", servo4_angle.current_angle);
        servo_motor1
            .try_send(ServoCmd::TurnToAngle(servo1_angle.current_angle as i32))
            .ok();
        servo_motor2
            .try_send(ServoCmd::TurnToAngle(servo2_angle.current_angle as i32))
            .ok();
        servo_motor3
            .try_send(ServoCmd::TurnToAngle(servo3_angle.current_angle as i32))
            .ok();
        servo_motor4
            .try_send(ServoCmd::TurnToAngle(servo4_angle.current_angle as i32))
            .ok();

        Timer::after(embassy_time::Duration::from_millis(50)).await;
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

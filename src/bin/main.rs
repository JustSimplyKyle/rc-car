#![no_std]
#![feature(impl_trait_in_assoc_type, type_alias_impl_trait)]
#![no_main]

extern crate alloc;
extern crate rc_car;

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, delay::Delay, timer::timg::TimerGroup};
use esp_println as _;
use num::clamp;
use rc_car::motor::{self, ServoCmd, Speed};
use rc_car::ps2::Ps2Controller;
use rc_car::ps2_controller_task::PS2_GAMEPAD;
use rc_car::web::{start_web_server, COMMAND_CHANNEL};
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

    let (servo_motor1, servo_motor2) =
        motor::MotorSpawner::new_servo(peripherals.MCPWM0, spawner.clone())
            .spawn(peripherals.GPIO14)
            .spawn(peripherals.GPIO13)
            .finish();

    let (step_motor1, step_motor2) =
        motor::MotorSpawner::new_step(peripherals.MCPWM1, spawner.clone())
            .spawn(peripherals.GPIO15)
            .spawn(peripherals.GPIO16)
            .finish();

    let step_motor1 = step_motor1.into_motor(peripherals.GPIO18, peripherals.GPIO19);

    let delay = Delay::new();

    Ps2Controller::spawn(
        peripherals.GPIO35,
        peripherals.GPIO36,
        peripherals.GPIO37,
        peripherals.GPIO38,
        delay,
        &spawner,
    );
    let mut receiver = PS2_GAMEPAD.receiver().unwrap();

    let mut speed_select = [Speed::Fast, Speed::Instant, Speed::Slow, Speed::Normal]
        .iter()
        .cycle();

    loop {
        use defmt::info;
        use rc_car::ps2::Button;

        let status = receiver.get().await;

        let buttons = status.active_buttons();

        let left_servo_angle = analog_to_servo(status.left_analog_stick.x); // 0 to 255
        let right_servo_angle = analog_to_servo(status.right_analog_stick.x); // 0 to 255

        for btn in buttons.iter().filter_map(|x| x.as_ref()) {
            info!("{} pressed", btn);
        }

        info!("{}", right_servo_angle);

        if status.button(Button::X) {
            let target_speed = *speed_select.next().unwrap();
            info!("Setting speed to {}", target_speed);
            servo_motor1.send(ServoCmd::SetSpeed(target_speed)).await;
            servo_motor2.send(ServoCmd::SetSpeed(target_speed)).await;
        }

        servo_motor1
            .try_send(ServoCmd::TurnToAngle(left_servo_angle.into()))
            .ok();

        servo_motor2
            .try_send(ServoCmd::TurnToAngle(right_servo_angle.into()))
            .ok();

        Timer::after(embassy_time::Duration::from_millis(100)).await;
    }
}

fn analog_to_servo(raw_value: u8) -> u8 {
    // 1. Cast to a larger type (u32) to prevent overflow during multiplication
    // 2. Multiply before dividing to maintain precision
    // 3. Rounding: Add half the divisor (255 / 2 ≈ 127) before dividing
    let mapped = (raw_value as u32 * 180 + 127) / 255;

    // Use clamp to ensure we stay within servo limits
    clamp(mapped as u8, 0, 180)
}

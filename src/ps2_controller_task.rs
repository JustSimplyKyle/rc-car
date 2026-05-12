use core::cell::Cell;

use defmt::info;
use defmt::warn;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::Watch;
use embassy_time::Duration;

use crate::ps2::Button;
use crate::ps2::Ps2Controller;
use crate::web::CommandType;
use crate::web::Status;
use crate::web::COMMAND_CHANNEL;

use embassy_time::Timer;

struct ButtonLogic {
    pressed: bool,
}

impl ButtonLogic {
    fn new() -> Self {
        Self { pressed: false }
    }

    /// Checks current input against previous state.
    /// Returns Some(Status) ONLY if the state changed.
    fn update(&mut self, is_pressed: bool) -> Option<Status> {
        if is_pressed != self.pressed {
            self.pressed = is_pressed;
            Some(if is_pressed {
                Status::Pressed
            } else {
                Status::Released
            })
        } else {
            None
        }
    }

    fn force_release(&mut self) -> Option<Status> {
        if self.pressed {
            self.pressed = false;
            Some(Status::Released)
        } else {
            None
        }
    }
}

#[derive(Default, defmt::Format, Clone, Copy)]
pub struct Ps2GamepadStatus {
    pub left_analog_stick: AnalogStick,
    pub right_analog_stick: AnalogStick,
    raw_buttons: u16,
}

impl Ps2GamepadStatus {
    pub fn pressed(&self, button: Button) -> bool {
        let mask: u16 = button.into();

        // Buttons are active LOW, so we invert raw_buttons.
        (!self.raw_buttons & mask) != 0
    }
    pub fn any<const T: usize>(&self, buttons: impl Into<[Button; T]>) -> bool {
        let buttons = buttons.into();
        buttons.iter().any(|x| self.pressed(*x))
    }
    pub fn all<const T: usize>(&self, buttons: impl Into<[Button; T]>) -> bool {
        let buttons = buttons.into();
        buttons.iter().all(|x| self.pressed(*x))
    }
    pub fn active_buttons(&self) -> [Option<Button>; 16] {
        let mut active = [const { None }; 16]; // Max 16 buttons
        let mut i = 0;
        for &btn in &Button::ALL {
            if self.pressed(btn) {
                active[i] = Some(btn);
                i += 1;
            }
        }
        active
    }
}

#[derive(Default, Debug, defmt::Format, Clone, Copy)]
pub struct AnalogStick {
    pub x: u8,
    pub y: u8,
}

pub static PS2_GAMEPAD: Watch<CriticalSectionRawMutex, Ps2GamepadStatus, 1> = Watch::new();

#[embassy_executor::task]
pub async fn ps2_controller_task(mut ps2: Ps2Controller<'static>) {
    // if ps2.config_gamepad().is_err() {
    //     warn!("Initial PS2 config failed...");
    // } else {
    //     info!("starting task");
    // }
    let sender = PS2_GAMEPAD.sender();

    loop {
        if ps2.read_gamepad() {
            let ly = ps2.analog_ly();
            let lx = ps2.analog_lx();
            let ry = ps2.analog_ry();
            let rx = ps2.analog_rx();
            let left_analog_stick = AnalogStick { x: lx, y: ly };
            let right_analog_stick = AnalogStick { x: rx, y: ry };
            let s = Ps2GamepadStatus {
                left_analog_stick,
                right_analog_stick,
                raw_buttons: ps2.raw_buttons,
            };
            sender.send(s);
        } else {
            warn!("Controller lost! Reconnecting...");
            let _ = ps2.config_gamepad();
        }

        Timer::after(Duration::from_millis(10)).await;
    }
}

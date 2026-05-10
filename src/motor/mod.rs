use embassy_time::Duration;

pub mod ledc_motor;
pub mod mcpwm_motor;
pub mod step_motor;

fn duty_from_angle(deg: u32, max_duty_cycle: u32) -> u16 {
    let min_duty = (25 * max_duty_cycle) / 1000;
    let max_duty = (125 * max_duty_cycle) / 1000;
    let duty_gap = max_duty - min_duty;
    (min_duty + ((deg * duty_gap) / 180)) as u16
}

#[derive(Clone, Copy, defmt::Format)]
pub enum Speed {
    Slow,
    Normal,
    Fast,
    Instant,
    Custom(Duration),
}

pub enum ServoCmd {
    TurnToAngle(i32),
    SetSpeed(Speed),
}

use core::time::Duration;

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Timer;
use esp_hal::gpio::{Level, Output, OutputPin};

pub struct StepMotor(&'static Signal<CriticalSectionRawMutex, f32>);

impl StepMotor {
    pub fn new(
        spawner: Spawner,
        step_pin: impl OutputPin + 'static,
        dir_pin: impl OutputPin + 'static,
        signal: &'static Signal<CriticalSectionRawMutex, f32>,
    ) -> Self {
        let step_pin = Output::new(step_pin, esp_hal::gpio::Level::Low, Default::default());
        let dir_pin = Output::new(dir_pin, esp_hal::gpio::Level::Low, Default::default());

        spawner
            .spawn(motor_task(step_pin, dir_pin, signal))
            .unwrap();

        Self(signal)
    }
    pub fn rpm(&self, val: f32) {
        self.0.signal(val);
    }
}

/// Encapsulates S-curve acceleration state.
/// Call `.next(current, target)` every step to get the next RPM.
struct SCurveAccel {
    journey_start_rpm: f32,
    last_target: f32,
    accel_peak: f32, // RPM/s at midpoint
    accel_min: f32,  // RPM/s at journey edges
}

impl SCurveAccel {
    fn new(initial_rpm: f32, accel_peak: f32, accel_min: f32) -> Self {
        Self {
            journey_start_rpm: initial_rpm,
            last_target: initial_rpm,
            accel_peak,
            accel_min,
        }
    }

    /// Returns the next RPM to step at, given current RPM, target RPM,
    /// and how long the current step took (to scale accel correctly).
    fn next(&mut self, current_rpm: f32, target_rpm: f32) -> f32 {
        let step_secs = rpm_to_step_delay(current_rpm.abs()).as_secs_f32();

        // Reset journey origin whenever target changes
        if (target_rpm - self.last_target).abs() > 0.5 {
            self.journey_start_rpm = current_rpm;
            self.last_target = target_rpm;
        }

        let total_dist = (target_rpm - self.journey_start_rpm).abs();
        let progress = if total_dist > 0.5 {
            ((current_rpm - self.journey_start_rpm) / (target_rpm - self.journey_start_rpm))
                .clamp(0.0, 1.0)
        } else {
            1.0
        };

        let factor = Self::s_curve_factor(progress);
        let accel = self.accel_min + factor * (self.accel_peak - self.accel_min);
        let max_delta = accel * step_secs;

        if target_rpm > current_rpm {
            (current_rpm + max_delta).min(target_rpm)
        } else {
            (current_rpm - max_delta).max(target_rpm)
        }
    }

    fn smoothstep(t: f32) -> f32 {
        let t = t.clamp(0.0, 1.0);
        t * t * (3.0 - 2.0 * t)
    }

    fn s_curve_factor(progress: f32) -> f32 {
        let t = progress.clamp(0.0, 1.0);
        if t <= 0.5 {
            Self::smoothstep(t * 2.0)
        } else {
            Self::smoothstep((1.0 - t) * 2.0)
        }
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

const MIN_RPM: f32 = 10.0;
const MAX_RPM: f32 = 300.0;

#[embassy_executor::task]
async fn motor_task(
    mut step_pin: Output<'static>,
    mut dir_pin: Output<'static>,
    signal: &'static Signal<CriticalSectionRawMutex, f32>,
) {
    let mut current_rpm: f32 = MIN_RPM;
    let mut target_rpm: f32 = MIN_RPM;

    let mut accel = SCurveAccel::new(MIN_RPM, MAX_RPM, 25.0);

    loop {
        if let Some(new_rpm) = signal.try_take() {
            let clamped = new_rpm.abs().clamp(MIN_RPM, MAX_RPM) * new_rpm.signum();
            let reversing = clamped.signum() != current_rpm.signum();
            target_rpm = if reversing {
                MIN_RPM * current_rpm.signum() // brake to zero-side first
            } else {
                clamped
            };
        }

        current_rpm = accel.next(current_rpm, target_rpm);

        dir_pin.set_level(if current_rpm >= 0.0 {
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

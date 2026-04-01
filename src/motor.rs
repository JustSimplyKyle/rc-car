use defmt::info;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{self, Receiver, Sender};
use embassy_sync::mutex::Mutex;
use embassy_time::Duration;
use embedded_hal::pwm::SetDutyCycle;
use esp_hal::gpio::{Level, Output, OutputPin};
use esp_hal::ledc::{self, timer, Ledc, LowSpeed};
use esp_hal::mcpwm::operator::{PwmPin, PwmPinConfig};
use esp_hal::mcpwm::timer::{PwmWorkingMode, Timer};
use esp_hal::mcpwm::{self, McPwm, PeripheralClockConfig, PwmPeripheral};
use esp_hal::peripherals::MCPWM0;
use esp_hal::time::Rate;
use num::range_step;
use static_cell::make_static;
use static_cell::StaticCell;

use core::convert::Infallible;
use core::iter;
use core::marker::PhantomData;
use core::mem::MaybeUninit;

use crate::mk_static;

const CHANNEL_SIZE: usize = 4;
const INITIAL_ANGLE: i32 = 0;

fn duty_from_angle(deg: u32, max_duty_cycle: u32) -> u16 {
    let min_duty = (25 * max_duty_cycle) / 1000;
    let max_duty = (125 * max_duty_cycle) / 1000;
    let duty_gap = max_duty - min_duty;
    (min_duty + ((deg * duty_gap) / 180)) as u16
}

pub enum ErasedPwmPin<'d, PWM: PwmPeripheral> {
    Op0(PwmPin<'d, PWM, 0, true>),
    Op1(PwmPin<'d, PWM, 1, true>),
    Op2(PwmPin<'d, PWM, 2, true>),
}

macro_rules! apply_pwm {
    ($enum_val:expr, $method:ident ( $($arg:expr),* )) => {
        match $enum_val {
            Self::Op0(p) => p.$method($($arg),*),
            Self::Op1(p) => p.$method($($arg),*),
            Self::Op2(p) => p.$method($($arg),*),
        }
    };
}

impl<'d, PWM: PwmPeripheral> ErasedPwmPin<'d, PWM> {
    fn set_timestamp(&mut self, timestamp: u16) {
        apply_pwm!(self, set_timestamp(timestamp));
    }
    fn set_duty_cycle_percent(&mut self, percent: u8) -> Result<(), Infallible> {
        apply_pwm!(self, set_duty_cycle_percent(percent))
    }
    fn period(&self) -> u16 {
        apply_pwm!(self, period())
    }
    const fn operator_index(&self) -> usize {
        match self {
            ErasedPwmPin::Op0(_) => 0,
            ErasedPwmPin::Op1(_) => 1,
            ErasedPwmPin::Op2(_) => 2,
        }
    }
}

/// Associates a motor type with per-operator timer config.
/// `MAPPINGS[i]` = (period_ticks, frequency) for operator i.
pub trait PwmController<'d, PWM: PwmPeripheral>: Sized {
    const MAPPINGS: [(u16, Rate); 3];

    fn timer_period(operator: usize) -> u16 {
        Self::MAPPINGS[operator].0
    }
    fn timer_frequency(operator: usize) -> Rate {
        Self::MAPPINGS[operator].1
    }

    fn new(pin: ErasedPwmPin<'d, PWM>, spawner: &Spawner) -> Self;
}

// ---------------------------------------------------------------------------
// Servo motor
// ---------------------------------------------------------------------------

pub type ServoMotor = Sender<'static, CriticalSectionRawMutex, ServoCmd, CHANNEL_SIZE>;

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

pub async fn servo_motor_loop<PWM: PwmPeripheral>(
    mut initial_angle: i32,
    mut speed: Speed,
    cmd: Receiver<'static, CriticalSectionRawMutex, ServoCmd, CHANNEL_SIZE>,
    mut pwm_pin: ErasedPwmPin<'static, PWM>,
) {
    let mut target_angle = initial_angle;
    loop {
        match cmd.receive().await {
            ServoCmd::TurnToAngle(new_angle) => target_angle = new_angle,
            ServoCmd::SetSpeed(new_speed) => speed = new_speed,
        }
        let period = pwm_pin.period();

        for angle in range_step(
            initial_angle,
            target_angle,
            (target_angle - initial_angle).signum(),
        ) {
            pwm_pin.set_timestamp(duty_from_angle(angle as u32, period.into()));
            match speed {
                Speed::Slow => embassy_time::Timer::after_millis(15).await,
                Speed::Normal => embassy_time::Timer::after_millis(10).await,
                Speed::Fast => embassy_time::Timer::after_millis(5).await,
                Speed::Instant => {}
                Speed::Custom(time) => embassy_time::Timer::after(time).await,
            }
        }
        initial_angle = target_angle;
    }
}

pub enum StepCmd {
    Forward { duty: u8 },
    Backward { duty: u8 },
    Stop,
}

pub async fn step_motor_loop<PWM: PwmPeripheral>(
    cmd: Receiver<'static, CriticalSectionRawMutex, StepCmd, CHANNEL_SIZE>,
    mut motor: StepMotor<'static, PWM>,
) {
    loop {
        match cmd.receive().await {
            StepCmd::Forward { duty } => {
                motor.set_duty_cycle_percent(duty);
                motor.go_forward();
            }
            StepCmd::Backward { duty } => {
                motor.set_duty_cycle_percent(duty);
                motor.go_backward();
            }
            StepCmd::Stop => {
                motor.stop();
            }
        }
    }
}

macro_rules! impl_servo_motor_task {
    ($pwm:ty, $task_name:ident) => {
        #[embassy_executor::task(pool_size = 3)]
        pub async fn $task_name(
            initial_angle: i32,
            speed: Speed,
            cmd: Receiver<'static, CriticalSectionRawMutex, ServoCmd, CHANNEL_SIZE>,
            pwm_pin: ErasedPwmPin<'static, $pwm>,
        ) {
            servo_motor_loop(initial_angle, speed, cmd, pwm_pin).await;
        }

        impl PwmController<'static, $pwm> for ServoMotor {
            const MAPPINGS: [(u16, Rate); 3] = [
                (20_000, Rate::from_hz(50)),
                (20_000, Rate::from_hz(50)),
                (20_000, Rate::from_hz(50)),
            ];
            fn new(pin: ErasedPwmPin<'static, $pwm>, spawner: &Spawner) -> Self {
                static CHANNELS: [StaticCell<
                    channel::Channel<CriticalSectionRawMutex, ServoCmd, CHANNEL_SIZE>,
                >; 3] = [StaticCell::new(), StaticCell::new(), StaticCell::new()];

                let op_idx = pin.operator_index();
                info!("{}", op_idx);
                let channel = CHANNELS[op_idx].init(channel::Channel::new());

                spawner
                    .spawn($task_name(
                        INITIAL_ANGLE,
                        Speed::Fast,
                        channel.receiver(),
                        pin,
                    ))
                    .unwrap();
                channel.sender()
            }
        }
    };
}

macro_rules! impl_step_motor_task {
    ($pwm:ty, $task_name:ident) => {
        #[embassy_executor::task]
        pub async fn $task_name(
            cmd: Receiver<'static, CriticalSectionRawMutex, StepCmd, CHANNEL_SIZE>,
            motor: StepMotor<'static, $pwm>,
        ) {
            step_motor_loop::<$pwm>(cmd, motor).await;
        }

        impl<'d: 'static> StepMotor<'d, $pwm> {
            pub fn spawn_task(
                self,
                spawner: &Spawner,
            ) -> Sender<'static, CriticalSectionRawMutex, StepCmd, CHANNEL_SIZE> {
                let channel = make_static!(channel::Channel::new());
                spawner.spawn($task_name(channel.receiver(), self)).unwrap();
                channel.sender()
            }
        }
    };
}

impl_servo_motor_task!(esp_hal::peripherals::MCPWM0<'static>, servo_task_mcpwm0);
impl_step_motor_task!(esp_hal::peripherals::MCPWM0<'static>, step_task_mcpwm0);

impl_servo_motor_task!(esp_hal::peripherals::MCPWM1<'static>, servo_task_mcpwm1);
impl_step_motor_task!(esp_hal::peripherals::MCPWM1<'static>, step_task_mcpwm1);

// ---------------------------------------------------------------------------
// Stepper motor
// ---------------------------------------------------------------------------

pub struct StepMotorBuilder<'d, PWM: PwmPeripheral> {
    pwm_pin: ErasedPwmPin<'d, PWM>,
}

pub struct StepMotor<'d, PWM: PwmPeripheral> {
    pwm_pin: ErasedPwmPin<'d, PWM>,
    pina: Output<'d>,
    pinb: Output<'d>,
}

impl<'d, PWM: PwmPeripheral> PwmController<'d, PWM> for StepMotorBuilder<'d, PWM> {
    const MAPPINGS: [(u16, Rate); 3] = [
        (100, Rate::from_khz(1)),
        (100, Rate::from_khz(1)),
        (100, Rate::from_khz(1)),
    ];
    fn new(pin: ErasedPwmPin<'d, PWM>, _spawner: &Spawner) -> Self {
        Self { pwm_pin: pin }
    }
}

impl<'d, PWM: PwmPeripheral> StepMotorBuilder<'d, PWM> {
    pub fn into_motor(
        self,
        pina: impl OutputPin + 'd,
        pinb: impl OutputPin + 'd,
    ) -> StepMotor<'d, PWM> {
        StepMotor {
            pwm_pin: self.pwm_pin,
            pina: Output::new(pina, esp_hal::gpio::Level::Low, Default::default()),
            pinb: Output::new(pinb, esp_hal::gpio::Level::Low, Default::default()),
        }
    }
}

impl<'d, PWM: PwmPeripheral> StepMotor<'d, PWM> {
    pub fn set_duty_cycle_percent(&mut self, duty_percentage: u8) {
        self.pwm_pin
            .set_duty_cycle_percent(duty_percentage)
            .unwrap();
    }
    pub fn go_forward(&mut self) {
        self.pina.set_high();
        self.pinb.set_low();
    }
    pub fn go_backward(&mut self) {
        self.pina.set_low();
        self.pinb.set_high();
    }
    pub fn stop(&mut self) {
        self.pina.set_low();
        self.pinb.set_low();
    }
}

// ---------------------------------------------------------------------------
// Custom (mixed) motor
// ---------------------------------------------------------------------------

pub struct CustomMotor<'d, PWM: PwmPeripheral> {
    pwm_pin: ErasedPwmPin<'d, PWM>,
}

impl<'d, PWM: PwmPeripheral> PwmController<'d, PWM> for CustomMotor<'d, PWM> {
    const MAPPINGS: [(u16, Rate); 3] = [
        (20_000, Rate::from_hz(50)), // op0 — servo
        (100, Rate::from_khz(1)),    // op1 — stepper
        (100, Rate::from_khz(1)),    // op2 — stepper
    ];
    fn new(pin: ErasedPwmPin<'d, PWM>, _spawner: &Spawner) -> Self {
        Self { pwm_pin: pin }
    }
}

impl<'d, PWM: PwmPeripheral> CustomMotor<'d, PWM> {
    /// Only valid for op0 (servo-configured operator).
    pub fn set_angle(&mut self, angle: u32) {
        let op = self.pwm_pin.operator_index();
        assert_eq!(
            Self::MAPPINGS[op],
            (20_000, Rate::from_hz(50)), // Replaced old struct mappings with tuple literal checks
            "set_angle called on a non-servo operator"
        );
        let period = self.pwm_pin.period();
        self.pwm_pin
            .set_timestamp(duty_from_angle(angle, period.into()));
    }

    /// Only valid for op1/op2 (stepper-configured operators).
    pub fn set_duty_cycle_percent(&mut self, duty_percentage: u8) {
        let op = self.pwm_pin.operator_index();
        assert_eq!(
            Self::MAPPINGS[op],
            (100, Rate::from_khz(1)),
            "set_duty_cycle_percent called on a non-stepper operator"
        );
        self.pwm_pin
            .set_duty_cycle_percent(duty_percentage)
            .unwrap();
    }
}

/// # Example
/// ```rust
/// let (m1, m2, m3) = MotorSpawner::new_servo(peripherals.MCPWM0, spawner)
///     .spawn(peripherals.GPIO14)
///     .spawn(peripherals.GPIO13)
///     .spawn(peripherals.GPIO16)
///     .finish();
/// // m1, m2, m3 are now directly embassy channel Sender handles!
/// ```
pub struct MotorSpawner<'d, PWM: PwmPeripheral, const SLOTS: u8, Motor> {
    op0: Option<mcpwm::operator::Operator<'d, 0, PWM>>,
    op1: Option<mcpwm::operator::Operator<'d, 1, PWM>>,
    op2: Option<mcpwm::operator::Operator<'d, 2, PWM>>,
    timer0: Option<mcpwm::timer::Timer<0, PWM>>,
    timer1: Option<mcpwm::timer::Timer<1, PWM>>,
    timer2: Option<mcpwm::timer::Timer<2, PWM>>,
    // Collected motors, stored in reverse spawn order.
    // Index 0 = last spawned, index 2 = first spawned.
    collected: [Option<Motor>; 3],
    spawner: Spawner,
    _motor: PhantomData<Motor>,
}

// --- Construction -----------------------------------------------------------

impl<'d, PWM> MotorSpawner<'d, PWM, 3, ServoMotor>
where
    PWM: PwmPeripheral + 'd,
    ServoMotor: PwmController<'d, PWM>,
{
    pub fn new_servo(pwm: PWM, spawner: Spawner) -> Self {
        Self::new(pwm, spawner)
    }
}

impl<'d, PWM: PwmPeripheral> MotorSpawner<'d, PWM, 3, StepMotorBuilder<'d, PWM>> {
    pub fn new_step(pwm: PWM, spawner: Spawner) -> Self {
        Self::new(pwm, spawner)
    }
}

impl<'d, PWM: PwmPeripheral> MotorSpawner<'d, PWM, 3, CustomMotor<'d, PWM>> {
    pub fn new_custom(pwm: PWM, spawner: Spawner) -> Self {
        Self::new(pwm, spawner)
    }
}

impl<'d, PWM, const SLOTS: u8, M> MotorSpawner<'d, PWM, SLOTS, M>
where
    PWM: PwmPeripheral + 'd,
    M: PwmController<'d, PWM>,
{
    fn new(pwm: PWM, spawner: Spawner) -> MotorSpawner<'d, PWM, 3, M> {
        let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(2)).unwrap();
        let mcpwm = McPwm::new(pwm, clock_cfg);
        MotorSpawner {
            op0: Some(mcpwm.operator0),
            op1: Some(mcpwm.operator1),
            op2: Some(mcpwm.operator2),
            timer0: Some(mcpwm.timer0),
            timer1: Some(mcpwm.timer1),
            timer2: Some(mcpwm.timer2),
            collected: [None, None, None],
            _motor: PhantomData,
            spawner,
        }
    }

    fn start_timer_for_operator<const SLOT: u8>(
        operator_index: usize,
        timer: &mut Timer<SLOT, PWM>,
        clock_cfg: &PeripheralClockConfig,
    ) {
        info!("period: {}", M::timer_period(operator_index));
        info!("frequency: {}", M::timer_frequency(operator_index));
        let timer_clock_cfg = clock_cfg
            .timer_clock_with_frequency(
                M::timer_period(operator_index) - 1,
                PwmWorkingMode::Increase,
                M::timer_frequency(operator_index),
            )
            .unwrap();
        timer.start(timer_clock_cfg);
        info!("timer started");
    }
}

impl<'d, PWM, const SLOTS: u8, Motor> MotorSpawner<'d, PWM, SLOTS, Motor>
where
    PWM: PwmPeripheral + 'd,
    Motor: PwmController<'d, PWM>,
{
    fn spawn_op(
        mut self,
        operator_index: usize,
        pin: impl OutputPin + 'd,
    ) -> MotorSpawner<'d, PWM, SLOTS, Motor> {
        let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(2)).unwrap();

        let pwm_pin = match operator_index {
            0 => {
                let mut op = self.op0.take().unwrap();
                let mut timer = self.timer0.take().unwrap();
                Self::start_timer_for_operator(0, &mut timer, &clock_cfg);
                op.set_timer(&timer);
                ErasedPwmPin::Op0(op.with_pin_a(pin, PwmPinConfig::UP_ACTIVE_HIGH))
            }
            1 => {
                let mut op = self.op1.take().unwrap();
                let mut timer = self.timer1.take().unwrap();
                Self::start_timer_for_operator(1, &mut timer, &clock_cfg);
                op.set_timer(&timer);
                ErasedPwmPin::Op1(op.with_pin_a(pin, PwmPinConfig::UP_ACTIVE_HIGH))
            }
            2 => {
                let mut op = self.op2.take().unwrap();
                let mut timer = self.timer2.take().unwrap();
                Self::start_timer_for_operator(2, &mut timer, &clock_cfg);
                op.set_timer(&timer);
                ErasedPwmPin::Op2(op.with_pin_a(pin, PwmPinConfig::UP_ACTIVE_HIGH))
            }
            _ => unreachable!(),
        };

        self.collected[operator_index] = Some(Motor::new(pwm_pin, &self.spawner));
        self
    }
}

// --- spawn (three concrete stable transitions: 3→2, 2→1, 1→0) --------------

impl<'d, PWM, M, const SLOTS: u8> MotorSpawner<'d, PWM, SLOTS, M>
where
    PWM: PwmPeripheral + 'd,
    M: PwmController<'d, PWM>,
{
    fn update_slot<const NEW_SLOT: u8>(self) -> MotorSpawner<'d, PWM, NEW_SLOT, M> {
        MotorSpawner {
            op0: self.op0,
            op1: self.op1,
            op2: self.op2,
            timer0: self.timer0,
            timer1: self.timer1,
            timer2: self.timer2,
            collected: self.collected,
            spawner: self.spawner,
            _motor: PhantomData,
        }
    }
}

impl<'d, PWM, M> MotorSpawner<'d, PWM, 3, M>
where
    PWM: PwmPeripheral + 'd,
    M: PwmController<'d, PWM>,
{
    pub fn spawn(self, pin: impl OutputPin + 'd) -> MotorSpawner<'d, PWM, 2, M> {
        let next = self.spawn_op(0, pin);
        next.update_slot()
    }
}

impl<'d, PWM, M> MotorSpawner<'d, PWM, 2, M>
where
    PWM: PwmPeripheral + 'd,
    M: PwmController<'d, PWM>,
{
    pub fn spawn(self, pin: impl OutputPin + 'd) -> MotorSpawner<'d, PWM, 1, M> {
        let next = self.spawn_op(1, pin);
        next.update_slot()
    }
}

impl<'d, PWM, M> MotorSpawner<'d, PWM, 1, M>
where
    PWM: PwmPeripheral + 'd,
    M: PwmController<'d, PWM>,
{
    pub fn spawn(self, pin: impl OutputPin + 'd) -> MotorSpawner<'d, PWM, 0, M> {
        let next = self.spawn_op(2, pin);
        next.update_slot()
    }
}

impl<'d, PWM, M> MotorSpawner<'d, PWM, 0, M>
where
    PWM: PwmPeripheral + 'd,
    M: PwmController<'d, PWM>,
{
    pub fn finish(mut self) -> (M, M, M) {
        (
            self.collected[0].take().unwrap(),
            self.collected[1].take().unwrap(),
            self.collected[2].take().unwrap(),
        )
    }
}

impl<'d, PWM, M> MotorSpawner<'d, PWM, 1, M>
where
    PWM: PwmPeripheral + 'd,
    M: PwmController<'d, PWM>,
{
    pub fn finish(mut self) -> (M, M) {
        (
            self.collected[0].take().unwrap(),
            self.collected[1].take().unwrap(),
        )
    }
}

impl<'d, PWM, M> MotorSpawner<'d, PWM, 2, M>
where
    PWM: PwmPeripheral + 'd,
    M: PwmController<'d, PWM>,
{
    pub fn finish(mut self) -> M {
        self.collected[0].take().unwrap()
    }
}

pub mod dc_motor {
    use defmt::info;
    use embedded_hal::pwm::SetDutyCycle;
    use esp_hal::{
        gpio::{DriveMode, Level, Output, OutputPin},
        ledc::{
            self,
            channel::{self, Channel, ChannelIFace},
            timer::{self, LSClockSource, Timer, TimerIFace},
            LSGlobalClkSource, Ledc, LowSpeed,
        },
    };
    use static_cell::{make_static, StaticCell};

    use crate::mk_static;

    pub struct DCMotor<'a> {
        pub pina: Output<'a>,
        pub pinb: Output<'a>,
        pub pwm: Channel<'a, LowSpeed>,
    }

    impl<'a> DCMotor<'a> {
        fn new(
            pina: impl OutputPin + 'a,
            pinb: impl OutputPin + 'a,
            pwm: Channel<'a, LowSpeed>,
        ) -> Self {
            Self {
                pina: Output::new(pina, Level::Low, Default::default()),
                pinb: Output::new(pinb, Level::Low, Default::default()),
                pwm,
            }
        }
        pub fn set_duty_percent(&self, duty_percentage: u8) {
            self.pwm.set_duty(duty_percentage).unwrap();
        }
        pub fn go_front(&mut self) {
            self.pina.set_high();
            self.pinb.set_low();
        }
        pub fn go_back(&mut self) {
            self.pina.set_low();
            self.pinb.set_high();
        }
        pub fn stop(&mut self) {
            self.pina.set_low();
            self.pinb.set_low();
        }
    }

    /// The Typestate Builder
    pub struct MotorSpawner<'a, State> {
        ledc: &'a Ledc<'a>,
        // Replaces the `static TIMER_STORAGE`. Tracks which Timer (0-3) holds which Config.
        timer_configs: [Option<timer::config::Config<LSClockSource>>; 4],
        motors: State,
    }

    impl<'a, State> MotorSpawner<'a, State> {
        /// Helper to find an existing timer config, or assign an empty slot
        fn get_or_assign_timer(
            mut configs: [Option<timer::config::Config<LSClockSource>>; 4],
            new_config: timer::config::Config<LSClockSource>,
        ) -> (
            timer::Number,
            [Option<timer::config::Config<LSClockSource>>; 4],
        ) {
            let is_eq = |x: ledc::timer::config::Config<LSClockSource>,
                         y: ledc::timer::config::Config<LSClockSource>| {
                x.duty == y.duty && x.frequency == y.frequency
            };
            // 1. Find matching configuration to reuse
            for i in 0..4 {
                if let Some(cfg) = configs[i] {
                    // Assuming Config implements PartialEq. If not, compare fields manually.
                    if is_eq(cfg, new_config) {
                        return (Self::idx_to_timer(i), configs);
                    }
                }
            }

            // 2. If no match, assign to the first empty slot
            for i in 0..4 {
                if configs[i].is_none() {
                    configs[i] = Some(new_config);
                    return (Self::idx_to_timer(i), configs);
                }
            }

            panic!("All 4 LEDC timers are in use with different configurations!");
        }

        fn idx_to_timer(idx: usize) -> timer::Number {
            match idx {
                0 => timer::Number::Timer0,
                1 => timer::Number::Timer1,
                2 => timer::Number::Timer2,
                3 => timer::Number::Timer3,
                _ => unreachable!(),
            }
        }
        fn timer_to_idx(timer: timer::Number) -> usize {
            match timer {
                timer::Number::Timer0 => 0,
                timer::Number::Timer1 => 1,
                timer::Number::Timer2 => 2,
                timer::Number::Timer3 => 3,
            }
        }
    }

    static TIMER_STORAGE: [StaticCell<ledc::timer::Timer<'static, LowSpeed>>; 4] = [
        StaticCell::new(),
        StaticCell::new(),
        StaticCell::new(),
        StaticCell::new(),
    ];

    // Initial state: 0 motors
    impl<'a: 'static> MotorSpawner<'a, ()> {
        pub fn new(ledc: &'a Ledc<'a>) -> Self {
            Self {
                ledc,
                timer_configs: [None; 4],
                motors: (),
            }
        }

        pub fn spawn<PwmPin, DirA, DirB>(
            self,
            pwm_pin: PwmPin,
            pina: DirA,
            pinb: DirB,
            config: timer::config::Config<LSClockSource>,
        ) -> MotorSpawner<'a, [DCMotor<'a>; 1]>
        where
            PwmPin: OutputPin + 'a,
            DirA: OutputPin + 'a,
            DirB: OutputPin + 'a,
        {
            // 1. Resolve which hardware timer number to use
            let (timer_num, next_configs) = Self::get_or_assign_timer(self.timer_configs, config);
            info!("{}", timer_num);

            // 2. Instantiate and configure hardware timer locally
            let hw_timer = TIMER_STORAGE[Self::timer_to_idx(timer_num)]
                .init(self.ledc.timer::<LowSpeed>(timer_num));
            hw_timer.configure(config).unwrap();

            // 3. Configure the PWM Channel
            let mut pwm = self.ledc.channel(channel::Number::Channel0, pwm_pin);

            pwm.configure(channel::config::Config {
                timer: hw_timer,
                duty_pct: 0,
                drive_mode: DriveMode::PushPull,
            })
            .unwrap();

            MotorSpawner {
                ledc: self.ledc,
                timer_configs: next_configs,
                motors: [DCMotor::new(pina, pinb, pwm)],
            }
        }
    }

    macro_rules! impl_spawner {
        ($from:literal, $to:literal, $channel:ident, [$($m:ident),*]) => {
            impl<'a:'static > MotorSpawner<'a, [DCMotor<'a>; $from]> {
                pub fn spawn<PwmPin: 'a, DirA: 'a, DirB: 'a>(
                    self,
                    pwm_pin: PwmPin, pina: DirA, pinb: DirB,
                    config: timer::config::Config<LSClockSource>,
                ) -> MotorSpawner<'a, [DCMotor<'a>; $to]>
                where
                    PwmPin: OutputPin, DirA: OutputPin, DirB: OutputPin,
                {
                    let (timer_num, next_configs) = Self::get_or_assign_timer(self.timer_configs, config);
                    info!("{}", timer_num);

                    let hw_timer = TIMER_STORAGE[Self::timer_to_idx(timer_num)]
                        .init(self.ledc.timer::<LowSpeed>(timer_num));
                    hw_timer.configure(config).unwrap();

                    let mut pwm = self.ledc.channel(channel::Number::$channel, pwm_pin);

                    pwm.configure(channel::config::Config {
                        timer: hw_timer,
                        duty_pct: 0,
                        drive_mode: DriveMode::PushPull,
                    }).unwrap();

                    let new_motor = DCMotor::new(pina, pinb, pwm);

                    let [$($m),*] = self.motors;
                    MotorSpawner {
                        ledc: self.ledc,
                        timer_configs: next_configs,
                        motors: [$($m,)* new_motor],
                    }
                }

                /// Returns the perfectly sized array of built motors
                pub fn finish(self) -> [DCMotor<'a>; $from] {
                    self.motors
                }
            }
        };
    }

    // impl_spawner(0, 1, Channel0, m-1)
    impl_spawner!(1, 2, Channel1, [m0]);
    impl_spawner!(2, 3, Channel2, [m0, m1]);
    impl_spawner!(3, 4, Channel3, [m0, m1, m2]);
    impl_spawner!(4, 5, Channel4, [m0, m1, m2, m3]);
    impl_spawner!(5, 6, Channel5, [m0, m1, m2, m3, m4]);
    impl_spawner!(6, 7, Channel6, [m0, m1, m2, m3, m4, m5]);
    impl_spawner!(7, 8, Channel7, [m0, m1, m2, m3, m4, m5, m6]);

    impl<'a> MotorSpawner<'a, [DCMotor<'a>; 8]> {
        pub fn finish(self) -> [DCMotor<'a>; 8] {
            self.motors
        }
    }
}

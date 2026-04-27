use core::marker::PhantomData;

use defmt::info;
use embassy_executor::{task, Spawner};
use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex, CriticalSectionMutex},
    channel::{Receiver, Sender},
};
use embedded_hal::pwm::SetDutyCycle;
use esp_hal::{
    gpio::{DriveMode, Level, Output, OutputPin},
    ledc::{
        self,
        channel::{self, Channel, ChannelHW, ChannelIFace},
        timer::{self, LSClockSource, Timer, TimerIFace},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
    mcpwm::PwmPeripheral,
    time::Rate,
};
use num::range_step;
use static_cell::{make_static, StaticCell};

use crate::motor::{duty_from_angle, ServoCmd, Speed, MOTOR_MOVING};

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

pub trait TimerConfigTrait {
    const FREQUENCY: u32;
    const DUTY_RESOLUTION: timer::config::Duty;

    fn as_config() -> timer::config::Config<LSClockSource> {
        timer::config::Config {
            frequency: Rate::from_hz(Self::FREQUENCY),
            duty: Self::DUTY_RESOLUTION,
            clock_source: LSClockSource::APBClk,
        }
    }
}

#[macro_export]
macro_rules! timer_config {
    ($name:ident, $freq:expr, $duty:ident) => {
        pub struct $name;
        impl TimerConfigTrait for $name {
            const FREQUENCY: u32 = $freq;
            const DUTY_RESOLUTION: esp_hal::ledc::timer::config::Duty =
                esp_hal::ledc::timer::config::Duty::$duty;
        }
    };
}

pub struct End;
pub struct Cons<Head, Tail>(PhantomData<Head>, Tail);

// TimerList node — stores the actual timer ref at runtime
pub struct TimerEntry<Config, Tail> {
    timer: &'static timer::Timer<'static, LowSpeed>,
    tail: Tail,
    _config: PhantomData<Config>,
}

pub trait Contains<T, Index> {}

// Base: T is at head
impl<T, Tail> Contains<T, End> for TimerEntry<T, Tail> {}

// Recursive: T is in tail
impl<T, Index, Head, Tail> Contains<T, Cons<Index, ()>> for TimerEntry<Head, Tail> where
    Tail: Contains<T, Index>
{
}

pub trait FindTimer<Config, Index> {
    fn get_timer(&self) -> &'static timer::Timer<'static, LowSpeed>;
}

// Base: Config at head
impl<Config, Tail> FindTimer<Config, End> for TimerEntry<Config, Tail> {
    fn get_timer(&self) -> &'static timer::Timer<'static, LowSpeed> {
        self.timer
    }
}

// Recursive: Config in tail
impl<Config, Head, Tail, Index> FindTimer<Config, Cons<Index, ()>> for TimerEntry<Head, Tail>
where
    Tail: FindTimer<Config, Index>,
{
    fn get_timer(&self) -> &'static timer::Timer<'static, LowSpeed> {
        self.tail.get_timer()
    }
}

pub trait ReuseTimer<Config, Index> {
    fn get_timer(&self) -> &'static timer::Timer<'static, LowSpeed>;
}

impl<Config, Index, List> ReuseTimer<Config, Index> for List
where
    List: FindTimer<Config, Index>,
{
    fn get_timer(&self) -> &'static timer::Timer<'static, LowSpeed> {
        FindTimer::<Config, Index>::get_timer(self)
    }
}

// Case 2: Config not in list — allocate new slot, list grows
pub trait AllocTimer<Config> {
    type Output;
    fn alloc(
        self,
        ledc: &'static Ledc<'static>,
        slot: usize,
    ) -> (&'static timer::Timer<'static, LowSpeed>, Self::Output);
}

static TIMER_STORAGE: [StaticCell<ledc::timer::Timer<'static, LowSpeed>>; 4] = [
    StaticCell::new(),
    StaticCell::new(),
    StaticCell::new(),
    StaticCell::new(),
];

// Base: empty list
impl<Config: TimerConfigTrait> AllocTimer<Config> for End {
    type Output = TimerEntry<Config, End>;

    fn alloc(
        self,
        ledc: &'static Ledc<'static>,
        slot: usize,
    ) -> (&'static timer::Timer<'static, LowSpeed>, Self::Output) {
        let hw = ledc.timer::<LowSpeed>(idx_to_timer(slot));
        let timer_ref = TIMER_STORAGE[slot].init(hw);
        timer_ref.configure(Config::as_config()).unwrap();
        (
            timer_ref,
            TimerEntry {
                timer: timer_ref,
                tail: End,
                _config: PhantomData,
            },
        )
    }
}

fn idx_to_timer(s: usize) -> timer::Number {
    match s {
        0 => timer::Number::Timer0,
        1 => timer::Number::Timer1,
        2 => timer::Number::Timer2,
        3 => timer::Number::Timer3,
        _ => unreachable!(),
    }
}

// Recursive: skip head, alloc into tail
impl<Config: TimerConfigTrait, Head, Tail> AllocTimer<Config> for TimerEntry<Head, Tail>
where
    Tail: AllocTimer<Config>,
{
    type Output = TimerEntry<Head, <Tail as AllocTimer<Config>>::Output>;

    fn alloc(
        self,
        ledc: &'static Ledc<'static>,
        slot: usize,
    ) -> (&'static timer::Timer<'static, LowSpeed>, Self::Output) {
        let (timer, new_tail) = self.tail.alloc(ledc, slot);
        (
            timer,
            TimerEntry {
                timer: self.timer,
                tail: new_tail,
                _config: PhantomData,
            },
        )
    }
}

pub trait HasRoom {}
impl HasRoom for End {}
impl<H, T> HasRoom for TimerEntry<H, T> where T: HasRoom2 {}

trait HasRoom2 {}
impl HasRoom2 for End {}
impl<H, T> HasRoom2 for TimerEntry<H, T> where T: HasRoom3 {}

trait HasRoom3 {}
impl HasRoom3 for End {}
impl<H, T> HasRoom3 for TimerEntry<H, T> where T: HasRoom4 {}

trait HasRoom4 {}
impl HasRoom4 for End {}

pub trait TimerCount {
    const COUNT: usize;
}
impl TimerCount for End {
    const COUNT: usize = 0;
}
impl<H, T: TimerCount> TimerCount for TimerEntry<H, T> {
    const COUNT: usize = 1 + T::COUNT;
}

pub struct MotorSpawner<'a, Motors, Timers> {
    ledc: &'a Ledc<'a>,
    motors: Motors,
    timers: Timers,
}

pub struct PwmMotor<'a> {
    sender: Sender<'a, CriticalSectionRawMutex, ServoCmd, 4>,
}

impl<'a> PwmMotor<'a> {
    pub fn new(spawner: Spawner, pwm: Channel<'static, LowSpeed>, initial_angle: i32) -> Self {
        let channel = make_static!(embassy_sync::channel::Channel::new());
        spawner
            .spawn(servo_motor_loop(
                initial_angle,
                Speed::Fast,
                channel.receiver(),
                pwm,
            ))
            .ok();

        Self {
            sender: channel.sender(),
        }
    }
    pub async fn send_cmd(&self, cmd: ServoCmd) {
        self.sender.send(cmd);
    }
}

#[task]
pub async fn servo_motor_loop(
    mut initial_angle: i32,
    mut speed: Speed,
    cmd: Receiver<'static, CriticalSectionRawMutex, ServoCmd, 4>,
    pwm: Channel<'static, LowSpeed>,
) {
    let mut target_angle = initial_angle;

    info!("Moving motor to initial pos.");

    let set_angle = |angle: i32| {
        pwm.set_duty_hw(
            duty_from_angle(angle.clamp(0, 180) as u32, pwm.max_duty_cycle().into()).into(),
        );
    };

    let _guard = MOTOR_MOVING.lock().await;
    set_angle(initial_angle);
    embassy_time::Timer::after_millis(500).await;
    drop(_guard);

    info!("ending moving motor to initial pos");

    loop {
        match cmd.receive().await {
            ServoCmd::TurnToAngle(new_angle) => target_angle = new_angle,
            ServoCmd::SetSpeed(new_speed) => speed = new_speed,
        }
        let diff = target_angle - initial_angle;

        if diff == 0 {
            continue;
        }

        info!("Moving motor");

        let step = diff.signum();

        for angle in range_step(initial_angle, target_angle, step) {
            set_angle(angle);

            match speed {
                Speed::Slow => embassy_time::Timer::after_millis(15).await,
                Speed::Normal => embassy_time::Timer::after_millis(10).await,
                Speed::Fast => embassy_time::Timer::after_millis(5).await,
                Speed::Instant => {}
                Speed::Custom(time) => embassy_time::Timer::after(time).await,
            }
        }

        initial_angle = target_angle;

        info!("finished moving");
    }
}

impl<'a> MotorSpawner<'a, [DCMotor<'a>; 0], End> {
    pub fn new_dc(ledc: &'a Ledc<'a>) -> Self {
        Self {
            ledc,
            motors: [],
            timers: End,
        }
    }
}

impl<'a> MotorSpawner<'a, [PwmMotor<'a>; 0], End> {
    pub fn new_pwm(ledc: &'a Ledc<'a>) -> Self {
        Self {
            ledc,
            motors: [],
            timers: End,
        }
    }
}

macro_rules! impl_spawner_inner {
    (
        $Motor:ident,
        $from:literal, $to:literal, $channel:ident, [$($m:ident),*]
        $(; dir $DirA:ident, $pina:ident, $DirB:ident, $pinb:ident)?
        $(; pre_args  {$($pre_param:ident  : $pre_ty:ty),+})?
        $(; post_args {$($post_param:ident : $post_ty:ty),+})?
    ) => {
        impl<'a: 'static, Timers> MotorSpawner<'a, [$Motor<'a>; $from], Timers>
        where
            Timers: TimerCount,
        {
            pub fn spawn_reuse<PwmPin, $($DirA, $DirB,)? Config, Index>(
                self,
                $($($pre_param: $pre_ty,)+)?   // e.g. spawner: Spawner
                pwm_pin: PwmPin,
                $($pina: $DirA, $pinb: $DirB,)?
                $($($post_param: $post_ty,)+)?  // e.g. initial_angle: i32
                _config: Config,
            ) -> MotorSpawner<'a, [$Motor<'a>; $to], Timers>
            where
                PwmPin: OutputPin + 'a,
                $($DirA: OutputPin + 'a, $DirB: OutputPin + 'a,)?
                Config: TimerConfigTrait,
                Timers: FindTimer<Config, Index>,
            {
                let shared_timer = self.timers.get_timer();
                let mut pwm = self.ledc.channel(channel::Number::$channel, pwm_pin);
                pwm.configure(channel::config::Config {
                    timer: shared_timer,
                    duty_pct: 0,
                    drive_mode: DriveMode::PushPull,
                }).unwrap();
                let new_motor = $Motor::new($($($pre_param,)+)? $($pina, $pinb,)? pwm $(, $($post_param),+)?);
                let [$($m),*] = self.motors;
                MotorSpawner { ledc: self.ledc, motors: [$($m,)* new_motor], timers: self.timers }
            }

            pub fn spawn_new<PwmPin, $($DirA, $DirB,)? Config>(
                self,
                $($($pre_param: $pre_ty,)+)?
                pwm_pin: PwmPin,
                $($pina: $DirA, $pinb: $DirB,)?
                $($($post_param: $post_ty,)+)?
                _config: Config,
            ) -> MotorSpawner<'a, [$Motor<'a>; $to], <Timers as AllocTimer<Config>>::Output>
            where
                PwmPin: OutputPin + 'a,
                $($DirA: OutputPin + 'a, $DirB: OutputPin + 'a,)?
                Config: TimerConfigTrait,
                Timers: AllocTimer<Config> + HasRoom,
            {
                let slot = Timers::COUNT;
                let (shared_timer, new_timers) = self.timers.alloc(self.ledc, slot);
                let mut pwm = self.ledc.channel(channel::Number::$channel, pwm_pin);
                pwm.configure(channel::config::Config {
                    timer: shared_timer,
                    duty_pct: 0,
                    drive_mode: DriveMode::PushPull,
                }).unwrap();
                let new_motor = $Motor::new($($($pre_param,)+)? $($pina, $pinb,)? pwm $(, $($post_param),+)?);
                let [$($m),*] = self.motors;
                MotorSpawner { ledc: self.ledc, motors: [$($m,)* new_motor], timers: new_timers }
            }

            pub fn finish(self) -> [$Motor<'a>; $from] {
                self.motors
            }
        }
    }
}

// Thin wrappers — unchanged call sites
macro_rules! impl_spawner {
    ($from:literal, $to:literal, $channel:ident, [$($m:ident),*]) => {
        impl_spawner_inner!(DCMotor, $from, $to, $channel, [$($m),*]
            ; dir DirA, pina, DirB, pinb);
    }
}
macro_rules! impl_spawner_pwm {
    ($from:literal, $to:literal, $channel:ident, [$($m:ident),*]) => {
        impl_spawner_inner!(PwmMotor, $from, $to, $channel, [$($m),*]
            ; pre_args  { spawner: Spawner }
            ; post_args { initial_angle: i32 });
    }
}
impl_spawner!(0, 1, Channel0, []);
impl_spawner!(1, 2, Channel1, [m0]);
impl_spawner!(2, 3, Channel2, [m0, m1]);
impl_spawner!(3, 4, Channel3, [m0, m1, m2]);
impl_spawner!(4, 5, Channel4, [m0, m1, m2, m3]);
impl_spawner!(5, 6, Channel5, [m0, m1, m2, m3, m4]);
impl_spawner!(6, 7, Channel6, [m0, m1, m2, m3, m4, m5]);
impl_spawner!(7, 8, Channel7, [m0, m1, m2, m3, m4, m5, m6]);
impl_spawner_pwm!(0, 1, Channel0, []);
impl_spawner_pwm!(1, 2, Channel1, [m0]);
impl_spawner_pwm!(2, 3, Channel2, [m0, m1]);
impl_spawner_pwm!(3, 4, Channel3, [m0, m1, m2]);
impl_spawner_pwm!(4, 5, Channel4, [m0, m1, m2, m3]);
impl_spawner_pwm!(5, 6, Channel5, [m0, m1, m2, m3, m4]);
impl_spawner_pwm!(6, 7, Channel6, [m0, m1, m2, m3, m4, m5]);
impl_spawner_pwm!(7, 8, Channel7, [m0, m1, m2, m3, m4, m5, m6]);

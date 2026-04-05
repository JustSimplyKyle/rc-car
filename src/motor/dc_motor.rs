use core::marker::PhantomData;

use esp_hal::{
    gpio::{DriveMode, Level, Output, OutputPin},
    ledc::{
        self,
        channel::{self, Channel, ChannelIFace},
        timer::{self, LSClockSource, Timer, TimerIFace},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
    time::Rate,
};
use static_cell::StaticCell;

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
    ($name:ident, $freq:expr, $duty:expr) => {
        pub struct $name;
        impl TimerConfigTrait for $name {
            const FREQUENCY: u32 = $freq;
            const DUTY_RESOLUTION: timer::config::Duty = $duty;
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

impl<'a> MotorSpawner<'a, [DCMotor<'a>; 0], End> {
    pub fn new(ledc: &'a Ledc<'a>) -> Self {
        Self {
            ledc,
            motors: [],
            timers: End,
        }
    }
}

macro_rules! impl_spawner {
    ($from:literal, $to:literal, $channel:ident, [$($m:ident),*]) => {
        impl<'a: 'static, Timers> MotorSpawner<'a, [DCMotor<'a>; $from], Timers>
        where
            Timers: TimerCount,
        {
            // Spawn reusing an existing timer
            pub fn spawn_reuse<PwmPin, DirA, DirB, Config, Index>(
                self,
                pwm_pin: PwmPin,
                pina: DirA,
                pinb: DirB,
                _config: Config,
            ) -> MotorSpawner<'a, [DCMotor<'a>; $to], Timers>
            where
                PwmPin: OutputPin + 'a,
                DirA: OutputPin + 'a,
                DirB: OutputPin + 'a,
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
                let new_motor = DCMotor::new(pina, pinb, pwm);
                let [$($m),*] = self.motors;
                MotorSpawner {
                    ledc: self.ledc,
                    motors: [$($m,)* new_motor],
                    timers: self.timers,
                }
            }

            // Spawn allocating a new timer slot
            pub fn spawn_new<PwmPin, DirA, DirB, Config>(
                self,
                pwm_pin: PwmPin,
                pina: DirA,
                pinb: DirB,
                _config: Config,
            ) -> MotorSpawner<'a, [DCMotor<'a>; $to], <Timers as AllocTimer<Config>>::Output>
            where
                PwmPin: OutputPin + 'a,
                DirA: OutputPin + 'a,
                DirB: OutputPin + 'a,
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
                let new_motor = DCMotor::new(pina, pinb, pwm);
                let [$($m),*] = self.motors;
                MotorSpawner {
                    ledc: self.ledc,
                    motors: [$($m,)* new_motor],
                    timers: new_timers,
                }
            }

            pub fn finish(self) -> [DCMotor<'a>; $from] {
                self.motors
            }
        }
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

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(type_alias_impl_trait)]

use ble_scanner::Scanner;
use embassy_executor::Executor;
use embassy_time::{Duration, Instant};
use esp_backtrace as _;
use esp_println::println;
use hal::{
    clock::{ClockControl, CpuClock},
    embassy,
    interrupt,
    peripherals::{self, Peripherals},
    prelude::*,
    systimer::SystemTimer,
    timer::TimerGroup,
    Rng,
    Rtc,
};
mod ble_scanner;
use static_cell::StaticCell;
static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[cfg(feature = "coex")]
use esp_wifi::esp_now;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();
    // setup logger
    // To change the log_level change the env section in .config/cargo.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();

    #[allow(unused_variables)]
    let (wifi, bluetooth) = peripherals.RADIO.split();
    let timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;

    let init_mode = if cfg!(feature = "coex") {
        esp_wifi::EspWifiInitFor::WifiBle
    } else {
        esp_wifi::EspWifiInitFor::Ble
    };

    let init = esp_wifi::initialize(
        init_mode,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let mut ble_scanner = Scanner::new(&init, bluetooth);
    ble_scanner.init();

    #[cfg(feature = "coex")]
    let _ = esp_now::EspNow::new(&init, wifi).unwrap();

    // Async requires the GPIO interrupt to wake futures
    interrupt::enable(peripherals::Interrupt::GPIO, interrupt::Priority::Priority1).unwrap();

    embassy::init(&clocks, timer_group0.timer0);
    let executor = EXECUTOR.init_with(Executor::new);

    executor.run(|spawner| {
        spawner.must_spawn(ble_scan_task(ble_scanner));
    });
}

#[embassy_executor::task]
pub async fn ble_scan_task(mut ble_scanner: Scanner) {
    const SCAN_DURATION: Duration = Duration::from_secs(10);
    loop {
        let now = Instant::now();
        ble_scanner.scan_until(now + SCAN_DURATION).await;
        println!(
            "scan result pkts:{:5} size:{:5}KB",
            ble_scanner.packets,
            ble_scanner.packets_size / 1024
        );
    }
}

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;
use alloc::vec::Vec;
use core::mem::MaybeUninit;
use embassy_executor::Executor;
use embassy_time::Duration;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_println::println;
use hal::{clock::ClockControl, embassy, peripherals::Peripherals, prelude::*, Delay};

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[embassy_executor::task]
async fn run1() {
    loop {
        esp_println::println!("Hello world from embassy using esp-hal-async!");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[entry]
fn main() -> ! {
    init_heap();
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);
    esp_println::logger::init_logger_from_env();

    embassy::init(
        &clocks,
        hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    // Embassy setup
    let executor = static_cell::make_static!(Executor::new());

    executor.run(|spawner| {
        spawner.spawn(run1()).ok();
    });

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    log::info!("Logger is setup");
    println!("Hello world!");
    loop {
        println!("Loop...");
        delay.delay_ms(500u32);
    }
}

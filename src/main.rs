#![feature(impl_trait_in_assoc_type)]
#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{AnyPin, Level, Output, Speed},
    i2c::{Error, I2c},
    spi::{self, Spi},
    usart::{self, Uart},
    Peripherals,
};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

// Declare async tasks
#[embassy_executor::task]
async fn blink(
    status_pin: AnyPin,
    led0_pin: AnyPin,
    led1_pin: AnyPin,
    led2_pin: AnyPin,
    led3_pin: AnyPin,
    led4_pin: AnyPin,
) {
    info!("Started blink task");

    let mut status = Output::new(status_pin, Level::Low, Speed::Low);

    let mut leds = [
        Output::new(led0_pin, Level::Low, Speed::Low),
        Output::new(led1_pin, Level::Low, Speed::Low),
        Output::new(led2_pin, Level::Low, Speed::Low),
        Output::new(led3_pin, Level::Low, Speed::Low),
        Output::new(led4_pin, Level::Low, Speed::Low),
    ];

    let mut led_num = 0;

    loop {
        // Timekeeping is globally available, no need to mess with hardware timers.
        status.toggle();
        Timer::after_millis(150).await;

        for led in leds.iter_mut() {
            led.set_low();
        }

        leds[led_num].set_high();

        led_num += 1;
        if led_num >= leds.len() {
            led_num = 0;
        }
    }
}

// main is itself an async function.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    spawner.spawn(blink(
        p.PB13.into(),

        p.PC13.into(),
        p.PC14.into(),
        p.PC15.into(),
        p.PF0.into(),
        p.PF1.into()
    )).unwrap();
}

#![feature(impl_trait_in_assoc_type)]
#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{AnyPin, Level, Output, Speed}, i2c::{Error, I2c}, pac::SPI1, peripherals::{DMA1_CH3, DMA1_CH4}, spi::{self, BitOrder, MisoPin, MosiPin, SckPin, Spi}, time::Hertz, usart::{self, Uart}, Peripheral, Peripherals
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

struct Accelerometer<'d, T: spi::Instance, Tx, Rx> {
    spi: Spi<'d, T, Tx, Rx>,
    cs: Output<'d, AnyPin>
}

impl<'d, T: spi::Instance, Tx: spi::TxDma<T>, Rx: spi::RxDma<T>> Accelerometer<'d, T, Tx, Rx> {
    const REG_WHO_AM_I: u8 = 0x0F;
    const WHO_AM_I_VAL: u8 = 0b00110011;

    async fn init(
        peri: impl Peripheral<P = T> + 'd,
        cs: AnyPin,
        sck: impl Peripheral<P = impl SckPin<T>> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T>> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T>> + 'd,
        txdma: impl Peripheral<P = Tx> + 'd,
        rxdma: impl Peripheral<P = Rx> + 'd,
    ) -> Result<Self, Error> {
        info!("Initializing accelerometer...");
        let mut config = spi::Config::default();
        config.bit_order = BitOrder::MsbFirst;
        config.frequency = Hertz(100000);
        let spi = Spi::new(peri, sck, mosi, miso, txdma, rxdma, config);
        let cs = Output::new(cs, Level::High, Speed::High);

        let mut accel = Self { spi, cs };

        let who_am_i = accel.read(Self::REG_WHO_AM_I).await;
        defmt::assert_eq!(Self::WHO_AM_I_VAL, who_am_i);

        info!("Successfully initialized and checked accelerometer ID!");
        Ok(accel)
    }

    async fn read(&mut self, addr: u8) -> u8 {
        defmt::assert_eq!(addr & 0b11000000, 0);

        let mut data: [u8; 1] = [
            (addr) | // Address
            (0 << 6) | // Increment address bit = 0
            (1 << 7) // READ value = 1
        ];

        self.cs.set_low();

        self.spi.write(&data).await.unwrap();
        self.spi.read(&mut data).await.unwrap();

        self.cs.set_high();

        return data[0];
    }
}

// main is itself an async function.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    spawner
        .spawn(blink(
            p.PB13.into(),
            p.PC13.into(),
            p.PC14.into(),
            p.PC15.into(),
            p.PF0.into(),
            p.PF1.into(),
        ))
        .unwrap();

    Accelerometer::init(p.SPI1, p.PA15.into(), p.PA5, p.PA7, p.PA6, p.DMA1_CH3, p.DMA1_CH4).await.unwrap();
}

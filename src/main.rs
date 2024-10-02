#![feature(impl_trait_in_assoc_type)]
#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{AnyPin, Level, Output, Speed},
    i2c::Error,
    mode::Async,
    spi::{self, BitOrder, Spi},
    time::Hertz,
    Peripheral,
};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

// Declare async tasks
#[embassy_executor::task]
async fn blink(status_pin: AnyPin) {
    info!("Started blink task");

    let mut status = Output::new(status_pin, Level::Low, Speed::Low);

    loop {
        // Timekeeping is globally available, no need to mess with hardware timers.
        status.toggle();
        Timer::after_millis(150).await;
    }
}

struct Accelerometer<'d> {
    spi: Spi<'d, Async>,
    cs: Output<'d>,
}

impl<'d> Accelerometer<'d> {
    const REG_WHO_AM_I: u8 = 0x0F;
    const WHO_AM_I_VAL: u8 = 0b00110011;

    async fn init<T: spi::Instance>(
        peri: impl Peripheral<P = T> + 'd,
        cs: AnyPin,
        sck: impl Peripheral<P = impl spi::SckPin<T>> + 'd,
        mosi: impl Peripheral<P = impl spi::MosiPin<T>> + 'd,
        miso: impl Peripheral<P = impl spi::MisoPin<T>> + 'd,
        tx_dma: impl Peripheral<P = impl spi::TxDma<T>> + 'd,
        rx_dma: impl Peripheral<P = impl spi::RxDma<T>> + 'd,
    ) -> Result<Self, Error> {
        info!("Initializing accelerometer...");
        let mut config = spi::Config::default();
        config.bit_order = BitOrder::MsbFirst;
        config.frequency = Hertz(100000);
        let spi = Spi::new(peri, sck, mosi, miso, tx_dma, rx_dma, config);
        let cs = Output::new(cs, Level::High, Speed::High);

        let mut accel = Self { spi, cs };

        let mut who_am_i = [0];
        unwrap!(accel.read(Self::REG_WHO_AM_I, &mut who_am_i).await);
        defmt::assert_eq!(Self::WHO_AM_I_VAL, who_am_i[0]);

        info!("Successfully initialized and checked accelerometer ID!");
        Ok(accel)
    }

    async fn read(&mut self, addr: u8, data: &mut [u8]) -> Result<(), spi::Error> {
        defmt::assert_eq!(addr & 0b11000000, 0);

        let addr_data: [u8; 1] = [
            (addr) | // Address
            (1 << 6) | // Increment address bit = 1
            (1 << 7), // READ value = 1
        ];

        self.cs.set_low();

        self.spi.write(&addr_data).await?;
        self.spi.read(data).await?;

        self.cs.set_high();

        Ok(())
    }
}

// main is itself an async function.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    unwrap!(spawner.spawn(blink(p.PA3.into())));

    let _accel1 = unwrap!(Accelerometer::init(
        p.SPI1,
        p.PA4.into(),
        p.PA1,
        p.PA2,
        p.PA6,
        p.DMA1_CH1,
        p.DMA1_CH2,
    ).await);
    
    info!("Accelerometer init complete!");
}

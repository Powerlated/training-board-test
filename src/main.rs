#![feature(impl_trait_in_assoc_type)]
#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{AnyPin, Level, Output, Speed},
    i2c::{Error, I2c},
    interrupt,
    mode::{Async, Blocking, Mode},
    pac::SPI1,
    peripherals::{self, DMA1_CH3, DMA1_CH4},
    spi::{self, BitOrder, MisoPin, MosiPin, SckPin, Spi},
    time::Hertz,
    usart::{self, Uart},
    Peripheral, Peripherals,
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

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    USART2 => usart::InterruptHandler<peripherals::USART2>;
});

/*
struct Barometer<'d> {
    usart: Uart<'d, Blocking>,
    cs: Output<'d>,
}

impl<'d> Barometer<'d> {
    const REG_WHO_AM_I: u8 = 0x0F;
    const WHO_AM_I_VAL: u8 = 0b10110001;

    fn init<T: usart::Instance>(
        peri: impl Peripheral<P = T> + 'd,
        cs: AnyPin,
        rx: impl Peripheral<P = impl usart::RxPin<T>> + 'd,
        tx: impl Peripheral<P = impl usart::TxPin<T>> + 'd,
        ck: impl Peripheral<P = impl usart::CkPin<T>> + 'd,
    ) -> Result<Self, Error> {
        info!("Initializing barometer...");
        let config = usart::Config::default();
        let usart = unwrap!(Uart::new_blocking_master(peri, rx, tx, ck, config));
        let cs = Output::new(cs, Level::High, Speed::High);

        let mut baro = Self { usart, cs };

        info!("Reading WHO_AM_I register...");
        let mut who_am_i = [0xFF];
        unwrap!(baro.read(Self::REG_WHO_AM_I, &mut who_am_i));
        defmt::assert_eq!(Self::WHO_AM_I_VAL, who_am_i[0]);

        info!("Successfully initialized and checked barometer ID!");
        Ok(baro)
    }

    fn read(&mut self, addr: u8, data: &mut [u8]) -> Result<(), usart::Error> {
        defmt::assert_eq!(addr & 0b10000000, 0);

        let mut addr_data: [u8; 1] = [
            (addr) | // Address
            (1 << 7), // READ value = 1
        ];
        // Reverse bits since USART is LSB first
        addr_data[0] = addr_data[0].reverse_bits();

        self.cs.set_low();

        self.usart.blocking_write(&addr_data)?;
        self.usart.blocking_read(data)?;

        self.cs.set_high();

        Ok(())
    }
}
*/

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
        // info!("Initializing accelerometer...");
        let mut config = spi::Config::default();
        config.bit_order = BitOrder::MsbFirst;
        config.frequency = Hertz(100000);
        let spi = Spi::new(peri, sck, mosi, miso, tx_dma, rx_dma, config);
        let cs = Output::new(cs, Level::High, Speed::High);

        let mut accel = Self { spi, cs };

        let mut who_am_i = [0];
        unwrap!(accel.read(Self::REG_WHO_AM_I, &mut who_am_i).await);
        defmt::assert_eq!(Self::WHO_AM_I_VAL, who_am_i[0]);

        // info!("Successfully initialized and checked accelerometer ID!");
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

    unwrap!(spawner.spawn(blink(p.PB13.into())));

    let accel1 = unwrap!(Accelerometer::init(
        p.SPI1,
        p.PA15.into(),
        p.PA5,
        p.PA7,
        p.PA6,
        p.DMA1_CH3,
        p.DMA1_CH4,
    ).await);
    info!("Accel 1 init complete");
    let accel2 = unwrap!(Accelerometer::init(
        p.SPI2,
        p.PB12.into(),
        p.PA0,
        p.PA4,
        p.PB2,
        p.DMA1_CH5,
        p.DMA1_CH6,
    ).await);
    info!("Accel 2 init complete");

    // let baro1 = Barometer::init(p.USART1, p.PA8.into(), p.PA10, p.PA9, p.PA12);
    // info!("Baro 1 init complete");
    // let baro2 = Barometer::init(p.USART2, p.PD3.into(), p.PA3, p.PA2, p.PA1);
    // info!("Baro 2 init complete");
}

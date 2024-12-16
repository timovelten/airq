#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cyw43_pio::PioSpi;
use defmt::{debug, info, unwrap};
use embassy_executor::{Executor, Spawner};
use embassy_rp::{
    bind_interrupts, gpio::{Level, Output}, i2c::{self}, peripherals::{DMA_CH0, I2C1, PIO0, UART0}, pio::{self, Pio}, uart::{self, BufferedUart}
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Timer};
use panic_probe as _;
use rtt_target::rtt_init_defmt;
use scd30::Scd30;
use sps30::Sps30;
use static_cell::StaticCell;

mod bluetooth;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>;
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

type Signal<T> = embassy_sync::signal::Signal<CriticalSectionRawMutex, T>;

#[entry]
fn main() -> ! {
    // It is beneficial to setup rtt_logging before constructing the embassy executor (because, as it turns
    // out, that may panic and we will never see that panic if rtt is not setup)
    rtt_init_defmt!();

    let executor = EXECUTOR.init_with(|| Executor::new());
    executor.run(|spawner| {
        spawner.must_spawn(async_main(spawner));
    });
}

#[embassy_executor::task]
async fn async_main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    //
    // Setup CYW43
    //
    static STATE: StaticCell<cyw43::State> = StaticCell::new();

    let fw = include_bytes!("../blobs/43439A0.bin");
    let clm = include_bytes!("../blobs/43439A0_clm.bin");
    let btfw = include_bytes!("../blobs/43439A0_btfw.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(&mut pio.common, pio.sm0, pio.irq0, cs, p.PIN_24, p.PIN_29, p.DMA_CH0);

    let state = STATE.init(cyw43::State::new());
    let (_net_device, bt_device, mut control, runner) = cyw43::new_with_bluetooth(state, pwr, spi, fw, btfw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));
    control.init(clm).await;

    //
    // SPS30
    //
    static TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 16]> = StaticCell::new();

    let tx_buf = &mut TX_BUF.init([0; 16])[..];
    let rx_buf = &mut RX_BUF.init([0; 16])[..];
    let uart_sps30 = BufferedUart::new(
        p.UART0,
        Irqs,
        p.PIN_16,
        p.PIN_17,
        tx_buf,
        rx_buf,
        Default::default(),
    );

    //
    // I2C for SCD30
    //
    let mut i2c_config: i2c::Config = Default::default();
    i2c_config.frequency = 50_000;
    let i2c = i2c::I2c::new_async(
        p.I2C1,
        p.PIN_15,
        p.PIN_14,
        Irqs,
        i2c_config,
    );

    // Setup the actual application
    static PM_SIGNAL: StaticCell<Signal<sps30::Measurement>> = StaticCell::new();
    static CO2_SIGNAL: StaticCell<Signal<scd30::Measurement>> = StaticCell::new();

    let pm_signal: &'static Signal<_> = PM_SIGNAL.init(Default::default());
    let co2_signal: &'static Signal<_> = CO2_SIGNAL.init(Default::default());

    unwrap!(spawner.spawn(bluetooth::runner(bt_device, pm_signal, co2_signal)));
    unwrap!(spawner.spawn(measure_pm(uart_sps30, pm_signal)));
    unwrap!(spawner.spawn(measure_co2(i2c, co2_signal)));
}

#[embassy_executor::task]
async fn cyw43_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn measure_pm(uart: BufferedUart<'static, UART0>, signal: &'static Signal<sps30::Measurement>) {
    let mut sps30 = Sps30::new(uart, Delay);

    // Reset because resetting the pico does not necessarily reset the sensor itself
    // (and calling start measurement if it is already started will result in an error)
    unwrap!(sps30.reset().await);
    unwrap!(sps30.start_measurement().await);
    info!("Started SPS30 measurement!");

    Timer::after_millis(2000).await;

    loop {
        let measurement = sps30.read_measured_value().await;
        debug!("SPS30: {}", measurement);
        match measurement {
            Ok(m) => signal.signal(m),
            Err(err) => info!("SPS30 Error: {}", &err),
        }

        Timer::after_secs(10).await;
    }
}

#[embassy_executor::task]
async fn measure_co2(i2c: i2c::I2c<'static, I2C1, i2c::Async>, signal: &'static Signal<scd30::Measurement>) {
    let mut scd = Scd30::new(i2c, Delay);
    let version = unwrap!(scd.read_firmware_version().await);
    info!("SCD30 firmware version is {}", version);

    loop {
        let measurement = scd.read_measurement().await;
        debug!("SCD30: {}", measurement);
        match measurement {
            Ok(m) => signal.signal(m),
            Err(err) => info!("SCD30 Error: {}", &err),
        }

        Timer::after_secs(10).await;
    }
}
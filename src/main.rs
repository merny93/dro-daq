//! This example shows how to use USB (Universal Serial Bus) in the RP2040 chip.
//!
//! This creates a USB serial port that echos.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::usb::{Driver, Instance};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::driver::EndpointError;
use heapless;
use panic_halt as _;

mod indicator;
mod usb;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // info!("Hello there!");

    let p = embassy_rp::init(Default::default());
    let (usb, mut class) = usb::usb_init(p.USB);

    let (mut irq3, mut sm0) = indicator::pio_init(p.PIO0, p.PIN_2, p.PIN_3);

    let mut led = Output::new(p.PIN_13, Level::Low);
    led.set_high();

    // Run the USB device.
    spawner.spawn(usb::usb_task(usb)).unwrap();

    //wait for a client to connect
    class.wait_connection().await;

    loop {
        let res_bits = indicator::get_bits(&mut sm0, &mut irq3, &mut led).await;
        let mut formated = indicator::format_bits(res_bits);
        let _ = send_data(&mut class, &mut formated).await;
    }
}

async fn send_data<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
    data: &mut heapless::String<40>,
) -> Result<(), EndpointError> {
    //create hello world message
    // echo the message
    data.push('\n').unwrap();
    class.write_packet(data.as_bytes()).await?;
    Ok(())
}

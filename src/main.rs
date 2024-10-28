//! This example shows how to use USB (Universal Serial Bus) in the RP2040 chip.
//!
//! This creates a USB serial port that echos.

#![no_std]
#![no_main]

// use defmt::{info, panic, unwrap};
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::usb::{Driver, Instance};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::driver::EndpointError;
// use {defmt_rtt as _, panic_probe as _};
use panic_halt as _;

mod usb;
mod indicator;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // info!("Hello there!");

    let p = embassy_rp::init(Default::default());
    let (usb,mut class) = usb::usb_init(p.USB);

    let (irq3, sm2) = indicator::pio_init(p.PIO0);

    let led = Output::new(p.PIN_13, Level::Low);

    spawner.spawn(indicator::pio_task_sm2(irq3, sm2, led)).unwrap();



    // Run the USB device.
    spawner.spawn(usb::usb_task(usb)).unwrap();

    // Do stuff with the class!
    loop {
        class.wait_connection().await;
        // info!("Connected");
        let _ = send_data(&mut class).await;
        // info!("Disconnected");
    }
}




async fn send_data<'d, T: Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>) -> Result<(), EndpointError> {
    loop {
        Timer::after_secs(1).await;
        //create hello world message
        let mut hello = [0; 13];
        hello[0] = 0x48;
        hello[1] = 0x65;
        hello[2] = 0x6c;
        hello[3] = 0x6c;
        hello[4] = 0x6f;
        hello[5] = 0x20;
        hello[6] = 0x57;
        hello[7] = 0x6f;
        hello[8] = 0x72;
        hello[9] = 0x6c;
        hello[10] = 0x64;
        hello[11] = 0x21;
        hello[12] = 0x0A;
        // echo the message

        class.write_packet(&hello).await?;
    }
}
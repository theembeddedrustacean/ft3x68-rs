// Example for FT3168 touch driver on the Waveshare 1.8 inch AMOLED display

#![no_std]
#![no_main]

use embedded_hal::i2c::Error;
use ft3x68_rs::{DriverError, Ft3x68Driver, PowerMode, ResetInterface, FT3168_DEVICE_ADDRESS};

extern crate alloc;
use embedded_hal_bus::i2c;
use embedded_hal_bus::util::AtomicCell;
use esp_alloc as _;
use esp_backtrace as _;
use esp_bootloader_esp_idf::esp_app_desc;
use esp_hal::{
    delay::Delay,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    i2c::master::{Config as I2cConfig, Error as I2cError, I2c},
    main,
    spi::{
        master::{Config as SpiConfig, Spi},
        Mode,
    },
    time::Rate,
    Blocking,
};
use esp_println::println;

esp_app_desc!();

// Provide a reset implementation for the FT3x68 touch driver
// In the Waveshare 1.8" AMOLED display, the reset pin is controlled via an I2C GPIO expander (TCA9554PWR).
// The touch reset pin is  connected to Pin 2
pub struct ResetDriver<I2C> {
    i2c: I2C,
}

impl<I2C> ResetDriver<I2C> {
    pub fn new(i2c: I2C) -> Self {
        ResetDriver { i2c }
    }
}

impl<I2C> ResetInterface for ResetDriver<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    type Error = I2cError;

    fn reset(&mut self) -> Result<(), Self::Error> {
        let delay = Delay::new();
        self.i2c.write(0x20, &[0x03, 0x00]).unwrap(); // Configure all pins as output
        self.i2c.write(0x20, &[0x01, 0b0000_0000]).unwrap(); // Drive low
        delay.delay_millis(20);
        self.i2c.write(0x20, &[0x01, 0b0000_0100]).unwrap(); // Drive high
        delay.delay_millis(300);
        Ok(())
    }
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();

    // I2C Configuration for Waveshare ESP32-S3 1.8inch AMOLED Touch Display
    // Display uses an I2C IO Expander (TCA9554PWR) to control the LCD_RESET and LCD_DC lines.
    // Pinout:
    // SDA -> GPIO15
    // SCL -> GPIO14
    // Schematic:
    // https://files.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8/ESP32-S3-Touch-AMOLED-1.8.pdf
    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO15)
    .with_scl(peripherals.GPIO14);

    let i2c_cell = AtomicCell::new(i2c);

    // Initialize I2C GPIO Reset Pin for the WaveShare 1.8" AMOLED display
    let reset = ResetDriver::new(i2c::AtomicDevice::new(&i2c_cell));

    // Instantiare and Initialize Display
    println!("Initializing Touch...");

    let mut touch = Ft3x68Driver::new(
        i2c::AtomicDevice::new(&i2c_cell),
        FT3168_DEVICE_ADDRESS,
        reset,
        delay,
    );

    touch
        .initialize()
        .expect("Failed to initialize touch driver");

    // Activate Gesture Mode to detect gestures
    touch
        .set_gesture_mode(true)
        .expect("Failed to set gesture mode");

    loop {
        // Read Device ID
        touch
            .read_device_id()
            .map(|id| {
                println!("Device ID: {}", id);
            })
            .unwrap_or_else(|e| {
                println!("Error reading device ID: {:?}", e);
            });
        // Read Touch Points
        touch
            .touch1()
            .map(|touch| println!("Touch 1: {:?}", touch))
            .unwrap_or_else(|e| println!("Error reading touch 1: {:?}", e));
        // Read Detected Gesture (if any)
        touch
            .read_gesture()
            .map(|gesture| println!("Gesture: {:?}", gesture))
            .unwrap_or_else(|e| println!("Error reading gesture: {:?}", e));
        delay.delay_millis(500);
    }
}

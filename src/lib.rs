//! # FT3x68 Touch Controller Driver Crate
//!
//! A driver for the FT3x68 touch controller(s), providing functionality to read touch points, gestures, and manage power modes.
//!
//! There is limited public information relative to the FocalTech FT touch controllers. As such, much of the operational infromation is not documented in datasheets.
//! This driver is largely based on the Arduino_DriveBus repo found here: <https://github.com/Xk-w/Arduino_DriveBus/tree/master>.
//!
//!
//! This driver is `embedded-hal` compatible and provides a generic interface for the implementing the device reset functionality.
//! This is because the reset pin can be controlled via a GPIO port or an I2C I/O expander-controlled pin.
//!
//! Some drivers include an INT pin indicating touch events, but this driver does not use it. If interrupts need to be supported, the user can attach the pin output to an interrupt and implement their own callback.
//!
//! The driver currenlty supports both the FT3168 and FT3268 devices.
//!
//! ## Usage
//!
//! 1. Implement the `ResetInterface` trait for your specific reset mechanism.
//! 2. Create an instance of the `Ft3x68Driver`.
//! 3. Initialize the touch driver.
//! 4. Use the provided methods to read touch points and/or gestures.
//!
//! ```rust
//! // Initialize GPIO Reset Pin with pin or I2C instance
//! // ResetDriver would be a type that implements the `ResetInterface` trait.
//! let reset = ResetDriver::new(PinInstance);
//!
//! let mut touch = Ft3x68Driver::new(
//!        I2CInstace,
//!        FT3168_DEVICE_ADDRESS,
//!        reset,
//!        delay,
//!    );
//!
//!    touch
//!        .initialize()
//!        .expect("Failed to initialize touch driver");
//!
//!   loop {
//!        touch
//!            .touch1()
//!            .map(|touch| println!("Touch 1: {:?}", touch))
//!            .unwrap();
//!    }
//! ```
//!
//! Notes:
//! - To detect gestures, the gesture mode must be enabled using the `set_gesture_mode` method.
//! - If the reset pin is controlled via an I2C GPIO expander sharing the same bus with the touch driver, you should use the `embedded_hal_bus` to manage multiple instances of I2C. Refer to the examples folder to see how that looks like.

#![no_std]
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use heapless::Vec;

// Constants for FT3x68 device addresses and registers
pub const FT3168_DEVICE_ADDRESS: u8 = 0x38;
pub const FT3268_DEVICE_ADDRESS: u8 = 0x38;

const FT3X68_RD_DEVICE_GESTUREID: u8 = 0xD3;
const FT3X68_RD_DEVICE_FINGERNUM: u8 = 0x02;
const FT3X68_RD_DEVICE_X1POSH: u8 = 0x03;
const FT3X68_RD_DEVICE_X1POSL: u8 = 0x04;
const FT3X68_RD_DEVICE_Y1POSH: u8 = 0x05;
const FT3X68_RD_DEVICE_Y1POSL: u8 = 0x06;
const FT3X68_RD_DEVICE_X2POSH: u8 = 0x09;
const FT3X68_RD_DEVICE_X2POSL: u8 = 0x0A;
const FT3X68_RD_DEVICE_Y2POSH: u8 = 0x0B;
const FT3X68_RD_DEVICE_Y2POSL: u8 = 0x0C;
const FT3X68_RD_WR_DEVICE_GESTUREID_MODE: u8 = 0xD0;
const FT3X68_RD_WR_DEVICE_POWER_MODE: u8 = 0xA5;
const FT3X68_RD_WR_DEVICE_PROXIMITY_SENSING_MODE: u8 = 0xB0;
const FT3X68_RD_DEVICE_ID: u8 = 0xA0;

/// Power modes for the touch device.
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum PowerMode {
    Active = 0x00,
    Monitor = 0x01,
    Standby = 0x02,
    Hibernate = 0x03,
}

/// Gesture IDs returned by the device.
#[derive(Debug, PartialEq, Eq)]
pub enum Gesture {
    None,
    SwipeLeft,
    SwipeRight,
    SwipeUp,
    SwipeDown,
    DoubleClick,
    Unknown(u8),
}

/// Represents a single touch point.
#[derive(Debug, Default)]
pub struct TouchPoint {
    pub x: u16,
    pub y: u16,
}

/// Represents the touch state - either active with coordinates or released
#[derive(Debug)]
pub enum TouchState {
    /// Touch is active with coordinates
    Pressed(TouchPoint),
    /// Touch was released (no active touches)
    Released,
}


/// Trait for controlling the FT3x658 hardware reset pin.
pub trait ResetInterface {
    /// The specific error type for this reset implementation.
    type Error;

    /// Performs the hardware reset sequence for the FT3x68 device.
    /// This could be a GPIO port or an I2C expander-controlled pin.
    /// Recommended Implmentation --> HIGH delay 1ms --> LOW delay 20ms --> HIGH delay 50ms
    fn reset(&mut self) -> Result<(), Self::Error>;
}

/// Driver Errors
#[derive(Debug)]
pub enum DriverError<ResetError, I2cError> {
    /// Error originating from the display interface (QSPI/SPI/I2C).
    I2cError(I2cError),
    /// Error originating from the reset pin control.
    ResetError(ResetError),
}

/// FT3x68 touch driver
pub struct Ft3x68Driver<I2C, D, RST> {
    i2c: I2C,
    device_address: u8,
    delay: D,
    reset: RST,
}

impl<I2C, D, RST> Ft3x68Driver<I2C, D, RST>
where
    I2C: I2c,
    D: DelayNs,
    RST: ResetInterface,
{
    /// Creates a new instance of the FT3x68 driver.
    pub fn new(i2c: I2C, device_address: u8, reset: RST, delay: D) -> Self {
        Ft3x68Driver {
            i2c,
            device_address,
            delay,
            reset,
        }
    }

    /// Initializes the device defaulting to the Active power mode.
    /// This method performs a hardware reset and sets the device to the specified power mode.
    /// The hardware reset is perfromed by the `reset` method of the `ResetInterface` trait implementation.
    pub fn initialize(&mut self) -> Result<(), DriverError<RST::Error, I2C::Error>> {
        // Perform hardware reset
        self.reset.reset().map_err(DriverError::ResetError)?;
        self.delay.delay_ms(50);
        // Set the device to active power mode
        self.i2c
            .write(
                self.device_address,
                &[FT3X68_RD_WR_DEVICE_POWER_MODE, PowerMode::Active as u8],
            )
            .map_err(DriverError::I2cError)?;
        self.delay.delay_ms(20);
        Ok(())
    }

    /// Initializes the device with a specific power mode.
    /// This method performs a hardware reset and sets the device to the specified power mode.
    /// The hardware reset is perfromed by the `reset` method of the `ResetInterface` trait implementation.
    pub fn initialize_with_mode(
        &mut self,
        mode: PowerMode,
    ) -> Result<(), DriverError<RST::Error, I2C::Error>> {
        // Perform hardware reset
        self.reset.reset().map_err(DriverError::ResetError)?;
        self.delay.delay_ms(50);
        // Set the device to active power mode
        self.i2c
            .write(
                self.device_address,
                &[FT3X68_RD_WR_DEVICE_POWER_MODE, mode as u8],
            )
            .map_err(DriverError::I2cError)?;
        self.delay.delay_ms(20);
        Ok(())
    }

    /// Reads the device ID.
    /// 0x00:FT6456 0x04:FT3268 0x01:FT3067 0x05:FT3368 0x02:FT3068 0x03:FT3168
    pub fn read_device_id(&mut self) -> Result<i32, I2C::Error> {
        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(self.device_address, &[FT3X68_RD_DEVICE_ID], &mut buffer)?;
        Ok(buffer[0] as i32)
    }

    /// Sets the power mode of the device.
    pub fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), I2C::Error> {
        self.i2c.write(
            self.device_address,
            &[FT3X68_RD_WR_DEVICE_POWER_MODE, mode as u8],
        )
    }

    /// Enables or disables the proximity sensing mode.
    pub fn set_proximity_sensing_mode(&mut self, enable: bool) -> Result<(), I2C::Error> {
        self.i2c.write(
            self.device_address,
            &[FT3X68_RD_WR_DEVICE_PROXIMITY_SENSING_MODE, enable as u8],
        )
    }

    /// Enables or disables the gesture recognition mode.
    pub fn set_gesture_mode(&mut self, enable: bool) -> Result<(), I2C::Error> {
        self.i2c.write(
            self.device_address,
            &[FT3X68_RD_WR_DEVICE_GESTUREID_MODE, enable as u8],
        )
    }

    /// Reads the detected gesture.
    pub fn read_gesture(&mut self) -> Result<Gesture, I2C::Error> {
        let mut buffer = [0u8; 1];
        self.i2c.write_read(
            self.device_address,
            &[FT3X68_RD_DEVICE_GESTUREID],
            &mut buffer,
        )?;
        Ok(match buffer[0] {
            0x00 => Gesture::None,
            0x20 => Gesture::SwipeLeft,
            0x21 => Gesture::SwipeRight,
            0x22 => Gesture::SwipeUp,
            0x23 => Gesture::SwipeDown,
            0x24 => Gesture::DoubleClick,
            other => Gesture::Unknown(other),
        })
    }

    /// Reads the number of active touch points.
    pub fn finger_number(&mut self) -> Result<u8, I2C::Error> {
        let mut buffer = [0u8; 1];
        self.i2c.write_read(
            self.device_address,
            &[FT3X68_RD_DEVICE_FINGERNUM],
            &mut buffer,
        )?;
        Ok(buffer[0])
    }

    /// Reads the state of the first touch point or detects a release.
    pub fn touch1(&mut self) -> Result<TouchState, I2C::Error> {
        let fingers = self.finger_number()?;
        if fingers == 0 {
            return Ok(TouchState::Released);
        }

        let mut data = [0u8; 4];
        self.i2c
            .write_read(self.device_address, &[FT3X68_RD_DEVICE_X1POSH], &mut data)?;

        let x = ((data[0] as u16 & 0x0F) << 8) | data[1] as u16;
        let y = ((data[2] as u16 & 0x0F) << 8) | data[3] as u16;

        Ok(TouchState::Pressed(TouchPoint { x, y }))
    }

    /// Reads the state of the second touch point or detects if no second touch exists.
    pub fn touch2(&mut self) -> Result<TouchState, I2C::Error> {
        let fingers = self.finger_number()?;
        if fingers < 2 {
            return Ok(TouchState::Released);
        }

        let mut data = [0u8; 4];
        self.i2c
            .write_read(self.device_address, &[FT3X68_RD_DEVICE_X2POSH], &mut data)?;

        let x = ((data[0] as u16 & 0x0F) << 8) | data[1] as u16;
        let y = ((data[2] as u16 & 0x0F) << 8) | data[3] as u16;

        Ok(TouchState::Pressed(TouchPoint { x, y }))
    }

    /// Returns all active touch points up to the maximum supported (typically 2).
    pub fn get_touches(&mut self) -> Result<Vec<TouchPoint, 2>, I2C::Error> {
        let mut touches = Vec::new();
        let fingers = self.finger_number()?;

        if fingers >= 1 {
            if let TouchState::Pressed(point) = self.touch1()? {
                touches.push(point).ok(); // Ignore error if Vec is full
            }
        }

        if fingers >= 2 {
            if let TouchState::Pressed(point) = self.touch2()? {
                touches.push(point).ok(); // Ignore error if Vec is full
            }
        }

        Ok(touches)
    }
}

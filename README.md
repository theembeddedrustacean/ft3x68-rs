# FT3x68 Touch Controller Driver Crate

A driver for the FT3x68 touch controller(s), providing functionality to read touch points, gestures, and manage power modes.

> **Note:** There is limited public information available for FocalTech FT touch controllers. Much of the operational detail is not documented in datasheets.  
> This driver is largely based on the [Arduino_DriveBus repository](https://github.com/Xk-w/Arduino_DriveBus/tree/master).

This driver is **`embedded-hal`** compatible and provides a generic interface for implementing the device reset functionality.  
The reset pin can be controlled by custom interface such as a GPIO port or an I2C I/O expander.

Some drivers include an `INT` pin to indicate touch events, but this driver **does not** use it.  
If interrupts are needed, users can connect the pin to an interrupt-capable input and implement their own callback.

Currently, this driver supports both the **FT3168** and **FT3268** devices.


## Usage

1. Implement the `ResetInterface` trait for your specific reset mechanism.
2. Create an instance of `Ft3x68Driver`.
3. Initialize the touch driver.
4. Use the provided methods to read touch points and/or gestures.

```rust
// Initialize GPIO Reset Pin or I2C-controlled Reset Pin
// ResetDriver is a type that implements the `ResetInterface` trait.
let reset = ResetDriver::new(PinInstance);

let mut touch = Ft3x68Driver::new(
    I2CInstance,
    FT3168_DEVICE_ADDRESS,
    reset,
    delay,
);

touch
    .initialize()
    .expect("Failed to initialize touch driver");

loop {
    touch
        .touch1()
        .map(|touch| println!("Touch 1: {:?}", touch))
        .unwrap();
}
```

## Running the Example

To run the example demonstrating touch support:

```bash
cargo run --release --example touch --all-features
```

> **Notes:**
> - To detect gestures, the gesture mode must be first enabled using the `set_gesture_mode` method.
> - If the reset pin is controlled via an I2C GPIO expander sharing the same bus with the touch driver, you should use the `embedded_hal_bus` to manage multiple instances of I2C. Refer to the examples folder to see how that looks like.

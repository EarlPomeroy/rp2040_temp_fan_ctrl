//! # Control fans for a Raspberry Pi cluster
//!
//! This application uses a Raspberry pico to measure temperature and a relay to control fans
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.

// Code based on:
// https://how2electronics.com/read-temperature-sensor-value-from-raspberry-pi-pico/
// https://docs.sunfounder.com/projects/thales-kit/en/latest/thermometer.html
//

#![no_std]
#![no_main]

mod write_to;
use crate::write_to::write_to::show;

// Ensure we halt the program on panic
use panic_halt as _;

// Some traits we need
use embedded_hal::adc::OneShot;
use embedded_hal::digital::v2::OutputPin;
use fugit::RateExtU32;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::{pac, Clock};

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM boot loader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spin lock are initialized.
///
/// The function configures the RP2040 peripherals, then performs a single I²C
/// write to a fixed address.
#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio4.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio5.into_mode::<hal::gpio::FunctionI2C>();

    let mut fan_ctrl_pin = pins.gpio6.into_push_pull_output();

    // Enable ADC
    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);

    // Enable the temperature sense channel
    let mut temperature_sensor = pins.gpio28.into_floating_input();

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let mut i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut lcd = lcd_lcm1602_i2c::Lcd::new(&mut i2c, &mut delay)
        .address(0x27)
        .cursor_on(false) // no visible cursors
        .rows(2) // two rows
        .init()
        .unwrap();

    // The 12-bit ADC pin reading is between 0 and 4096 for thermistor
    let conversion_factor = 3.3 / 4096.0;
    let resistor_val: f64 = 10000.0;
    fan_ctrl_pin.set_low().unwrap();
    lcd = lcd.init().unwrap();
    lcd.clear().unwrap();

    loop {
        // read 12-bit value and convert to some percentage of the 3.3v
        let adc_temp_value: u16 = adc.read(&mut temperature_sensor).unwrap();
        let voltage: f64 = adc_temp_value as f64 * conversion_factor;

        let resistance = resistor_val * voltage / (3.3 - voltage);
        // get temp kelvin
        let temp_k = 1.0
            / (((libm::log(resistance as f64 / resistor_val)) / 3950.0) + (1.0 / (273.15 + 25.0)));

        // convert to Celsius
        let temp = temp_k - 273.15;

        if temp > 30.0 {
            fan_ctrl_pin.set_high().unwrap();
        } else {
            fan_ctrl_pin.set_low().unwrap();
        }

        let mut buf = [0u8; 64];

        lcd.set_cursor(0, 0).unwrap();
        let output: &str = show(&mut buf, format_args!("Temp: {}", temp)).unwrap();
        lcd.write_str(output).unwrap();

        lcd.set_cursor(1, 0).unwrap();
        let output: &str = show(&mut buf, format_args!("Pin val: {}", adc_temp_value)).unwrap();
        lcd.write_str(output).unwrap();
    }
}

// End of file

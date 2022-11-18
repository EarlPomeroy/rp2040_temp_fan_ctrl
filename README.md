# rp2040_temp_fan_ctrl

Fan Controller based around an RP2040 microcontroller

The LCD uses GPIO pins 4 and 5 for i2c output. The LED output is removed in the latest code and is only for debugging and testing. If you need the LCD, uncomment the code and recompile.

The relay I'm using is a JZC-11F controlled with GPIO pin 6. Setting the pin high closes the relay and enables the fans. The LED in the schematic is to dissipate the current when the coil is de-energized.

The fan’s power goes through the normally open pin to common on the relay. Energizing the coil closes the relay and powers the fans.

I’ve set the fan temperature to 50 ℃ as the idle cluster runs about 45 ℃.

# To Build project

```bash
$ cargo build --release
```

# To Flash the RP2040

Hold **BOOTSEL** button and connect to USB

Once mounted, run:

```bash
$ cargo run --release
```

_Note_: This requires the elf2uf2-rs cargo plugin. Install this using:

```bash
$ cargo install elf2uf2-rs
```

See [JoNil/elf2uf2-rs](https://github.com/JoNil/elf2uf2-rs) for code and details.

# st7735-lcd-doublebuffering

This is a library for LCD TFT Displays ST7735 with double buffering, uses ``alloc`` for buffers allocations. Designed for powerful MCU's like ESP32
This library only works with [embedded_graphics](https://github.com/embedded-graphics/embedded-graphics).

# Features
* Double Buffering: Smooth updates without screen flicker, ensuring high-quality display performance.
* No Low-Level Control: Focused on high-level buffered drawing, without low-level pixel manipulation or display control (no plans for low-level support).

# Example

Draws a circle that flies everywhere, bouncing off the edges of the screen. Using ST7735 1.8" TFT Module and ESP32-C3 board (CORE-ESP32 LUATOS)

Wiring:

``` 
VCC -> Any 3.3V pin
GND -> Any GND pin
CS -> Any GND pin (unused in this exmaple)
RESET -> IO06
A0 -> IO07
SDA -> IO03
SCL -> IO02
LED -> Any 3.3V pin
```

```rust
use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, PrimitiveStyle},
};
use esp_idf_svc::hal::{delay::FreeRtos, gpio::PinDriver, spi};
use esp_idf_svc::hal::{gpio::Gpio0, prelude::*};
use st7735_lcd_doublebuffering::{Orientation, ST7735Buffered};

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let spi = peripherals.spi2;
    let sclk = peripherals.pins.gpio2;
    let sdo = peripherals.pins.gpio3;
    let sdi = Option::<Gpio0>::None;
    let cs = Option::<Gpio0>::None;
    let driver_config = Default::default();
    let spi_config = spi::SpiConfig::new().baudrate(60.MHz().into());
    let spi =
        spi::SpiDeviceDriver::new_single(spi, sclk, sdo, sdi, cs, &driver_config, &spi_config)
            .unwrap();

    let rst = PinDriver::output(peripherals.pins.gpio6).unwrap();
    let dc = PinDriver::output(peripherals.pins.gpio7).unwrap();

    let rgb = true;
    let width = 160;
    let height = 128;

    let mut delay = FreeRtos;
    let mut display = ST7735Buffered::new(spi, dc, Some(rst), rgb, width, height);

    display
        .init(&mut delay, &Orientation::LandscapeSwapped)
        .unwrap();
    display.set_offset(0, 0);

    let style = PrimitiveStyle::with_fill(Rgb565::RED);
    let mut x = 0;
    let mut y = 0;
    let mut x_speed = 2;
    let mut y_speed = 2;
    let diameter = 20;

    loop {
        display.clear(Rgb565::BLACK).unwrap();
        Circle::new(Point::new(x, y), diameter)
            .into_styled(style)
            .draw(&mut display)
            .unwrap();
        display.swap_buffers().unwrap();
        x += x_speed;
        y += y_speed;

        if x <= 0 || x + diameter as i32 >= width as i32 {
            x_speed = -x_speed;
        }

        if y <= 0 || y + diameter as i32 >= height as i32 {
            y_speed = -y_speed;
        }
        FreeRtos::delay_ms(40); // limit update to ~20 FPS
    }
}

```
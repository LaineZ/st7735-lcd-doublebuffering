#![no_std]
//! This crate provides a Double-buffered ST7735 TFT driver. Based on [st7735-lcd-rs](https://github.com/sajattack/st7735-lcd-rs)
extern crate alloc;

pub(crate) mod instruction;
use crate::instruction::Instruction;

use alloc::vec::Vec;
use embedded_graphics_core::pixelcolor::raw::RawU16;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi;
use embedded_hal::{delay::DelayNs, spi::SpiDevice};

use embedded_graphics_core::{draw_target::DrawTarget, pixelcolor::Rgb565, prelude::*};

pub const DEFAULT_HIGH_GAMMA_CURVE: [u8; 16] = [
    0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2c, 0x29, 0x25, 0x2b, 0x39, 0x00, 0x01, 0x03, 0x10,
];

pub const DEFAULT_LOW_GAMMA_CURVE: [u8; 16] = [
    0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2c, 0x2e, 0x2e, 0x37, 0x3f, 0x00, 0x00, 0x02, 0x10,
];

/// ST7735 driver to connect to TFT displays.
pub struct ST7735Buffered<SPI, DC>
where
    SPI: spi::SpiDevice,
    DC: OutputPin,
{
    /// SPI
    spi: SPI,
    /// Data/command pin.
    dc: DC,
    /// Whether the display is RGB (true) or BGR (false)
    rgb: bool,
    /// Global image offset
    dx: u16,
    dy: u16,
    width: u32,
    height: u32,
    back_buffer: Vec<Rgb565>,
}

/// Display orientation.
#[derive(Clone, Copy)]
pub enum Orientation {
    Portrait = 0x00,
    Landscape = 0x60,
    PortraitSwapped = 0xC0,
    LandscapeSwapped = 0xA0,
}

impl<SPI, DC> ST7735Buffered<SPI, DC>
where
    SPI: spi::SpiDevice,
    DC: OutputPin,
{
    /// Creates a new driver instance that uses hardware SPI.
    /// # Examples
    /// ```ignore
    /// // Initialization of 1.8" TFT module with 160×128 resoulution in landscape orientation
    /// // ... platform-dependent SPI initialization and GPIO
    /// let mut display = st7735_lcd::ST7735::new(spi, dc, true, 160, 128);
    /// display.init(&mut delay).unwrap();
    /// display.set_orientation(&Orientation::LandscapeSwapped).unwrap();
    /// display.set_offset(0, 0);
    /// display.clear(Rgb565::BLACK).unwrap();
    /// ```
    pub fn new(spi: SPI, dc: DC, rgb: bool, width: u32, height: u32) -> Self {
        let display = ST7735Buffered {
            spi,
            dc,
            rgb,
            dx: 0,
            dy: 0,
            width,
            height,
            back_buffer: alloc::vec![
                Rgb565::BLACK;
                width as usize * height as usize
            ],
        };

        display
    }

    /// Runs commands to initialize the display.
    pub fn init<DELAY>(&mut self, delay: &mut DELAY, orientation: &Orientation) -> Result<(), ()>
    where
        DELAY: DelayNs,
    {
        self.write_command(Instruction::SWRESET, &[])?;
        delay.delay_ms(200);
        self.write_command(Instruction::SLPOUT, &[])?;
        delay.delay_ms(200);
        self.write_command(Instruction::FRMCTR1, &[0x01, 0x2C, 0x2D])?;
        self.write_command(Instruction::FRMCTR2, &[0x01, 0x2C, 0x2D])?;
        self.write_command(Instruction::FRMCTR3, &[0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D])?;
        self.write_command(Instruction::INVCTR, &[0x07])?;
        self.write_command(Instruction::PWCTR1, &[0xA2, 0x02, 0x84])?;
        self.write_command(Instruction::PWCTR2, &[0xC5])?;
        self.write_command(Instruction::PWCTR3, &[0x0A, 0x00])?;
        self.write_command(Instruction::PWCTR4, &[0x8A, 0x2A])?;
        self.write_command(Instruction::PWCTR5, &[0x8A, 0xEE])?;
        self.write_command(Instruction::VMCTR1, &[0x0E])?;
        self.write_command(Instruction::GMCTRP1, &DEFAULT_HIGH_GAMMA_CURVE)?;
        self.write_command(Instruction::GMCTRN1, &DEFAULT_LOW_GAMMA_CURVE)?;
        self.write_command(Instruction::INVOFF, &[])?;
        if self.rgb {
            self.write_command(Instruction::MADCTL, &[0x00])?;
        } else {
            self.write_command(Instruction::MADCTL, &[0x08])?;
        }
        self.write_command(Instruction::COLMOD, &[0x05])?;
        self.set_orientation(orientation)?;
        self.clear()?; // prevent noise from render
        self.write_command(Instruction::DISPON, &[])?;
        delay.delay_ms(200);
        Ok(())
    }

    fn clear(&mut self) -> Result<(), ()> {
        self.set_pixels_buffered(
            0,
            0,
            self.width as u16 - 1,
            self.height as u16 - 1,
            core::iter::repeat(RawU16::from(Rgb565::BLACK).into_inner())
                .take((self.width * self.height) as usize),
        )
    }

    /// Hardware reset via GPIO, use this if RST connected via generic GPIO pin. Call BEFORE init
    pub fn hard_reset<DELAY, RST>(&mut self, delay: &mut DELAY, rst: &mut RST) -> Result<(), ()>
    where
        DELAY: DelayNs,
        RST: OutputPin,
    {
        rst.set_high().map_err(|_| ())?;
        delay.delay_ms(10);
        rst.set_low().map_err(|_| ())?;
        delay.delay_ms(10);
        rst.set_high().map_err(|_| ())?;
        Ok(())
    }

    fn write_command(&mut self, command: Instruction, params: &[u8]) -> Result<(), ()> {
        self.dc.set_low().map_err(|_| ())?;
        self.spi.write(&[command as u8]).map_err(|_| ())?;
        if !params.is_empty() {
            self.start_data()?;
            self.write_data(params)?;
        }
        Ok(())
    }

    fn start_data(&mut self) -> Result<(), ()> {
        self.dc.set_high().map_err(|_| ())
    }

    fn write_data(&mut self, data: &[u8]) -> Result<(), ()> {
        self.spi.write(data).map_err(|_| ())
    }

    /// Writes a data word to the display.
    fn write_word(&mut self, value: u16) -> Result<(), ()> {
        self.write_data(&value.to_be_bytes())
    }

    fn write_words_buffered(&mut self, words: impl IntoIterator<Item = u16>) -> Result<(), ()> {
        let mut buffer = [0; 32];
        let mut index = 0;
        for word in words {
            let as_bytes = word.to_be_bytes();
            buffer[index] = as_bytes[0];
            buffer[index + 1] = as_bytes[1];
            index += 2;
            if index >= buffer.len() {
                self.write_data(&buffer)?;
                index = 0;
            }
        }
        self.write_data(&buffer[0..index])
    }

    /// Whether the colours are inverted (true) or not (false)
    pub fn set_inversion(&mut self, inverted: bool) -> Result<(), ()> {
        if inverted {
            self.write_command(Instruction::INVON, &[])?;
        } else {
            self.write_command(Instruction::INVOFF, &[])?;
        }

        Ok(())
    }

    /// Sets the display orientation
    pub fn set_orientation(&mut self, orientation: &Orientation) -> Result<(), ()> {
        if self.rgb {
            self.write_command(Instruction::MADCTL, &[*orientation as u8])?;
        } else {
            self.write_command(Instruction::MADCTL, &[*orientation as u8 | 0x08])?;
        }
        Ok(())
    }

    /// Sets the global offset of the displayed image
    pub fn set_offset(&mut self, dx: u16, dy: u16) {
        self.dx = dx;
        self.dy = dy;
    }

    /// Sets the address window for the display.
    fn set_address_window(&mut self, sx: u16, sy: u16, ex: u16, ey: u16) -> Result<(), ()> {
        self.write_command(Instruction::CASET, &[])?;
        self.start_data()?;
        self.write_word(sx + self.dx)?;
        self.write_word(ex + self.dx)?;
        self.write_command(Instruction::RASET, &[])?;
        self.start_data()?;
        self.write_word(sy + self.dy)?;
        self.write_word(ey + self.dy)
    }

    fn write_pixels_buffered<P: IntoIterator<Item = u16>>(&mut self, colors: P) -> Result<(), ()> {
        self.write_command(Instruction::RAMWR, &[])?;
        self.start_data()?;
        self.write_words_buffered(colors)
    }

    fn set_pixels_buffered<P: IntoIterator<Item = u16>>(
        &mut self,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
        colors: P,
    ) -> Result<(), ()> {
        self.set_address_window(sx, sy, ex, ey)?;
        self.write_pixels_buffered(colors)
    }

    /// Swaps the front and back buffers and updates the display.
    /// # Performance
    /// - Reduces flickering by ensuring that partial frame updates are not displayed.
    /// - Displays by itself limited by internal RAM-buffer speed, so you never exceed ~20 fps, even your beautiful UI framework draws in 1000+ fps
    /// # Examples
    /// ```ignore
    /// // .. initialization of SPI, GPIO, display ..
    /// let mut display = ST7735Buffered::new(spi, dc, rgb, inverted, width, height);
    /// display.init(&mut delay).unwrap();
    /// display.set_orientation(&Orientation::Landscape).unwrap();
    /// display.set_offset(0, 0);
    /// let style = PrimitiveStyle::with_fill(Rgb565::RED);
    /// let mut x = 0;
    /// loop {
    ///      Rectangle::new(Point::new(x, 20), Size::new(20, 20))
    ///         .into_styled(style)
    ///         .draw(&mut buffer);
    ///
    ///     buffer.swap_buffers();
    ///     x = (x + 1) % 160; // Moving square by X axis and looping over...
    ///     // Delay, for this example we assume running something like ESP32 with FreeRtos, this depends on your HAL
    ///     FreeRtos::delay_ms(50); // Limiting refresh rate ~20 fps
    /// }
    /// ```
    pub fn swap_buffers(&mut self) -> Result<(), ()> {
        let size = self.size();
        let pixels = core::mem::take(&mut self.back_buffer)
            .into_iter()
            .map(|f| RawU16::from(f).into_inner());

        self.set_pixels_buffered(0, 0, size.width as u16 - 1, size.height as u16 - 1, pixels)
    }
}

impl<SPI: SpiDevice, DC: OutputPin> DrawTarget for ST7735Buffered<SPI, DC> {
    type Color = Rgb565;

    type Error = ();

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let size = self.size();
        if self.back_buffer.is_empty() {
            return Err(());
        }

        for pixel in pixels {
            let pos = pixel.0;
            let color = pixel.1;

            if pos.x >= 0 && pos.y >= 0 && pos.x < size.width as i32 && pos.y < size.height as i32 {
                let index = pos.y as usize * size.width as usize + pos.x as usize;
                self.back_buffer[index] = color;
            }
        }

        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let display_size = self.size();
        self.back_buffer =
            alloc::vec![color; display_size.width as usize * display_size.height as usize];
        Ok(())
    }
}

impl<SPI, DC> OriginDimensions for ST7735Buffered<SPI, DC>
where
    SPI: spi::SpiDevice,
    DC: OutputPin,
{
    fn size(&self) -> Size {
        Size::new(self.width, self.height)
    }
}

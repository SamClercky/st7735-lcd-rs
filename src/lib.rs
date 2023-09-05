#![no_std]

//! This crate provides a ST7735 driver to connect to TFT displays.

pub mod instruction;

use crate::instruction::Instruction;

use embassy_nrf::{
    gpio::{AnyPin, Level, Output, OutputDrive},
    spim::{self, Instance},
};
use embassy_time::{Duration, Timer};
use embedded_hal::blocking::delay::DelayMs;

/// ST7735 driver to connect to TFT displays.
pub struct ST7735<'d, SPIMINST>
where
    SPIMINST: Instance,
{
    /// SPI
    spi: spim::Spim<'d, SPIMINST>,

    /// Data/command pin.
    dc: Output<'d, AnyPin>,

    /// Reset pin.
    rst: Output<'d, AnyPin>,

    /// Whether the display is RGB (true) or BGR (false)
    rgb: bool,

    /// Whether the colours are inverted (true) or not (false)
    inverted: bool,

    /// Global image offset
    dx: u16,
    dy: u16,
    width: u32,
    height: u32,
}

/// Display orientation.
#[derive(Clone, Copy)]
pub enum Orientation {
    Portrait = 0x00,
    Landscape = 0x60,
    PortraitSwapped = 0xC0,
    LandscapeSwapped = 0xA0,
}

impl<'d, SPI> ST7735<'d, SPI>
where
    SPI: Instance,
{
    /// Creates a new driver instance that uses hardware SPI.
    pub fn new(
        spi: spim::Spim<'d, SPI>,
        dc: AnyPin,
        rst: AnyPin,
        rgb: bool,
        inverted: bool,
        width: u32,
        height: u32,
    ) -> Self {
        let display = ST7735 {
            spi,
            dc: Output::new(dc, Level::Low, OutputDrive::Standard),
            rst: Output::new(rst, Level::Low, OutputDrive::Standard),
            rgb,
            inverted,
            dx: 0,
            dy: 0,
            width,
            height,
        };

        display
    }

    /// Runs commands to initialize the display.
    pub async fn init<DELAY>(&mut self, delay: &mut DELAY) -> Result<(), ()>
    where
        DELAY: DelayMs<u8>,
    {
        self.hard_reset().await;
        self.write_command(Instruction::SWRESET, &[]).await?;
        delay.delay_ms(200);
        self.write_command(Instruction::SLPOUT, &[]).await?;
        delay.delay_ms(200);
        self.write_command(Instruction::FRMCTR1, &[0x01, 0x2C, 0x2D])
            .await?;
        self.write_command(Instruction::FRMCTR2, &[0x01, 0x2C, 0x2D])
            .await?;
        self.write_command(Instruction::FRMCTR3, &[0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D])
            .await?;
        self.write_command(Instruction::INVCTR, &[0x07]).await?;
        self.write_command(Instruction::PWCTR1, &[0xA2, 0x02, 0x84])
            .await?;
        self.write_command(Instruction::PWCTR2, &[0xC5]).await?;
        self.write_command(Instruction::PWCTR3, &[0x0A, 0x00])
            .await?;
        self.write_command(Instruction::PWCTR4, &[0x8A, 0x2A])
            .await?;
        self.write_command(Instruction::PWCTR5, &[0x8A, 0xEE])
            .await?;
        self.write_command(Instruction::VMCTR1, &[0x0E]).await?;
        if self.inverted {
            self.write_command(Instruction::INVON, &[]).await?;
        } else {
            self.write_command(Instruction::INVOFF, &[]).await?;
        }
        if self.rgb {
            self.write_command(Instruction::MADCTL, &[0x00]).await?;
        } else {
            self.write_command(Instruction::MADCTL, &[0x08]).await?;
        }
        self.write_command(Instruction::COLMOD, &[0x05]).await?;
        self.write_command(Instruction::DISPON, &[]).await?;
        delay.delay_ms(200);
        Ok(())
    }

    pub async fn hard_reset(&mut self) {
        self.rst.set_high();
        Timer::after(Duration::from_millis(10)).await;
        self.rst.set_low();
        Timer::after(Duration::from_millis(10)).await;
        self.rst.set_high();
    }

    async fn write_command(&mut self, command: Instruction, params: &[u8]) -> Result<(), ()> {
        self.dc.set_low();
        self.spi.write(&[command as u8]).await.map_err(|_| ())?;
        if !params.is_empty() {
            self.start_data();
            self.write_data(params).await?;
        }
        Ok(())
    }

    fn start_data(&mut self) {
        self.dc.set_high();
    }

    async fn write_data(&mut self, data: &[u8]) -> Result<(), ()> {
        self.spi.write(data).await.map_err(|_| ())
    }

    /// Writes a data word to the display.
    async fn write_word(&mut self, value: u16) -> Result<(), ()> {
        self.write_data(&value.to_be_bytes()).await
    }

    async fn write_words_buffered(
        &mut self,
        words: impl IntoIterator<Item = u16>,
    ) -> Result<(), ()> {
        const BUFFER_SIZE: usize = 16;
        let mut buffer = [0; BUFFER_SIZE];
        let mut index = 0;
        for word in words {
            let as_bytes = word.to_be_bytes();
            buffer[index] = as_bytes[0];
            buffer[index + 1] = as_bytes[1];
            index += 2;
            if index >= buffer.len() {
                self.write_data(&buffer).await?;
                index = 0;
            }
        }
        self.write_data(&buffer[0..index]).await
    }

    pub async fn set_orientation(&mut self, orientation: &Orientation) -> Result<(), ()> {
        if self.rgb {
            self.write_command(Instruction::MADCTL, &[*orientation as u8])
                .await?;
        } else {
            self.write_command(Instruction::MADCTL, &[*orientation as u8 | 0x08])
                .await?;
        }
        Ok(())
    }

    /// Sets the global offset of the displayed image
    pub fn set_offset(&mut self, dx: u16, dy: u16) {
        self.dx = dx;
        self.dy = dy;
    }

    /// Sets the address window for the display.
    pub async fn set_address_window(
        &mut self,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
    ) -> Result<(), ()> {
        self.write_command(Instruction::CASET, &[]).await?;
        self.start_data();
        self.write_word(sx + self.dx).await?;
        self.write_word(ex + self.dx).await?;
        self.write_command(Instruction::RASET, &[]).await?;
        self.start_data();
        self.write_word(sy + self.dy).await?;
        self.write_word(ey + self.dy).await
    }

    /// Sets a pixel color at the given coords.
    pub async fn set_pixel(&mut self, x: u16, y: u16, color: u16) -> Result<(), ()> {
        self.set_address_window(x, y, x, y).await?;
        self.write_command(Instruction::RAMWR, &[]).await?;
        self.start_data();
        self.write_word(color).await
    }

    /// Writes pixel colors sequentially into the current drawing window
    pub async fn write_pixels<P: IntoIterator<Item = u16>>(&mut self, colors: P) -> Result<(), ()> {
        self.write_command(Instruction::RAMWR, &[]).await?;
        self.start_data();
        for color in colors {
            self.write_word(color).await?;
        }
        Ok(())
    }
    pub async fn write_pixels_buffered<P: IntoIterator<Item = u16>>(
        &mut self,
        colors: P,
    ) -> Result<(), ()> {
        self.write_command(Instruction::RAMWR, &[]).await?;
        self.start_data();
        self.write_words_buffered(colors).await
    }

    /// Sets pixel colors at the given drawing window
    pub async fn set_pixels<P: IntoIterator<Item = u16>>(
        &mut self,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
        colors: P,
    ) -> Result<(), ()> {
        self.set_address_window(sx, sy, ex, ey).await?;
        self.write_pixels(colors).await
    }

    pub async fn set_pixels_buffered<P: IntoIterator<Item = u16>>(
        &mut self,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
        colors: P,
    ) -> Result<(), ()> {
        self.set_address_window(sx, sy, ex, ey).await?;
        self.write_pixels_buffered(colors).await
    }
}

#[cfg(feature = "graphics")]
extern crate embedded_graphics;
#[cfg(feature = "graphics")]
use self::embedded_graphics::prelude::*;

//#[cfg(feature = "graphics")]
//impl<SPI, DC, RST> DrawTarget for ST7735<'_, SPI, DC, RST>
//where
//    SPI: Instance,
//    DC: OutputPin,
//    RST: OutputPin,
//{
//    type Error = ();
//    type Color = Rgb565;
//
//    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
//    where
//        I: IntoIterator<Item = Pixel<Self::Color>>,
//    {
//        for Pixel(coord, color) in pixels.into_iter() {
//            // Only draw pixels that would be on screen
//            if coord.x >= 0
//                && coord.y >= 0
//                && coord.x < self.width as i32
//                && coord.y < self.height as i32
//            {
//                self.set_pixel(
//                    coord.x as u16,
//                    coord.y as u16,
//                    RawU16::from(color).into_inner(),
//                )?;
//            }
//        }
//
//        Ok(())
//    }
//
//    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
//    where
//        I: IntoIterator<Item = Self::Color>,
//    {
//        // Clamp area to drawable part of the display target
//        let drawable_area = area.intersection(&Rectangle::new(Point::zero(), self.size()));
//
//        if drawable_area.size != Size::zero() {
//            self.set_pixels_buffered(
//                drawable_area.top_left.x as u16,
//                drawable_area.top_left.y as u16,
//                (drawable_area.top_left.x + (drawable_area.size.width - 1) as i32) as u16,
//                (drawable_area.top_left.y + (drawable_area.size.height - 1) as i32) as u16,
//                area.points()
//                    .zip(colors)
//                    .filter(|(pos, _color)| drawable_area.contains(*pos))
//                    .map(|(_pos, color)| RawU16::from(color).into_inner()),
//            )?;
//        }
//
//        Ok(())
//    }
//
//    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
//        self.set_pixels_buffered(
//            0,
//            0,
//            self.width as u16 - 1,
//            self.height as u16 - 1,
//            core::iter::repeat(RawU16::from(color).into_inner())
//                .take((self.width * self.height) as usize),
//        )
//    }
//}

#[cfg(feature = "graphics")]
impl<SPI> OriginDimensions for ST7735<'_, SPI>
where
    SPI: Instance,
{
    fn size(&self) -> Size {
        Size::new(self.width, self.height)
    }
}

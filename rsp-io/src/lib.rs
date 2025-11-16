pub mod error;
pub mod image_reader;
use image::{DynamicImage, ImageBuffer, ImageError};
use std::path::Path;

pub fn load_image(path: impl AsRef<Path>) -> Result<DynamicImage, ImageError> {
    image::open(path)
}

pub fn save_image(img: &DynamicImage, path: impl AsRef<Path>) -> Result<(), ImageError> {
    img.save(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use image::{Rgb, RgbImage, RgbaImage};

    #[test]
    fn test_round_trip() {
        // Create test image
        let img = RgbImage::from_fn(100, 100, |x, y| {
            Rgb([(x % 256) as u8, (y % 256) as u8, 128])
        });

        let dynamic = DynamicImage::ImageRgb8(img.clone());

        // Save and reload
        save_image(&dynamic, "test_output.png").unwrap();
        let loaded = load_image("test_output.png").unwrap();

        // Verify dimentions match
        assert_eq!(loaded.width(), 100);
        assert_eq!(loaded.height(), 100);

        // Cleanup
        std::fs::remove_file("test_output.png").ok();
    }
}

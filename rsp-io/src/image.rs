use gdal::Dataset;
use gdal::raster::{GdalDataType, RasterBand};
use ndarray::Array3;
use std::path::Path;
use thiserror::Error;

use crate::metadata::ImageMetadata;

#[derive(Error, Debug)]
pub enum ImageError {
    #[error("GDAL error: {0}")]
    Gdal(#[from] gdal::errors::GdalError),
    #[error("Invalid image dimensions")]
    InvalidDimensions,
    #[error("Invalid band type: expected {expected}, found {found:?}")]
    InvalidBandType {
        expected: &'static str,
        found: GdalDataType,
    },
}

pub type Result<T> = std::result::Result<T, ImageError>;

/// Core image structure with metadata
pub struct Image {
    dataset: Dataset,
    width: usize,
    height: usize,
    band_count: usize,
    metadata: ImageMetadata,
}

impl Image {
    /// Open an image from file path and extract all metadata
    pub fn open<P: AsRef<Path>>(path: P) -> Result<Self> {
        let dataset = Dataset::open(path)?;
        let (width, height) = dataset.raster_size();
        let band_count = dataset.raster_count() as usize;

        // Extract all available metadata
        let metadata = ImageMetadata::from_gdal_dataset(&dataset);

        Ok(Self {
            dataset,
            width,
            height,
            band_count,
            metadata,
        })
    }

    /// Create an image wrapper from an existing GDAL dataset. Primarily used for testing
    /// with in-memory datasets.
    pub fn from_dataset(dataset: Dataset) -> Result<Self> {
        let (width, height) = dataset.raster_size();
        let band_count = dataset.raster_count() as usize;

        let metadata = ImageMetadata::from_gdal_dataset(&dataset);

        Ok(Self {
            dataset,
            width,
            height,
            band_count,
            metadata,
        })
    }

    /// Get reference to underlying GDAL dataset
    pub fn dataset(&self) -> &Dataset {
        &self.dataset
    }

    /// Get mutable reference to image metadata
    pub fn metadata_mut(&mut self) -> &mut ImageMetadata {
        &mut self.metadata
    }

    /// Get image metadata
    pub fn metadata(&self) -> &ImageMetadata {
        &self.metadata
    }

    /// Get image dimensions (width, height)
    pub fn size(&self) -> (usize, usize) {
        (self.width, self.height)
    }

    /// Get width
    pub fn width(&self) -> usize {
        self.width
    }

    /// Get height
    pub fn height(&self) -> usize {
        self.height
    }

    /// Get number of bands
    pub fn band_count(&self) -> usize {
        self.band_count
    }

    /// Read full image as u8 array (shape: [height, width, bands])
    pub fn read_u8(&self) -> Result<Array3<u8>> {
        self.read_window_u8(0, 0, self.width, self.height)
    }

    /// Read image window as u8 array
    ///
    /// # Arguments
    /// * `x_off` - Column offset (starting from 0)
    /// * `y_off` - Row offset (starting from 0)
    /// * `width` - Window width
    /// * `height` - Window height
    pub fn read_window_u8(
        &self,
        x_off: usize,
        y_off: usize,
        width: usize,
        height: usize,
    ) -> Result<Array3<u8>> {
        if x_off + width > self.width || y_off + height > self.height {
            return Err(ImageError::InvalidDimensions);
        }

        self.validate_band_types(GdalDataType::UInt8, "u8")?;

        let mut data = Array3::<u8>::zeros((height, width, self.band_count));

        for band_idx in 0..self.band_count {
            let band = self.dataset.rasterband(band_idx + 1)?;
            let buffer = band.read_as::<u8>(
                (x_off as isize, y_off as isize),
                (width, height),
                (width, height),
                None,
            )?;

            for y in 0..height {
                for x in 0..width {
                    data[[y, x, band_idx]] = buffer.data()[y * width + x];
                }
            }
        }

        Ok(data)
    }

    /// Read full image as u16 array
    pub fn read_u16(&self) -> Result<Array3<u16>> {
        self.read_window_u16(0, 0, self.width, self.height)
    }

    /// Read image window as u16 array
    pub fn read_window_u16(
        &self,
        x_off: usize,
        y_off: usize,
        width: usize,
        height: usize,
    ) -> Result<Array3<u16>> {
        if x_off + width > self.width || y_off + height > self.height {
            return Err(ImageError::InvalidDimensions);
        }

        self.validate_band_types(GdalDataType::UInt16, "u16")?;

        let mut data = Array3::<u16>::zeros((height, width, self.band_count));

        for band_idx in 0..self.band_count {
            let band = self.dataset.rasterband(band_idx + 1)?;
            let buffer = band.read_as::<u16>(
                (x_off as isize, y_off as isize),
                (width, height),
                (width, height),
                None,
            )?;

            for y in 0..height {
                for x in 0..width {
                    data[[y, x, band_idx]] = buffer.data()[y * width + x];
                }
            }
        }

        Ok(data)
    }

    /// Read full image as f32 array
    pub fn read_f32(&self) -> Result<Array3<f32>> {
        self.read_window_f32(0, 0, self.width, self.height)
    }

    /// Read image window as f32 array
    pub fn read_window_f32(
        &self,
        x_off: usize,
        y_off: usize,
        width: usize,
        height: usize,
    ) -> Result<Array3<f32>> {
        if x_off + width > self.width || y_off + height > self.height {
            return Err(ImageError::InvalidDimensions);
        }

        self.validate_band_types(GdalDataType::Float32, "f32")?;

        let mut data = Array3::<f32>::zeros((height, width, self.band_count));

        for band_idx in 0..self.band_count {
            let band = self.dataset.rasterband(band_idx + 1)?;
            let buffer = band.read_as::<f32>(
                (x_off as isize, y_off as isize),
                (width, height),
                (width, height),
                None,
            )?;

            for y in 0..height {
                for x in 0..width {
                    data[[y, x, band_idx]] = buffer.data()[y * width + x];
                }
            }
        }

        Ok(data)
    }

    /// Get geotransform if available
    pub fn geotransform(&self) -> Option<[f64; 6]> {
        self.dataset.geo_transform().ok()
    }

    /// Get projection string if available
    pub fn projection(&self) -> Option<String> {
        let projection = self.dataset.projection();
        if projection.is_empty() {
            None
        } else {
            Some(projection)
        }
    }

    fn validate_band_types(
        &self,
        expected: GdalDataType,
        expected_name: &'static str,
    ) -> Result<()> {
        for band_idx in 1..=self.band_count {
            let band = self.dataset.rasterband(band_idx)?;
            validate_band_type(&band, expected, expected_name)?;
        }

        Ok(())
    }
}

fn validate_band_type(
    band: &RasterBand,
    expected: GdalDataType,
    expected_name: &'static str,
) -> Result<()> {
    let actual = band.band_type();
    if actual != expected {
        return Err(ImageError::InvalidBandType {
            expected: expected_name,
            found: actual,
        });
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use gdal::DriverManager;

    fn mem_dataset_u8() -> Dataset {
        DriverManager::get_driver_by_name("MEM")
            .unwrap()
            .create_with_band_type::<u8, _>("", 2, 2, 1)
            .unwrap()
    }

    fn mem_dataset_f32() -> Dataset {
        DriverManager::get_driver_by_name("MEM")
            .unwrap()
            .create_with_band_type::<f32, _>("", 2, 2, 1)
            .unwrap()
    }

    #[test]
    fn read_u8_rejects_wrong_type() {
        let image = Image::from_dataset(mem_dataset_f32()).unwrap();
        let err = image.read_u8().unwrap_err();
        match err {
            ImageError::InvalidBandType { expected, found: _ } => {
                assert_eq!(expected, "u8");
            }
            _ => panic!("Unexpected error: {err}"),
        }
    }

    #[test]
    fn read_u8_succeeds_for_byte_band() {
        let image = Image::from_dataset(mem_dataset_u8()).unwrap();
        let data = image.read_u8().unwrap();
        assert_eq!(data.shape(), &[2, 2, 1]);
    }
}

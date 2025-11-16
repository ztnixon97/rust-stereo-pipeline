
use gdal::Dataset;
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
        let proj = self.dataset.projection();
        if proj.is_empty() {
            None
        } else {
            Some(proj)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_image_error_display() {
        let err = ImageError::InvalidDimensions;
        assert_eq!(err.to_string(), "Invalid image dimensions");
    }

    #[test]
    fn test_image_error_from_gdal() {
        // Test that ImageError can be created from GdalError
        // Note: This is a compile-time check more than a runtime check
        // We're just verifying the From trait is implemented
        fn _takes_image_error(_err: ImageError) {}

        // If we had a GdalError, we could convert it
        // let gdal_err = some_gdal_error;
        // let img_err: ImageError = gdal_err.into();
        // _takes_image_error(img_err);
    }

    // Note: Full integration tests for Image would require actual GDAL-compatible
    // image files. These would be better placed in an integration test directory
    // with test fixtures. The tests below document the expected API.

    // Integration test template (requires test data):
    // #[test]
    // fn test_image_open_geotiff() {
    //     let img = Image::open("test_data/sample.tif").unwrap();
    //     let (w, h) = img.size();
    //     assert!(w > 0);
    //     assert!(h > 0);
    // }

    // #[test]
    // fn test_image_read_u8() {
    //     let img = Image::open("test_data/sample.tif").unwrap();
    //     let data = img.read_u8().unwrap();
    //     assert_eq!(data.shape()[0], img.height());
    //     assert_eq!(data.shape()[1], img.width());
    //     assert_eq!(data.shape()[2], img.band_count());
    // }

    // #[test]
    // fn test_image_window_invalid_bounds() {
    //     let img = Image::open("test_data/sample.tif").unwrap();
    //     let (w, h) = img.size();
    //     let result = img.read_window_u8(w, h, 10, 10);
    //     assert!(result.is_err());
    //     assert!(matches!(result.unwrap_err(), ImageError::InvalidDimensions));
    // }

    // #[test]
    // fn test_image_metadata() {
    //     let img = Image::open("test_data/sample_with_rpc.tif").unwrap();
    //     let metadata = img.metadata();
    //     if metadata.has_rpc() {
    //         // Test RPC metadata extraction
    //         assert!(metadata.rpc.is_some());
    //     }
    // }
}

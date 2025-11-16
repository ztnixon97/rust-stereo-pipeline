use thiserror::Error;

#[derive(Error, Debug)]
pub enum IoError {
    #[error("Image read error: {0}")]
    ImageRead(String),

    #[error("EXIF parse error: {0}")]
    ExifParse(String),
}


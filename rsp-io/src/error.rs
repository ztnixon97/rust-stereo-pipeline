use thiserror::Error;

#[derive(Error, Debug)]
pub enum Ioerror {
    #[error("Image read error: {0}")]
    ImageRead(String),

    #[error("EXIF parse error: {}")]
    ExifParse(String),
}

pub type Result<T> = std::result::Result<T, IoError>;

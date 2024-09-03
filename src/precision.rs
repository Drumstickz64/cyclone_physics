#[cfg(not(feature = "double_precision"))]
pub type Real = f32;
#[cfg(feature = "double_precision")]
pub type Real = f64;

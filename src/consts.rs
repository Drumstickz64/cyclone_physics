use crate::math::vector::Vec3;

pub const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
pub const HIGH_GRAVITY: Vec3 = Vec3::new(0.0, -19.62, 0.0);

#[cfg(not(feature = "double_precision"))]
pub use std::f32::consts::*;
#[cfg(feature = "double_precision")]
pub use std::f64::consts::*;

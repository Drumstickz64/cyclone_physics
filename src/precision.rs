#[cfg(not(f64))]
pub type Real = f32;
#[cfg(f64)]
pub type Real = f64;

use std::{
    fmt,
    ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign},
};

use crate::precision::Real;

use super::matrix::Mat4;

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Default)]
pub struct Vec3 {
    pub x: Real,
    pub y: Real,
    pub z: Real,
}

impl Vec3 {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);
    pub const X: Self = Self::new(1.0, 0.0, 0.0);
    pub const Y: Self = Self::new(0.0, 1.0, 0.0);
    pub const Z: Self = Self::new(0.0, 0.0, 1.0);
    pub const NEG_X: Self = Self::new(-1.0, 0.0, 0.0);
    pub const NEG_Y: Self = Self::new(0.0, -1.0, 0.0);
    pub const NEG_Z: Self = Self::new(0.0, 0.0, -1.0);
    pub const ONE: Self = Self::new(1.0, 1.0, 1.0);
    pub const NEG_ONE: Self = Self::new(-1.0, -1.0, -1.0);

    pub const fn new(x: Real, y: Real, z: Real) -> Self {
        Self { x, y, z }
    }

    pub const fn splat(v: Real) -> Self {
        Self { x: v, y: v, z: v }
    }

    pub fn magnitude(self) -> Real {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn squared_magnitude(self) -> Real {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn normalized(self) -> Self {
        let sq_mag = self.squared_magnitude();

        if sq_mag <= 0.0 {
            return self;
        }

        self / sq_mag.sqrt()
    }

    pub fn distance_to(self, other: Self) -> Real {
        (other - self).magnitude()
    }

    pub fn distance_to_squared(self, other: Self) -> Real {
        (other - self).squared_magnitude()
    }

    pub fn direction_to(self, other: Self) -> Self {
        (other - self).normalized()
    }

    pub fn dot(self, rhs: Self) -> Real {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    pub fn cross(self, rhs: Self) -> Self {
        Self {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x,
        }
    }

    pub fn component_product(self, rhs: Self) -> Self {
        Self {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
            z: self.z * rhs.z,
        }
    }

    pub fn to_world_coords(self, transform: &Mat4) -> Self {
        transform.transform(self)
    }

    pub fn to_local_coords(self, transform: &Mat4) -> Self {
        transform.transform_inverse(self)
    }

    pub fn to_world_coords_dir(self, transform: &Mat4) -> Self {
        transform.transform_direction(self)
    }

    pub fn to_local_coords_dir(self, transform: &Mat4) -> Self {
        transform.transform_inverse_direction(self)
    }
}

impl Add for Vec3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl Sub for Vec3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl SubAssign for Vec3 {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl Mul<Real> for Vec3 {
    type Output = Self;

    fn mul(self, scalar: Real) -> Self::Output {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

impl Mul<Vec3> for Real {
    type Output = Vec3;

    fn mul(self, rhs: Vec3) -> Self::Output {
        rhs * self
    }
}

impl MulAssign<Real> for Vec3 {
    fn mul_assign(&mut self, rhs: Real) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl Div<Real> for Vec3 {
    type Output = Self;

    fn div(self, scalar: Real) -> Self::Output {
        Self {
            x: self.x / scalar,
            y: self.y / scalar,
            z: self.z / scalar,
        }
    }
}

impl DivAssign<Real> for Vec3 {
    fn div_assign(&mut self, rhs: Real) {
        self.x /= rhs;
        self.y /= rhs;
        self.z /= rhs;
    }
}

impl Neg for Vec3 {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

impl fmt::Display for Vec3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let Vec3 { x, y, z } = *self;
        write!(f, "<{x}, {y}, {z}>")
    }
}

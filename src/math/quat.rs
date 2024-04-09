use std::ops::{Mul, MulAssign};

use crate::{precision::Real, Vec3};

/// Holds a three degree of freedom orientation.
///
/// Quaternions have several mathematical properties that make them useful
/// for representing orientations, but require four items of data to
/// hold the three degrees of freedom. These four items of data can
/// be viewed as the coefficients of a complex number with three
/// imaginary parts. The mathematics of the quaternion is then
/// defined and is roughly correspondent to the math of 3D
/// rotations. A quaternion is only a valid rotation if it is
/// normalised: i.e. it has a length of 1.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct Quat {
    pub r: Real,
    pub i: Real,
    pub j: Real,
    pub k: Real,
}

impl Quat {
    pub const IDENTITY: Self = Self::from_rijk(1.0, 0.0, 0.0, 0.0);

    #[inline(always)]
    pub const fn from_rijk(r: Real, i: Real, j: Real, k: Real) -> Self {
        Self { r, i, j, k }
    }

    #[inline(always)]
    pub const fn to_array(self) -> [Real; 4] {
        [self.r, self.i, self.j, self.k]
    }

    #[inline(always)]
    pub fn normalized(self) -> Self {
        let squared_magnitude =
            self.r * self.r + self.i * self.i + self.j * self.j + self.k * self.k;

        if squared_magnitude == 0.0 {
            return Self {
                r: 1.0,
                i: 0.0,
                j: 0.0,
                k: 0.0,
            };
        }

        let inv_magnitude = squared_magnitude.sqrt().recip();

        Self {
            r: self.r * inv_magnitude,
            i: self.i * inv_magnitude,
            j: self.j * inv_magnitude,
            k: self.k * inv_magnitude,
        }
    }

    #[inline(always)]
    pub fn rotated_by_vector(self, vector: Vec3) -> Self {
        let q = Self::from_rijk(0.0, vector.x, vector.y, vector.z);

        self * q
    }

    /// Adds the given vector to this, scaled by the given amount.
    ///
    /// #[inline(always)]This is used to update the orientation quaternion by a rotation and time.
    pub fn add_scaled_vector(self, vector: Vec3, scale: Real) -> Self {
        let q = Self::from_rijk(0.0, vector.x * scale, vector.y * scale, vector.z * scale);
        let q = q * self;

        Self {
            r: self.r + q.r * 0.5,
            i: self.i + q.i * 0.5,
            j: self.j + q.j * 0.5,
            k: self.k + q.k * 0.5,
        }
    }

    #[inline(always)]
    pub fn inverse(self) -> Self {
        debug_assert!(self.is_normalized());
        self.conjugate()
    }

    #[inline(always)]
    pub fn conjugate(self) -> Self {
        Quat::from_rijk(self.r, -self.i, -self.j, -self.k)
    }

    #[inline(always)]
    pub fn is_normalized(self) -> bool {
        (self.squared_magnitude() - 1.0).abs() <= 2e-4
    }

    #[inline(always)]
    pub fn magnitude(self) -> Real {
        self.squared_magnitude().sqrt()
    }

    #[inline(always)]
    pub fn squared_magnitude(self) -> Real {
        self.r * self.r + self.i * self.i + self.j * self.j + self.k * self.k
    }
}

impl Mul<Quat> for Quat {
    type Output = Quat;

    fn mul(self, multiplier: Quat) -> Self::Output {
        Self {
            r: self.r * multiplier.r
                - self.i * multiplier.i
                - self.j * multiplier.j
                - self.k * multiplier.k,
            i: self.r * multiplier.i + self.i * multiplier.r + self.j * multiplier.k
                - self.k * multiplier.j,
            j: self.r * multiplier.j + self.j * multiplier.r + self.k * multiplier.i
                - self.i * multiplier.k,
            k: self.r * multiplier.k + self.k * multiplier.r + self.i * multiplier.j
                - self.j * multiplier.i,
        }
    }
}

impl MulAssign<Quat> for Quat {
    fn mul_assign(&mut self, multiplier: Quat) {
        let q = *self;

        self.r = q.r * multiplier.r - q.i * multiplier.i - q.j * multiplier.j - q.k * multiplier.k;
        self.i = q.r * multiplier.i + q.i * multiplier.r + q.j * multiplier.k - q.k * multiplier.j;
        self.j = q.r * multiplier.j + q.j * multiplier.r + q.k * multiplier.i - q.i * multiplier.k;
        self.k = q.r * multiplier.k + q.k * multiplier.r + q.i * multiplier.j - q.j * multiplier.i;
    }
}

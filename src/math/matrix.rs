use std::ops::{Mul, MulAssign};

use crate::{precision::Real, Vec3};

use super::quat::Quat;

/// Holds an inertia tensor, consisting of a 3x3 row-major matrix.
/// This matrix is not padding to produce an aligned structure, since
/// it is most commonly used with a mass (single real) and two
/// damping coefficients to make the 12-element characteristics array
/// of a rigid body.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Mat3 {
    pub data: [Real; 9],
}

impl Mat3 {
    pub const IDENTITY: Self = Self::new([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);

    pub const fn new(data: [Real; 9]) -> Self {
        Self { data }
    }

    /// Transform the given vector by this matrix.
    pub fn transform(&self, vector: Vec3) -> Vec3 {
        Vec3::new(
            vector.x * self.data[0] + vector.y * self.data[1] + vector.z * self.data[2],
            vector.x * self.data[3] + vector.y * self.data[4] + vector.z * self.data[5],
            vector.x * self.data[6] + vector.y * self.data[7] + vector.z * self.data[8],
        )
    }

    /// Returns a matrix which is this matrix multiplied by the given
    /// other matrix.
    pub fn mul_mat3(&self, rhs: Self) -> Self {
        Mat3::new([
            self.data[0] * rhs.data[0] + self.data[1] * rhs.data[3] + self.data[2] * rhs.data[6],
            self.data[0] * rhs.data[1] + self.data[1] * rhs.data[4] + self.data[2] * rhs.data[7],
            self.data[0] * rhs.data[2] + self.data[1] * rhs.data[5] + self.data[2] * rhs.data[8],
            self.data[3] * rhs.data[0] + self.data[4] * rhs.data[3] + self.data[5] * rhs.data[6],
            self.data[3] * rhs.data[1] + self.data[4] * rhs.data[4] + self.data[5] * rhs.data[7],
            self.data[3] * rhs.data[2] + self.data[4] * rhs.data[5] + self.data[5] * rhs.data[8],
            self.data[6] * rhs.data[0] + self.data[7] * rhs.data[3] + self.data[8] * rhs.data[6],
            self.data[6] * rhs.data[1] + self.data[7] * rhs.data[4] + self.data[8] * rhs.data[7],
            self.data[6] * rhs.data[2] + self.data[7] * rhs.data[5] + self.data[8] * rhs.data[8],
        ])
    }

    pub fn inverse(&self) -> Self {
        let t1 = self.data[0] * self.data[4];
        let t2 = self.data[0] * self.data[5];
        let t3 = self.data[1] * self.data[3];
        let t4 = self.data[2] * self.data[3];
        let t5 = self.data[1] * self.data[6];
        let t6 = self.data[2] * self.data[6];

        // Calculate the determinant.
        let det = t1 * self.data[8] - t2 * self.data[7] - t3 * self.data[8]
            + t4 * self.data[7]
            + t5 * self.data[5]
            - t6 * self.data[4];

        debug_assert_ne!(det, 0.0);

        let invd = det.recip();
        Self::new([
            (self.data[4] * self.data[8] - self.data[5] * self.data[7]) * invd,
            -(self.data[1] * self.data[8] - self.data[2] * self.data[7]) * invd,
            (self.data[1] * self.data[5] - self.data[2] * self.data[4]) * invd,
            -(self.data[3] * self.data[8] - self.data[5] * self.data[6]) * invd,
            (self.data[0] * self.data[8] - t6) * invd,
            -(t2 - t4) * invd,
            (self.data[3] * self.data[7] - self.data[4] * self.data[6]) * invd,
            -(self.data[0] * self.data[7] - t5) * invd,
            (t1 - t3) * invd,
        ])
    }

    pub fn invert(&mut self) {
        *self = self.inverse();
    }

    pub fn determinant(&self) -> Real {
        let t1 = self.data[0] * self.data[4];
        let t2 = self.data[0] * self.data[5];
        let t3 = self.data[1] * self.data[3];
        let t4 = self.data[2] * self.data[3];
        let t5 = self.data[1] * self.data[6];
        let t6 = self.data[2] * self.data[6];

        t1 * self.data[8] - t2 * self.data[7] - t3 * self.data[8]
            + t4 * self.data[7]
            + t5 * self.data[5]
            - t6 * self.data[4]
    }

    pub fn transpose(&self) -> Self {
        Self::new([
            self.data[0],
            self.data[3],
            self.data[6],
            self.data[1],
            self.data[4],
            self.data[7],
            self.data[2],
            self.data[5],
            self.data[8],
        ])
    }
}

impl Mul<Vec3> for Mat3 {
    type Output = Vec3;

    /// Transform the given vector by this matrix.
    fn mul(self, vector: Vec3) -> Self::Output {
        self.transform(vector)
    }
}

impl Mul<Mat3> for Mat3 {
    type Output = Mat3;

    /// Returns a matrix which is this matrix multiplied by the given
    /// other matrix.
    fn mul(self, rhs: Mat3) -> Self::Output {
        self.mul_mat3(rhs)
    }
}

impl From<Quat> for Mat3 {
    fn from(value: Quat) -> Self {
        let Quat { r, i, j, k } = value;

        Self::new([
            1.0 - (2.0 * j * j + 2.0 * k * k),
            2.0 * i * j + 2.0 * k * r,
            2.0 * i * k - 2.0 * j * r,
            2.0 * i * j - 2.0 * k * r,
            1.0 - (2.0 * i * i + 2.0 * k * k),
            2.0 * j * k + 2.0 * i * r,
            2.0 * i * k + 2.0 * j * r,
            2.0 * j * k - 2.0 * i * r,
            1.0 - (2.0 * i * i + 2.0 * j * j),
        ])
    }
}

impl MulAssign for Mat3 {
    /// Multiplies this matrix in place by the given other matrix.
    fn mul_assign(&mut self, rhs: Self) {
        let mut t1: Real;
        let mut t2: Real;
        let mut t3: Real;
        t1 = self.data[0] * rhs.data[0] + self.data[1] * rhs.data[3] + self.data[2] * rhs.data[6];
        t2 = self.data[0] * rhs.data[1] + self.data[1] * rhs.data[4] + self.data[2] * rhs.data[7];
        t3 = self.data[0] * rhs.data[2] + self.data[1] * rhs.data[5] + self.data[2] * rhs.data[8];
        self.data[0] = t1;
        self.data[1] = t2;
        self.data[2] = t3;
        t1 = self.data[3] * rhs.data[0] + self.data[4] * rhs.data[3] + self.data[5] * rhs.data[6];
        t2 = self.data[3] * rhs.data[1] + self.data[4] * rhs.data[4] + self.data[5] * rhs.data[7];
        t3 = self.data[3] * rhs.data[2] + self.data[4] * rhs.data[5] + self.data[5] * rhs.data[8];
        self.data[3] = t1;
        self.data[4] = t2;
        self.data[5] = t3;
        t1 = self.data[6] * rhs.data[0] + self.data[7] * rhs.data[3] + self.data[8] * rhs.data[6];
        t2 = self.data[6] * rhs.data[1] + self.data[7] * rhs.data[4] + self.data[8] * rhs.data[7];
        t3 = self.data[6] * rhs.data[2] + self.data[7] * rhs.data[5] + self.data[8] * rhs.data[8];
        self.data[6] = t1;
        self.data[7] = t2;
        self.data[8] = t3;
    }
}

/// Holds a transform matrix, consisting of a rotation matrix and
/// a position. The matrix has 12 elements, it is assumed that the
/// remaining four are (0,0,0,1); producing a homogenous matrix.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Mat4 {
    pub data: [Real; 12],
}

impl Mat4 {
    pub const IDENTITY: Self =
        Self::new([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);

    pub const fn new(data: [Real; 12]) -> Self {
        Self { data }
    }

    pub fn from_orientation_and_position(orientation: Quat, position: Vec3) -> Self {
        Self::new([
            1.0 - 2.0 * orientation.j * orientation.j - 2.0 * orientation.k * orientation.k,
            2.0 * orientation.i * orientation.j - 2.0 * orientation.r * orientation.k,
            2.0 * orientation.i * orientation.k + 2.0 * orientation.r * orientation.j,
            position.x,
            2.0 * orientation.i * orientation.j + 2.0 * orientation.r * orientation.k,
            1.0 - 2.0 * orientation.i * orientation.i - 2.0 * orientation.k * orientation.k,
            2.0 * orientation.j * orientation.k - 2.0 * orientation.r * orientation.i,
            position.y,
            2.0 * orientation.i * orientation.k - 2.0 * orientation.r * orientation.j,
            2.0 * orientation.j * orientation.k + 2.0 * orientation.r * orientation.i,
            1.0 - 2.0 * orientation.i * orientation.i - 2.0 * orientation.j * orientation.j,
            position.z,
        ])
    }

    /// Transform the given vector by this matrix.
    pub fn transform(self, vector: Vec3) -> Vec3 {
        Vec3::new(
            vector.x * self.data[0]
                + vector.y * self.data[1]
                + vector.z * self.data[2]
                + self.data[3],
            vector.x * self.data[4]
                + vector.y * self.data[5]
                + vector.z * self.data[6]
                + self.data[7],
            vector.x * self.data[8]
                + vector.y * self.data[9]
                + vector.z * self.data[10]
                + self.data[11],
        )
    }

    /// Transform the given vector by the transformational inverse
    /// of this matrix.
    ///
    /// **NOTE**: This function relies on the fact that the inverse of
    /// a pure rotation matrix is its transpose. It separates the
    /// translational and rotation components, transposes the
    /// rotation, and multiplies out. If the matrix is not a
    /// scale and shear free transform matrix, then this function
    /// will not give correct results.
    pub fn transform_inverse(&self, mut vector: Vec3) -> Vec3 {
        vector.x -= self.data[3];
        vector.y -= self.data[7];
        vector.z -= self.data[11];

        Vec3::new(
            vector.x * self.data[0] + vector.y * self.data[4] + vector.z * self.data[8],
            vector.x * self.data[1] + vector.y * self.data[5] + vector.z * self.data[9],
            vector.x * self.data[2] + vector.y * self.data[6] + vector.z * self.data[10],
        )
    }

    /// Transform the given direction vector by this matrix.
    ///
    /// **NOTE**: When a direction is converted between frames of
    /// reference, there is no translation required.
    pub fn transform_direction(&self, vector: Vec3) -> Vec3 {
        Vec3::new(
            vector.x * self.data[0] + vector.y * self.data[1] + vector.z * self.data[2],
            vector.x * self.data[4] + vector.y * self.data[5] + vector.z * self.data[6],
            vector.x * self.data[8] + vector.y * self.data[9] + vector.z * self.data[10],
        )
    }

    /// Transform the given direction vector by the
    /// transformational inverse of this matrix.
    ///
    /// **NOTE**: This function relies on the fact that the inverse of
    /// a pure rotation matrix is its transpose. It separates the
    /// translational and rotation components, transposes the
    /// rotation, and multiplies out. If the matrix is not a
    /// scale and shear free transform matrix, then this function
    /// will not give correct results.
    ///
    /// **NOTE**: When a direction is converted between frames of
    /// reference, there is no translation required.
    pub fn transform_inverse_direction(&self, vector: Vec3) -> Vec3 {
        Vec3::new(
            vector.x * self.data[0] + vector.y * self.data[4] + vector.z * self.data[8],
            vector.x * self.data[1] + vector.y * self.data[5] + vector.z * self.data[9],
            vector.x * self.data[2] + vector.y * self.data[6] + vector.z * self.data[10],
        )
    }

    /// Returns a matrix which is this matrix multiplied by the given
    /// other matrix.
    pub fn mul_mat4(&self, rhs: Self) -> Self {
        let mut result = Mat4::default();

        result.data[0] =
            rhs.data[0] * self.data[0] + rhs.data[4] * self.data[1] + rhs.data[8] * self.data[2];
        result.data[4] =
            rhs.data[0] * self.data[4] + rhs.data[4] * self.data[5] + rhs.data[8] * self.data[6];
        result.data[8] =
            rhs.data[0] * self.data[8] + rhs.data[4] * self.data[9] + rhs.data[8] * self.data[10];
        result.data[1] =
            rhs.data[1] * self.data[0] + rhs.data[5] * self.data[1] + rhs.data[9] * self.data[2];
        result.data[5] =
            rhs.data[1] * self.data[4] + rhs.data[5] * self.data[5] + rhs.data[9] * self.data[6];
        result.data[9] =
            rhs.data[1] * self.data[8] + rhs.data[5] * self.data[9] + rhs.data[9] * self.data[10];
        result.data[2] =
            rhs.data[2] * self.data[0] + rhs.data[6] * self.data[1] + rhs.data[10] * self.data[2];
        result.data[6] =
            rhs.data[2] * self.data[4] + rhs.data[6] * self.data[5] + rhs.data[10] * self.data[6];
        result.data[10] =
            rhs.data[2] * self.data[8] + rhs.data[6] * self.data[9] + rhs.data[10] * self.data[10];
        result.data[3] = rhs.data[3] * self.data[0]
            + rhs.data[7] * self.data[1]
            + rhs.data[11] * self.data[2]
            + self.data[3];
        result.data[7] = rhs.data[3] * self.data[4]
            + rhs.data[7] * self.data[5]
            + rhs.data[11] * self.data[6]
            + self.data[7];
        result.data[11] = rhs.data[3] * self.data[8]
            + rhs.data[7] * self.data[9]
            + rhs.data[11] * self.data[10]
            + self.data[11];

        result
    }

    pub fn inverse(&self) -> Self {
        let det = self.determinant();
        debug_assert_ne!(det, 0.0);
        let det = det.recip();

        let mut data = [0.0; 12];
        data[0] = (-self.data[9] * self.data[6] + self.data[5] * self.data[10]) * det;
        data[4] = (self.data[8] * self.data[6] - self.data[4] * self.data[10]) * det;
        data[8] = (-self.data[8] * self.data[5] + self.data[4] * self.data[9]) * det;

        data[1] = (self.data[9] * self.data[2] - self.data[1] * self.data[10]) * det;
        data[5] = (-self.data[8] * self.data[2] + self.data[0] * self.data[10]) * det;
        data[9] = (self.data[8] * self.data[1] - self.data[0] * self.data[9]) * det;

        data[2] = (-self.data[5] * self.data[2] + self.data[1] * self.data[6]) * det;
        data[6] = (self.data[4] * self.data[2] - self.data[0] * self.data[6]) * det;
        data[10] = (-self.data[4] * self.data[1] + self.data[0] * self.data[5]) * det;

        data[3] = (self.data[9] * self.data[6] * self.data[3]
            - self.data[5] * self.data[10] * self.data[3]
            - self.data[9] * self.data[2] * self.data[7]
            + self.data[1] * self.data[10] * self.data[7]
            + self.data[5] * self.data[2] * self.data[11]
            - self.data[1] * self.data[6] * self.data[11])
            * det;
        data[7] = (-self.data[8] * self.data[6] * self.data[3]
            + self.data[4] * self.data[10] * self.data[3]
            + self.data[8] * self.data[2] * self.data[7]
            - self.data[0] * self.data[10] * self.data[7]
            - self.data[4] * self.data[2] * self.data[11]
            + self.data[0] * self.data[6] * self.data[11])
            * det;
        data[11] = (self.data[8] * self.data[5] * self.data[3]
            - self.data[4] * self.data[9] * self.data[3]
            - self.data[8] * self.data[1] * self.data[7]
            + self.data[0] * self.data[9] * self.data[7]
            + self.data[4] * self.data[1] * self.data[11]
            - self.data[0] * self.data[5] * self.data[11])
            * det;

        Self::new(data)
    }

    pub fn invert(&mut self) {
        *self = self.inverse();
    }

    pub fn determinant(&self) -> Real {
        self.data[8] * self.data[5] * self.data[2]
            + self.data[4] * self.data[9] * self.data[2]
            + self.data[8] * self.data[1] * self.data[6]
            - self.data[0] * self.data[9] * self.data[6]
            - self.data[4] * self.data[1] * self.data[10]
            + self.data[0] * self.data[5] * self.data[10]
    }
}

impl Mul<Vec3> for Mat4 {
    type Output = Vec3;

    /// Transform the given vector by this matrix.
    fn mul(self, vector: Vec3) -> Self::Output {
        self.transform(vector)
    }
}

impl Mul<Mat4> for Mat4 {
    type Output = Mat4;

    /// Returns a matrix which is this matrix multiplied by the given
    /// other matrix.
    fn mul(self, rhs: Mat4) -> Self::Output {
        self.mul_mat4(rhs)
    }
}

impl MulAssign<Mat4> for Mat4 {
    /// Multiplies this matrix in place by the given other matrix.
    fn mul_assign(&mut self, rhs: Mat4) {
        *self = self.mul_mat4(rhs)
    }
}

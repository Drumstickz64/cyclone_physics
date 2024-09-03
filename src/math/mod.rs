use crate::{Mat4, Vec3};

pub mod matrix;
pub mod quat;
pub mod vector;

pub fn make_orthonormal_basis(a: Vec3, b: Vec3) -> (Vec3, Vec3, Vec3) {
    let a = a.normalized();
    let c = a.cross(b);
    assert_ne!(
        c.squared_magnitude(),
        0.0,
        "Vectors a ({:?}) and b ({:?}) are parallel",
        a,
        b
    );

    let c = c.normalized();

    let b = c.cross(a);

    (a, b, c)
}

pub fn local_to_world(point: Vec3, transform: Mat4) -> Vec3 {
    transform.transform(point)
}

pub fn world_to_local(point: Vec3, transform: Mat4) -> Vec3 {
    transform.transform_inverse(point)
}

pub fn world_to_local_dirn(direction: Vec3, transform: Mat4) -> Vec3 {
    transform.transform_inverse_direction(direction)
}

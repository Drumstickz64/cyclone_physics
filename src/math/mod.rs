use self::vector::Vec3;

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

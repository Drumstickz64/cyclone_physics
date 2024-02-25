use self::vector::Vector;

pub mod vector;

pub fn make_orthonormal_basis(a: Vector, b: Vector) -> (Vector, Vector, Vector) {
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

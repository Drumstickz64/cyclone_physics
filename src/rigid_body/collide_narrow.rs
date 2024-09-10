use crate::{precision::Real, Mat4, Vec3};

use super::RigidBodyId;

/// A contact represents two bodies in contact. Resolving a
/// contact removes their interpenetration, and applies sufficient
/// impulse to keep them apart. Colliding bodies may also rebound.
/// Contacts can be used to represent positional joints, by making
/// the contact constraint keep the bodies in their correct orientation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Contact {
    pub body_a: RigidBodyId,
    pub body_b: Option<RigidBodyId>,
    /// Holds the position of the contact in world coordinates.
    pub point: Vec3,
    /// Holds the direction of the contact in world coordinates.
    pub normal: Vec3,
    /// Holds the depth of penetration at the contact point. If both
    /// bodies are specified, then the contact point should be midway
    /// between the interpenetrating points.
    pub penetration: Real,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Collider {
    Single(Primitive),
    Set(Vec<Primitive>),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Primitive {
    pub offset: Mat4,
    pub shape: PrimitiveShape,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PrimitiveShape {
    Sphere(Sphere),
    Plane(Plane),
    Cuboid(Cuboid),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sphere {
    pub radius: Real,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Plane {
    pub normal: Vec3,
    pub offset: Real,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cuboid {
    pub half_size: Vec3,
}

#[derive(Debug)]
pub struct CollisionData<'contacts> {
    pub body_a: RigidBodyId,
    pub body_b: Option<RigidBodyId>,
    pub contacts: &'contacts mut Vec<Contact>,
}

pub mod algo {
    use super::*;

    #[rustfmt::skip]
    pub fn transform_cuboid_to_axis(cuboid: Cuboid, cuboid_transform: &Mat4, axis: Vec3) -> Real {
        debug_assert!(axis.is_normalized());

        cuboid.half_size.x * (cuboid_transform.get_x_axis().dot(axis)).abs() +
        cuboid.half_size.y * (cuboid_transform.get_y_axis().dot(axis)).abs() +
        cuboid.half_size.z * (cuboid_transform.get_z_axis().dot(axis)).abs()
    }

    pub fn cuboids_penetration_on_axis(
        cuboid_a: Cuboid,
        transform_a: &Mat4,
        cuboid_b: Cuboid,
        transform_b: &Mat4,
        axis: Vec3,
        to_center: Vec3,
    ) -> Real {
        let projected_a = transform_cuboid_to_axis(cuboid_a, transform_a, axis);
        let projected_b = transform_cuboid_to_axis(cuboid_b, transform_b, axis);

        let distance = to_center.dot(axis).abs();
        // Return the overlap (i.e., positive indicates
        // overlap, negative indicates separation).
        projected_a + projected_b - distance
    }

    pub fn cuboid_edge_edge_contact_point(
        axis_a: Vec3,
        edge_point_a: Vec3,
        axis_b: Vec3,
        edge_point_b: Vec3,
    ) -> Vec3 {
        // The vector between the test points on each edge.
        let to_st = edge_point_a - edge_point_b;
        // How much of those vectors are in the direction of each edge?
        let dp_sta_a = to_st.dot(axis_a);
        let dp_sta_b = to_st.dot(axis_b);
        // Work out how far along each edge is the closest point.
        let sm_a = axis_a.squared_magnitude();
        let sm_b = axis_b.squared_magnitude();
        let dot_product_edges = axis_a.dot(axis_b);
        let denom = sm_a * sm_b - dot_product_edges.powi(2);
        let a = (dot_product_edges * dp_sta_b - sm_b * dp_sta_a) / denom;
        let b = (sm_a * dp_sta_b - dot_product_edges * dp_sta_a) / denom;
        // Use a point midway between the two nearest points.
        let nearest_point_a = edge_point_a + axis_a * a;
        let nearest_point_b = edge_point_b + axis_b * b;

        nearest_point_a * 0.5 + nearest_point_b * 0.5
    }

    pub fn sphere_and_sphere(
        sphere_a: Sphere,
        transform_a: &Mat4,
        sphere_b: Sphere,
        transform_b: &Mat4,
        data: CollisionData,
    ) {
        let position_a = transform_a.get_position();
        let position_b = transform_b.get_position();

        let midline = position_a - position_b;
        let distance = midline.magnitude();

        if distance <= 0.0 || distance > sphere_a.radius + sphere_b.radius {
            return;
        }

        let normal = midline / distance;
        data.contacts.push(Contact {
            body_a: data.body_a,
            body_b: data.body_b,
            point: position_a + midline * 0.5,
            normal,
            penetration: sphere_a.radius + sphere_b.radius - distance,
        });
    }

    pub fn sphere_and_half_space(
        sphere: Sphere,
        sphere_transform: &Mat4,
        plane: Plane,
        data: CollisionData,
    ) {
        let sphere_pos = sphere_transform.get_position();
        let sphere_distance = sphere_pos.dot(plane.normal) - sphere.radius - plane.offset;
        if sphere_distance >= 0.0 {
            return;
        }

        data.contacts.push(Contact {
            body_a: data.body_a,
            body_b: data.body_b,
            point: sphere_pos - plane.normal * (sphere_distance + sphere.radius),
            normal: plane.normal,
            penetration: -sphere_distance,
        });
    }

    pub fn cuboid_and_half_space(
        cuboid: Cuboid,
        cuboid_transform: &Mat4,
        plane: Plane,
        data: CollisionData,
    ) {
        // Work out the projected radius of the cuboid onto the plane direction
        let projected_radius = transform_cuboid_to_axis(cuboid, cuboid_transform, plane.normal);

        // Work out how far the cuboid is from the origin
        let cuboid_distance = cuboid_transform.get_position().dot(plane.normal) - projected_radius;

        // Check for the intersection
        if cuboid_distance > plane.offset {
            return;
        }

        // We have an intersection, so find the intersection points. We can make
        // do with only checking vertices. If the cuboid is resting on a plane
        // or on an edge, it will be reported as four or two contact points.
        // Go through each combination of + and - for each half-size.
        static MULTS: [Vec3; 8] = [
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(-1.0, 1.0, 1.0),
            Vec3::new(1.0, -1.0, 1.0),
            Vec3::new(-1.0, -1.0, 1.0),
            Vec3::new(1.0, 1.0, -1.0),
            Vec3::new(-1.0, 1.0, -1.0),
            Vec3::new(1.0, -1.0, -1.0),
            Vec3::new(-1.0, -1.0, -1.0),
        ];

        for mult in MULTS {
            let vertex_pos = cuboid_transform.transform(cuboid.half_size.component_product(mult));
            let vertex_distance = vertex_pos.dot(plane.normal);
            if vertex_distance <= plane.offset {
                data.contacts.push(Contact {
                    body_a: data.body_a,
                    body_b: data.body_b,
                    // The contact point is halfway between the vertex and the
                    // plane. We multiply the normal by half the separation
                    // distance and add the vertex location.
                    point: plane.normal * (vertex_distance - plane.offset) + vertex_pos,
                    normal: plane.normal,
                    penetration: plane.offset - vertex_distance,
                });
            }
        }
    }

    pub fn cuboid_and_sphere(
        cuboid: Cuboid,
        cuboid_transform: &Mat4,
        sphere: Sphere,
        sphere_transform: &Mat4,
        data: CollisionData,
    ) {
        let center = sphere_transform.get_position();
        let rel_center = cuboid_transform.transform_inverse(center);
        if rel_center.x.abs() - sphere.radius > cuboid.half_size.x
            || rel_center.y.abs() - sphere.radius > cuboid.half_size.y
            || rel_center.z.abs() - sphere.radius > cuboid.half_size.z
        {
            return;
        }

        let closest_point = rel_center.clamp(-cuboid.half_size, cuboid.half_size);
        let distance_squared = rel_center.distance_to_squared(closest_point);
        if distance_squared > sphere.radius.powi(2) {
            return;
        }

        let closest_point_world = cuboid_transform.transform(closest_point);
        data.contacts.push(Contact {
            body_a: data.body_a,
            body_b: data.body_b,
            point: closest_point_world,
            normal: center.direction_to(closest_point_world),
            penetration: sphere.radius - distance_squared.sqrt(),
        });
    }

    pub fn cuboid_and_cuboid(
        cuboid_a: Cuboid,
        transform_a: &Mat4,
        cuboid_b: Cuboid,
        transform_b: &Mat4,
        data: CollisionData,
    ) {
        let axes = [
            // Face axes for object A.
            transform_a.get_x_axis(),
            transform_a.get_y_axis(),
            transform_a.get_z_axis(),
            // Face axes for object B.
            transform_b.get_x_axis(),
            transform_b.get_y_axis(),
            transform_b.get_z_axis(),
            // Edge-edge axes
            transform_a.get_x_axis().cross(transform_b.get_x_axis()),
            transform_a.get_x_axis().cross(transform_b.get_y_axis()),
            transform_a.get_x_axis().cross(transform_b.get_z_axis()),
            transform_a.get_y_axis().cross(transform_b.get_x_axis()),
            transform_a.get_y_axis().cross(transform_b.get_y_axis()),
            transform_a.get_y_axis().cross(transform_b.get_z_axis()),
            transform_a.get_z_axis().cross(transform_b.get_x_axis()),
            transform_a.get_z_axis().cross(transform_b.get_y_axis()),
            transform_a.get_z_axis().cross(transform_b.get_z_axis()),
        ];

        let to_center = transform_b.get_position() - transform_a.get_position();

        let mut best_overlap = Real::MAX;
        let mut best_case = usize::MAX;

        for (i, axis) in axes.iter().copied().enumerate() {
            // Check for axes that were generated by (almost) parallel edges.
            if axis.squared_magnitude() < 0.001 {
                continue;
            }

            let axis = axis.normalized();
            let overlap = cuboids_penetration_on_axis(
                cuboid_a,
                transform_a,
                cuboid_b,
                transform_b,
                axis,
                to_center,
            );

            if overlap < best_overlap {
                best_overlap = overlap;
                best_case = i;
            }
        }

        assert_ne!(best_case, usize::MAX);

        // We now know there's a collision, and we know which
        // of the axes gave the smallest penetration. We now
        // can deal with it in different ways depending on
        // the case.
        match best_case {
            // We've got a vertex of cuboid two on a face of cuboid one.
            0..3 => fill_point_face_cuboid_cuboid(
                transform_a,
                cuboid_b,
                transform_b,
                to_center,
                data,
                best_case,
                best_overlap,
            ),
            // We've got a vertex of box one on a face of box two.
            // We use the same algorithm as above, but swap around
            // one and two (and therefore also the vector between their
            // centres).
            3..6 => fill_point_face_cuboid_cuboid(
                transform_b,
                cuboid_a,
                transform_a,
                to_center * -1.0,
                data,
                best_case - 3,
                best_overlap,
            ),
            // We've got an edge-edge contact. Find out which axes
            6..14 => {
                let axis_index_a = (best_case - 6) / 3;
                let axis_index_b = (best_case - 6) % 3;
                let axis_a = transform_a.get_axis_vector(axis_index_a);
                let axis_b = transform_b.get_axis_vector(axis_index_b);
                let mut axis = axis_a.cross(axis_b).normalized();

                // The axis should point from box one to box two.
                if to_center.dot(axis) > 0.0 {
                    axis = -axis;
                }

                // We have the axes, but not the edges: each axis has 4 edges parallel
                // to it, we need to find which of the 4 for each object. We do
                // that by finding the point in the centre of the edge. We know
                // its component in the direction of the box's collision axis is zero
                // (its a mid-point) and we determine which of the extremes in each
                // of the other axes is closest.
                let mut edge_point_a = cuboid_a.half_size;
                let mut edge_point_b = cuboid_b.half_size;
                for i in 0..3 {
                    if i == axis_index_a {
                        edge_point_a[i] = 0.0;
                    } else if transform_a.get_axis_vector(i).dot(axis) > 0.0 {
                        edge_point_a[i] = -edge_point_a[i]
                    }

                    if i == axis_index_b {
                        edge_point_b[i] = 0.0;
                    } else if transform_b.get_axis_vector(i).dot(axis) < 0.0 {
                        edge_point_b[i] = -edge_point_b[i];
                    }
                }

                // Move them into world coordinates (they are already oriented
                // correctly, since they have been derived from the axes).
                let edge_point_a_world = transform_a.transform(edge_point_a);
                let edge_point_b_world = transform_b.transform(edge_point_b);

                // So we have a point and a direction for the colliding edges.
                // We need to find out point of closest approach of the two
                // line-segments.
                let vertex = cuboid_edge_edge_contact_point(
                    axis_a,
                    edge_point_a_world,
                    axis_b,
                    edge_point_b_world,
                );

                data.contacts.push(Contact {
                    body_a: data.body_a,
                    body_b: data.body_b,
                    point: vertex,
                    normal: axis,
                    penetration: best_overlap,
                });
            }
            _ => unreachable!(
                "expected the axis index to be in range [0, 15), but it was {best_case}"
            ),
        }
    }

    /// This method is called when we know that a vertex from
    /// box two is in contact with box one.
    fn fill_point_face_cuboid_cuboid(
        transform_a: &Mat4,
        other: Cuboid,
        transform_b: &Mat4,
        to_center: Vec3,
        data: CollisionData,
        normal_index: usize,
        overlap: f32,
    ) {
        // We know which axis the collision is on (i.e. best),
        // but we need to work out which of the two faces on
        // this axis.
        let mut normal = transform_a.get_axis_vector(normal_index);
        if to_center.dot(normal) > 0.0 {
            normal = -normal;
        }

        // Work out which vertex of box two we're colliding with.
        // Using toCentre doesn't work!
        let mut vertex = other.half_size;
        if transform_b.get_x_axis().dot(normal) < 0.0 {
            vertex.x = -vertex.x
        };
        if transform_b.get_y_axis().dot(normal) < 0.0 {
            vertex.y = -vertex.y
        };
        if transform_b.get_z_axis().dot(normal) < 0.0 {
            vertex.z = -vertex.z
        };

        data.contacts.push(Contact {
            body_a: data.body_a,
            body_b: data.body_b,
            point: transform_b.transform(vertex),
            normal,
            penetration: overlap,
        });
    }
}

use crate::{precision::Real, Mat3, Mat4, Quat, Vec3};

/// A rigid body is the basic simulation object in the physics
/// core.
///
/// It has position and orientation data, along with first
/// derivatives. It can be integrated forward through time, and
/// have forces, torques and impulses (linear or angular) applied
/// to it. The rigid body manages its state and allows access
/// through a set of methods.
#[derive(Debug, Clone)]
pub struct RigidBody {
    pub inverse_mass: Real,
    /// Holds the amount of damping applied to linear
    /// motion. Damping is required to remove energy added
    /// through numerical instability in the integrator.
    pub linear_damping: Real,
    pub position: Vec3,
    pub orientation: Quat,
    pub velocity: Vec3,
    pub rotation: Vec3,
    /// Holds the inverse of the body’s inertia tensor. The
    /// intertia tensor provided must not be degenerate
    /// (that would mean the body had zero inertia for
    /// spinning along one axis). As long as the tensor is
    /// finite, it will be invertible. The inverse tensor
    /// is used for similar reasons to the use of inverse
    /// mass.
    ///
    /// The inertia tensor, unlike the other variables that
    /// define a rigid body, is given in body space.
    pub inverse_inertia_tensor: Mat3,
    /// Holds a transform matrix for converting body space into
    /// world space and vice versa. This can be achieved by calling
    /// the Vec3::to_*_coords() methods.
    transform_matrix: Mat4,
    /// Holds the inverse inertia tensor of the body in world
    /// space. The inverse inertia tensor member is specified in
    /// the body's local space.
    inverse_inertia_tensor_world: Mat3,
    force_accum: Vec3,
    torque_accum: Vec3,
    is_awake: bool,
}

impl RigidBody {
    pub fn calc_inertia_tensor(&self) -> Mat3 {
        self.inverse_inertia_tensor.inverse()
    }

    pub fn set_inertia_tensor(&mut self, inertia_tensor: Mat3) {
        self.inverse_inertia_tensor = inertia_tensor.inverse();
    }

    /// Integrates the rigid body forward in time by the given amount.
    /// This function uses a Newton-Euler integration method, which is a
    /// linear approximation to the correct integral. For this reason it
    /// may be inaccurate in some cases.
    pub fn integrate(&mut self, duration: Real) {
        todo!()
    }

    /// Adds the given force to centre of mass of the rigid body.
    /// The force is expressed in world-coordinates.
    pub fn add_force(&mut self, force: Vec3) {
        self.force_accum += force;
        self.is_awake = true;
    }

    /// Adds the given force to the given point on the rigid body.
    /// Both the force and the application point are given in world space.
    /// Because the force is not applied at the centre of mass, it may be split
    /// into both a force and torque.
    pub fn add_force_at_point(&mut self, force: Vec3, point: Vec3) {
        let from_center_of_mass = point - self.position;
        self.force_accum += force;
        self.torque_accum += from_center_of_mass.cross(force);
        self.is_awake = true;
    }

    /// Adds the given force to the given point on the rigid body.
    /// The direction of the force is given in world coordinates,
    /// but the application point is given in body space. This is
    /// useful for spring forces, or other forces fixed to the body.
    pub fn add_force_at_body_point(&mut self, force: Vec3, point: Vec3) {
        let point = self.get_point_in_world_space(point);
        self.add_force_at_point(force, point);
    }

    /// Calculates internal data from state data. This should be called
    /// after the body's state is altered directly (it is called
    /// automatically during integration). If you change the body's state
    /// and then intend to integrate before querying any data (such as
    /// the transform matrix), then you can ommit this step.
    pub fn update_derived_data(&mut self) {
        self.orientation = self.orientation.normalized();

        self.transform_matrix = Mat4::from_orientation_and_pos(self.orientation, self.position);
        self.inverse_inertia_tensor = inverse_inertia_tensor_to_world_coords(
            self.inverse_inertia_tensor,
            self.transform_matrix,
        );
    }

    pub fn get_point_in_world_space(&self, point: Vec3) -> Vec3 {
        self.transform_matrix.transform(point)
    }

    pub fn get_point_in_local_space(&self, point: Vec3) -> Vec3 {
        self.transform_matrix.transform_inverse(point)
    }

    pub(crate) fn clear_accumelators(&mut self, force: Vec3, point: Vec3) {
        self.force_accum = Vec3::ZERO;
        self.torque_accum = Vec3::ZERO;
    }
}

fn inverse_inertia_tensor_to_world_coords(iit: Mat3, rotmat: Mat4) -> Mat3 {
    let t4 =
        rotmat.data[0] * iit.data[0] + rotmat.data[1] * iit.data[3] + rotmat.data[2] * iit.data[6];
    let t9 =
        rotmat.data[0] * iit.data[1] + rotmat.data[1] * iit.data[4] + rotmat.data[2] * iit.data[7];
    let t14 =
        rotmat.data[0] * iit.data[2] + rotmat.data[1] * iit.data[5] + rotmat.data[2] * iit.data[8];
    let t28 =
        rotmat.data[4] * iit.data[0] + rotmat.data[5] * iit.data[3] + rotmat.data[6] * iit.data[6];
    let t33 =
        rotmat.data[4] * iit.data[1] + rotmat.data[5] * iit.data[4] + rotmat.data[6] * iit.data[7];
    let t38 =
        rotmat.data[4] * iit.data[2] + rotmat.data[5] * iit.data[5] + rotmat.data[6] * iit.data[8];
    let t52 =
        rotmat.data[8] * iit.data[0] + rotmat.data[9] * iit.data[3] + rotmat.data[10] * iit.data[6];
    let t57 =
        rotmat.data[8] * iit.data[1] + rotmat.data[9] * iit.data[4] + rotmat.data[10] * iit.data[7];
    let t62 =
        rotmat.data[8] * iit.data[2] + rotmat.data[9] * iit.data[5] + rotmat.data[10] * iit.data[8];

    Mat3::new([
        t4 * rotmat.data[0] + t9 * rotmat.data[1] + t14 * rotmat.data[2],
        t4 * rotmat.data[4] + t9 * rotmat.data[5] + t14 * rotmat.data[6],
        t4 * rotmat.data[8] + t9 * rotmat.data[9] + t14 * rotmat.data[10],
        t28 * rotmat.data[0] + t33 * rotmat.data[1] + t38 * rotmat.data[2],
        t28 * rotmat.data[4] + t33 * rotmat.data[5] + t38 * rotmat.data[6],
        t28 * rotmat.data[8] + t33 * rotmat.data[9] + t38 * rotmat.data[10],
        t52 * rotmat.data[0] + t57 * rotmat.data[1] + t62 * rotmat.data[2],
        t52 * rotmat.data[4] + t57 * rotmat.data[5] + t62 * rotmat.data[6],
        t52 * rotmat.data[8] + t57 * rotmat.data[9] + t62 * rotmat.data[10],
    ])
}

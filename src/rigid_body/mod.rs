pub mod fgen;
mod system;

pub use system::PhysicsSystem;

use slotmap::{new_key_type, SlotMap};

use derive_more::{From, Index, IndexMut, IntoIterator};

use crate::{math::local_to_world, precision::Real, Mat3, Mat4, Quat, Vec3};

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
    pub damping: Real,
    pub angular_damping: Real,
    pub position: Vec3,
    pub orientation: Quat,
    pub velocity: Vec3,
    pub angular_velocity: Vec3,
    /// used to add a constant acceleration to the acceleration
    /// computed from forces at each frame
    pub acceleration: Vec3,
    /// used to add a constant angular acceleration to the angular acceleration
    /// computed from torques at each frame
    pub angular_acceleration: Vec3,

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
    last_frame_acceleration: Vec3,
}

impl RigidBody {
    pub fn new(mass: Real) -> Self {
        assert_ne!(mass, 0.0, "Rigid bodies can't have zero mass");
        let inverse_mass = if mass == Real::INFINITY {
            0.0
        } else {
            mass.recip()
        };

        Self {
            inverse_mass,
            damping: 0.99,
            angular_damping: 0.99,
            position: Vec3::ZERO,
            orientation: Quat::IDENTITY,
            velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            acceleration: Vec3::ZERO,
            angular_acceleration: Vec3::ZERO,
            inverse_inertia_tensor: Mat3::IDENTITY,
            transform_matrix: Mat4::IDENTITY,
            inverse_inertia_tensor_world: Mat3::IDENTITY,
            force_accum: Vec3::ZERO,
            torque_accum: Vec3::ZERO,
            is_awake: true,
            last_frame_acceleration: Vec3::ZERO,
        }
    }

    pub fn with_mass(mut self, mass: Real) -> Self {
        self.set_mass(mass);
        self
    }

    pub fn with_inverse_mass(mut self, inverse_mass: Real) -> Self {
        self.inverse_mass = inverse_mass;
        self
    }

    pub fn with_damping(mut self, damping: Real) -> Self {
        self.damping = damping;
        self
    }

    pub fn with_angular_damping(mut self, angular_damping: Real) -> Self {
        self.angular_damping = angular_damping;
        self
    }

    pub fn with_position(mut self, position: Vec3) -> Self {
        self.position = position;
        self
    }

    pub fn with_orientation(mut self, orientation: Quat) -> Self {
        self.orientation = orientation;
        self
    }

    pub fn with_angular_velocity(mut self, angular_velocity: Vec3) -> Self {
        self.angular_velocity = angular_velocity;
        self
    }

    pub fn with_inverse_inertia_tensor(mut self, inverse_inertia_tensor: Mat3) -> Self {
        self.inverse_inertia_tensor = inverse_inertia_tensor;
        self
    }

    pub fn with_inertia_tensor(mut self, inertia_tensor: Mat3) -> Self {
        self.set_inertia_tensor(inertia_tensor);
        self
    }

    pub fn with_acceleration(mut self, frame_start_acceleration: Vec3) -> Self {
        self.acceleration = frame_start_acceleration;
        self
    }

    pub fn with_angular_acceleration(mut self, frame_start_angular_acceleration: Vec3) -> Self {
        self.angular_acceleration = frame_start_angular_acceleration;
        self
    }

    //NOTE: why >= and not >? doesn't inverse_mass being zero mean that the objects mass
    // is infinite?
    pub fn has_finite_mass(&self) -> bool {
        self.inverse_mass >= 0.0
    }

    pub fn mass(&self) -> Real {
        if self.inverse_mass == 0.0 {
            return Real::MAX;
        }

        self.inverse_mass.recip()
    }

    pub fn set_mass(&mut self, mass: Real) {
        assert_ne!(mass, 0.0, "Rigid bodies can't have zero mass");
        self.inverse_mass = mass.recip()
    }

    pub fn calc_inertia_tensor(&self) -> Mat3 {
        self.inverse_inertia_tensor.inverse()
    }

    pub fn inertia_tensor(&self) -> Mat3 {
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
        self.last_frame_acceleration = self.acceleration + self.force_accum * self.inverse_mass;
        let angular_acceleration =
            self.angular_acceleration + self.inverse_inertia_tensor_world * self.torque_accum;

        self.velocity =
            self.velocity * self.damping.powf(duration) + self.last_frame_acceleration * duration;
        self.angular_velocity = self.angular_velocity * self.angular_damping.powf(duration)
            + angular_acceleration * duration;

        self.position += self.velocity * duration;
        self.orientation = self
            .orientation
            .add_scaled_vector(self.angular_velocity, duration);

        self.update_derived_data();

        self.clear_accumelators();
    }

    pub fn transform_matrix(&self) -> Mat4 {
        self.transform_matrix
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
        let point = local_to_world(point, self.transform_matrix);
        self.add_force_at_point(force, point);
    }

    /// Adds the given torque to the rigid body.
    /// The force is expressed in world-coordinates.
    pub fn add_torque(&mut self, torque: Vec3) {
        self.torque_accum += torque;
        self.is_awake = true;
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

    pub fn get_point_in_local_space(&self, point: Vec3) -> Vec3 {
        self.transform_matrix.transform_inverse(point)
    }

    pub fn get_point_in_world_space(&self, point: Vec3) -> Vec3 {
        self.transform_matrix.transform(point)
    }

    pub(crate) fn clear_accumelators(&mut self) {
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

new_key_type! {
    pub struct RigidBodyId;
}

#[derive(Debug, Clone, Default, IntoIterator, Index, IndexMut, From)]
pub struct RigidBodySet {
    inner: SlotMap<RigidBodyId, RigidBody>,
}

impl RigidBodySet {
    pub fn new() -> Self {
        Self {
            inner: SlotMap::with_key(),
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            inner: SlotMap::with_capacity_and_key(capacity),
        }
    }

    pub fn insert(&mut self, value: RigidBody) -> RigidBodyId {
        self.inner.insert(value)
    }

    pub fn remove(&mut self, key: RigidBodyId) -> Option<RigidBody> {
        self.inner.remove(key)
    }

    pub fn clear(&mut self) {
        self.inner.clear()
    }

    pub fn len(&self) -> usize {
        self.inner.len()
    }

    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    pub fn get(&self, key: RigidBodyId) -> Option<&RigidBody> {
        self.inner.get(key)
    }

    pub fn get_mut(&mut self, key: RigidBodyId) -> Option<&mut RigidBody> {
        self.inner.get_mut(key)
    }

    pub fn capacity(&self) -> usize {
        self.inner.capacity()
    }

    pub fn bodies(&self) -> impl Iterator<Item = &RigidBody> {
        self.inner.values()
    }

    pub fn bodies_mut(&mut self) -> impl Iterator<Item = &mut RigidBody> {
        self.inner.values_mut()
    }

    pub fn handles(&self) -> impl Iterator<Item = RigidBodyId> + '_ {
        self.inner.keys()
    }

    pub fn get_disjoint_mut<const N: usize>(
        &mut self,
        keys: [RigidBodyId; N],
    ) -> Option<[&mut RigidBody; N]> {
        self.inner.get_disjoint_mut(keys)
    }

    pub fn contains(&self, key: RigidBodyId) -> bool {
        self.inner.contains_key(key)
    }

    pub fn reserve(&mut self, additional: usize) {
        self.inner.reserve(additional)
    }

    pub fn iter(&self) -> impl Iterator<Item = (RigidBodyId, &RigidBody)> {
        self.inner.iter()
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (RigidBodyId, &mut RigidBody)> {
        self.inner.iter_mut()
    }

    pub fn drain(&mut self) -> impl Iterator<Item = (RigidBodyId, RigidBody)> + '_ {
        self.inner.drain()
    }
}

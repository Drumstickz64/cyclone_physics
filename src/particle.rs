use slotmap::{new_key_type, SlotMap};

use derive_more::{AsMut, AsRef, From, Index, IndexMut, IntoIterator};

use crate::{math::vector::Vec3, precision::Real};

#[derive(Debug, Clone)]
pub struct Particle {
    position: Vec3,
    velocity: Vec3,
    acceleration: Vec3,
    damping: Real,
    inverse_mass: Real,
    force_accum: Vec3,
}

impl Particle {
    pub fn new(mass: Real) -> Self {
        assert_ne!(mass, 0.0, "Particles can't have zero mass");
        let inverse_mass = if mass == Real::INFINITY {
            0.0
        } else {
            mass.recip()
        };

        Self {
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            acceleration: Vec3::ZERO,
            damping: 0.99,
            inverse_mass,
            force_accum: Vec3::ZERO,
        }
    }

    pub fn with_position(mut self, position: Vec3) -> Self {
        self.position = position;
        self
    }

    pub fn with_velocity(mut self, velocity: Vec3) -> Self {
        self.velocity = velocity;
        self
    }

    pub fn with_acceleration(mut self, acceleration: Vec3) -> Self {
        self.acceleration = acceleration;
        self
    }

    pub fn with_damping(mut self, damping: Real) -> Self {
        assert!((0.0..=1.0).contains(&damping));
        self.damping = damping;
        self
    }

    pub fn with_mass(mut self, mass: Real) -> Self {
        self.set_mass(mass);
        self
    }

    pub fn integrate(&mut self, duration: Real) {
        if self.inverse_mass <= 0.0 {
            return;
        }

        assert_ne!(duration, 0.0);

        self.position += self.velocity * duration;

        let resulting_acc = self.acceleration + self.force_accum * self.inverse_mass;

        self.velocity = self.velocity * self.damping.powf(duration) + resulting_acc * duration;

        self.clear_accumulator();
    }

    pub fn add_force(&mut self, force: Vec3) {
        self.force_accum += force;
    }

    pub fn kinetic_energy(&self) -> Real {
        if self.inverse_mass == 0.0 {
            return Real::MAX;
        }

        0.5 * self.inverse_mass.recip() * self.velocity.squared_magnitude()
    }

    pub fn position(&self) -> Vec3 {
        self.position
    }

    pub fn set_position(&mut self, position: Vec3) {
        self.position = position;
    }

    pub fn velocity(&self) -> Vec3 {
        self.velocity
    }

    pub fn set_velocity(&mut self, velocity: Vec3) {
        self.velocity = velocity;
    }

    pub fn acceleration(&self) -> Vec3 {
        self.acceleration
    }

    pub fn set_acceleration(&mut self, acceleration: Vec3) {
        self.acceleration = acceleration;
    }

    pub fn damping(&self) -> f32 {
        self.damping
    }

    pub fn set_damping(&mut self, damping: Real) {
        self.damping = damping;
    }

    pub fn inverse_mass(&self) -> f32 {
        self.inverse_mass
    }

    pub fn set_inverse_mass(&mut self, inverse_mass: Real) {
        self.inverse_mass = inverse_mass;
    }

    pub fn mass(&self) -> Real {
        if self.inverse_mass == 0.0 {
            return Real::MAX;
        }

        self.inverse_mass.recip()
    }

    pub fn set_mass(&mut self, mass: Real) {
        assert_ne!(mass, 0.0, "Particles can't have zero mass");
        self.inverse_mass = mass.recip();
    }

    pub fn clear_accumulator(&mut self) {
        self.force_accum = Vec3::ZERO;
    }
}

new_key_type! {
    pub struct ParticleHandle;
}

#[derive(Debug, Clone, Default, IntoIterator, Index, IndexMut, From, AsRef, AsMut)]
pub struct ParticleSet {
    inner: SlotMap<ParticleHandle, Particle>,
}

impl ParticleSet {
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

    pub fn insert(&mut self, value: Particle) -> ParticleHandle {
        self.inner.insert(value)
    }

    pub fn remove(&mut self, key: ParticleHandle) -> Option<Particle> {
        self.inner.remove(key)
    }

    pub fn clear(&mut self) {
        self.inner.clear()
    }

    pub fn len(&self) -> usize {
        self.inner.len()
    }

    pub fn get(&self, key: ParticleHandle) -> Option<&Particle> {
        self.inner.get(key)
    }

    pub fn get_mut(&mut self, key: ParticleHandle) -> Option<&mut Particle> {
        self.inner.get_mut(key)
    }

    pub fn capacity(&self) -> usize {
        self.inner.capacity()
    }

    pub fn particles(&self) -> Particles<'_> {
        Particles(self.inner.values())
    }

    pub fn particles_mut(&mut self) -> ParticlesMut<'_> {
        ParticlesMut(self.inner.values_mut())
    }

    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    pub fn handles(&self) -> Handles {
        Handles(self.inner.keys())
    }

    pub fn get_disjoint_mut<const N: usize>(
        &mut self,
        keys: [ParticleHandle; N],
    ) -> Option<[&mut Particle; N]> {
        self.inner.get_disjoint_mut(keys)
    }

    pub fn contains_handle(&self, key: ParticleHandle) -> bool {
        self.inner.contains_key(key)
    }

    pub fn iter(&self) -> Iter {
        Iter(self.inner.iter())
    }

    pub fn iter_mut(&mut self) -> IterMut {
        IterMut(self.inner.iter_mut())
    }

    pub fn reserve(&mut self, additional: usize) {
        self.inner.reserve(additional)
    }

    pub fn drain(&mut self) -> Drain {
        Drain(self.inner.drain())
    }
}

#[derive(Debug, Clone)]
pub struct Particles<'a>(slotmap::basic::Values<'a, ParticleHandle, Particle>);

impl<'a> Iterator for Particles<'a> {
    type Item = &'a Particle;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next()
    }
}

impl<'a> ExactSizeIterator for Particles<'a> {}

#[derive(Debug)]
pub struct ParticlesMut<'a>(slotmap::basic::ValuesMut<'a, ParticleHandle, Particle>);

impl<'a> Iterator for ParticlesMut<'a> {
    type Item = &'a mut Particle;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next()
    }
}

impl<'a> ExactSizeIterator for ParticlesMut<'a> {}

#[derive(Debug, Clone)]
pub struct Iter<'a>(slotmap::basic::Iter<'a, ParticleHandle, Particle>);

impl<'a> Iterator for Iter<'a> {
    type Item = (ParticleHandle, &'a Particle);

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next()
    }
}

impl<'a> ExactSizeIterator for Iter<'a> {}

#[derive(Debug)]
pub struct IterMut<'a>(slotmap::basic::IterMut<'a, ParticleHandle, Particle>);

impl<'a> Iterator for IterMut<'a> {
    type Item = (ParticleHandle, &'a mut Particle);

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next()
    }
}

impl<'a> ExactSizeIterator for IterMut<'a> {}

#[derive(Debug)]
pub struct Drain<'a>(slotmap::basic::Drain<'a, ParticleHandle, Particle>);

impl<'a> Iterator for Drain<'a> {
    type Item = (ParticleHandle, Particle);

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next()
    }
}

impl<'a> ExactSizeIterator for Drain<'a> {}

#[derive(Debug, Clone)]
pub struct Handles<'a>(slotmap::basic::Keys<'a, ParticleHandle, Particle>);

impl<'a> Iterator for Handles<'a> {
    type Item = ParticleHandle;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next()
    }
}

impl<'a> ExactSizeIterator for Handles<'a> {}

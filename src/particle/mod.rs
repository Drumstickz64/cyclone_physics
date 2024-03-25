pub mod contacts;
pub mod fgen;
pub mod links;
mod pipeline;

pub use pipeline::ParticlePipeline;

use crate::{precision::Real, Vec3};

use slotmap::{new_key_type, SlotMap};

use derive_more::{AsMut, AsRef, From, Index, IndexMut, IntoIterator};

#[derive(Debug, Clone)]
pub struct Particle {
    pub position: Vec3,
    pub velocity: Vec3,
    pub acceleration: Vec3,
    pub damping: Real,
    pub inverse_mass: Real,
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

        debug_assert_ne!(duration, 0.0);

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

    pub(crate) fn clear_accumulator(&mut self) {
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

    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
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

    pub fn particles(&self) -> impl Iterator<Item = &Particle> {
        self.inner.values()
    }

    pub fn particles_mut(&mut self) -> impl Iterator<Item = &mut Particle> {
        self.inner.values_mut()
    }

    pub fn handles(&self) -> impl Iterator<Item = ParticleHandle> + '_ {
        self.inner.keys()
    }

    pub fn get_disjoint_mut<const N: usize>(
        &mut self,
        keys: [ParticleHandle; N],
    ) -> Option<[&mut Particle; N]> {
        self.inner.get_disjoint_mut(keys)
    }

    pub fn contains(&self, key: ParticleHandle) -> bool {
        self.inner.contains_key(key)
    }

    pub fn reserve(&mut self, additional: usize) {
        self.inner.reserve(additional)
    }

    pub fn iter(&self) -> impl Iterator<Item = (ParticleHandle, &Particle)> {
        self.inner.iter()
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (ParticleHandle, &mut Particle)> {
        self.inner.iter_mut()
    }

    pub fn drain(&mut self) -> impl Iterator<Item = (ParticleHandle, Particle)> + '_ {
        self.inner.drain()
    }
}

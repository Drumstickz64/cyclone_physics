use derive_more::{AsMut, AsRef, From, Index, IndexMut, IntoIterator};

use slotmap::{new_key_type, SlotMap};

use crate::{
    math::vector::Vec3,
    particle::{ParticleHandle, ParticleSet},
    precision::Real,
};

pub trait ParticleForceGenerator {
    fn update_force(
        &mut self,
        particles: &mut ParticleSet,
        particle: ParticleHandle,
        duration: Real,
    );
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ParticleForceRegistration {
    pub particle: ParticleHandle,
    pub fg: ForceGeneratorHandle,
}

pub struct ParticleSpring {
    pub other: ParticleHandle,
    pub spring_constant: Real,
    pub rest_length: Real,
}

impl ParticleForceGenerator for ParticleSpring {
    fn update_force(
        &mut self,
        particles: &mut ParticleSet,
        particle_key: ParticleHandle,
        _duration: Real,
    ) {
        let other_pos = particles[self.other].position;
        let particle = &mut particles[particle_key];
        let delta = particle.position - other_pos;

        let force =
            -self.spring_constant * (delta.magnitude() - self.rest_length) * delta.normalized();

        particle.add_force(force);
    }
}

pub struct ParticleAnchoredSpring {
    pub anchor: Vec3,
    pub spring_constant: Real,
    pub rest_length: Real,
}

impl ParticleForceGenerator for ParticleAnchoredSpring {
    fn update_force(
        &mut self,
        particles: &mut ParticleSet,
        particle_key: ParticleHandle,
        _duration: Real,
    ) {
        let particle = &mut particles[particle_key];
        let delta = particle.position - self.anchor;

        let force =
            -self.spring_constant * (delta.magnitude() - self.rest_length) * delta.normalized();

        particle.add_force(force);
    }
}

pub struct ParticleBungee {
    pub other: ParticleHandle,
    pub spring_constant: Real,
    pub rest_length: Real,
}

impl ParticleForceGenerator for ParticleBungee {
    fn update_force(
        &mut self,
        particles: &mut ParticleSet,
        particle_key: ParticleHandle,
        _duration: Real,
    ) {
        let other_pos = particles[self.other].position;
        let particle = &mut particles[particle_key];
        let delta = particle.position - other_pos;
        let delta_mag = delta.magnitude();

        if delta_mag <= self.rest_length {
            return;
        }

        let force = -self.spring_constant * (-self.rest_length) * delta.normalized();

        particle.add_force(force);
    }
}

pub struct ParticleBuoyancy {
    pub max_depth: Real,
    pub volume: Real,
    pub liquid_height: Real,
    pub liquid_density: Real,
}

impl ParticleBuoyancy {
    pub fn new(max_depth: Real, volume: Real, liquid_height: Real) -> Self {
        Self {
            max_depth,
            volume,
            liquid_height,
            liquid_density: 1000.0,
        }
    }

    pub fn with_liquid_density(mut self, liquid_density: Real) -> Self {
        self.liquid_density = liquid_density;
        self
    }
}

impl ParticleForceGenerator for ParticleBuoyancy {
    fn update_force(
        &mut self,
        particles: &mut ParticleSet,
        particle: ParticleHandle,
        _duration: Real,
    ) {
        let particle = &mut particles[particle];
        let depth = particle.position.y;

        if depth >= self.liquid_height + self.max_depth {
            return;
        }

        let buoyancy_force = if depth <= self.liquid_height - self.max_depth {
            self.volume * self.liquid_density
        } else {
            let degree_of_submersion =
                (depth - self.liquid_height - self.max_depth) / 2.0 * self.max_depth;
            degree_of_submersion * self.volume * self.liquid_density
        };

        particle.add_force(Vec3::Y * buoyancy_force);
    }
}

new_key_type! {
    pub struct ForceGeneratorHandle;
}

#[derive(IntoIterator, Index, IndexMut, From, AsRef, AsMut)]
pub struct ForceGeneratorSet {
    inner: SlotMap<ForceGeneratorHandle, Box<dyn ParticleForceGenerator>>,
}

impl Default for ForceGeneratorSet {
    fn default() -> Self {
        Self::new()
    }
}

impl ForceGeneratorSet {
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

    pub fn insert(&mut self, value: Box<dyn ParticleForceGenerator>) -> ForceGeneratorHandle {
        self.inner.insert(value)
    }

    pub fn remove(&mut self, key: ForceGeneratorHandle) -> Option<Box<dyn ParticleForceGenerator>> {
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

    pub fn capacity(&self) -> usize {
        self.inner.capacity()
    }

    pub fn reserve(&mut self, additional: usize) {
        self.inner.reserve(additional)
    }

    pub fn get(&self, key: ForceGeneratorHandle) -> Option<&dyn ParticleForceGenerator> {
        self.inner.get(key).map(|fg| &**fg)
    }

    pub fn get_mut(
        &mut self,
        key: ForceGeneratorHandle,
    ) -> Option<&mut Box<dyn ParticleForceGenerator>> {
        self.inner.get_mut(key)
    }

    pub fn force_generators(&self) -> impl Iterator<Item = &dyn ParticleForceGenerator> {
        self.inner.values().map(|fg| fg.as_ref())
    }

    pub fn force_generators_mut(
        &mut self,
    ) -> impl Iterator<Item = &mut Box<dyn ParticleForceGenerator>> {
        self.inner.values_mut()
    }

    pub fn handles(&self) -> impl Iterator<Item = ForceGeneratorHandle> + '_ {
        self.inner.keys()
    }

    pub fn get_disjoint_mut<const N: usize>(
        &mut self,
        keys: [ForceGeneratorHandle; N],
    ) -> Option<[&mut Box<dyn ParticleForceGenerator>; N]> {
        self.inner.get_disjoint_mut(keys)
    }

    pub fn contains_handle(&self, key: ForceGeneratorHandle) -> bool {
        self.inner.contains_key(key)
    }

    pub fn iter(
        &self,
    ) -> impl Iterator<Item = (ForceGeneratorHandle, &Box<dyn ParticleForceGenerator>)> {
        self.inner.iter()
    }

    pub fn iter_mut(
        &mut self,
    ) -> impl Iterator<Item = (ForceGeneratorHandle, &mut Box<dyn ParticleForceGenerator>)> {
        self.inner.iter_mut()
    }

    pub fn drain(
        &mut self,
    ) -> impl Iterator<Item = (ForceGeneratorHandle, Box<dyn ParticleForceGenerator>)> + '_ {
        self.inner.drain()
    }
}

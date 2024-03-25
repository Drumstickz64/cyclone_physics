use std::collections::HashSet;

use derive_more::{AsMut, AsRef, From, Index, IndexMut, IntoIterator};

use slotmap::{new_key_type, SlotMap};

use crate::{precision::Real, Vec3};

use super::{Particle, ParticleHandle, ParticleSet};

pub trait ParticleForceGenerator {
    fn update_forces(&self, particles: &mut ParticleSet, duration: Real);
}

pub trait ParticleTargetedForceGenerator {
    fn update_force(&self, target: &mut Particle, duration: Real);
}

pub struct ParticleSpring {
    pub target: ParticleHandle,
    pub other: ParticleHandle,
    pub spring_constant: Real,
    pub rest_length: Real,
}

impl ParticleForceGenerator for ParticleSpring {
    fn update_forces(&self, particles: &mut ParticleSet, _duration: Real) {
        let [particle, other] = particles
            .get_disjoint_mut([self.target, self.other])
            .unwrap();

        let delta = particle.position - other.position;

        let force =
            -self.spring_constant * (delta.magnitude() - self.rest_length) * delta.normalized();

        particle.add_force(force);
        other.add_force(-force);
    }
}

pub struct ParticleAnchoredSpring {
    pub target: ParticleHandle,
    pub anchor: Vec3,
    pub spring_constant: Real,
    pub rest_length: Real,
}

impl ParticleForceGenerator for ParticleAnchoredSpring {
    fn update_forces(&self, particles: &mut ParticleSet, _duration: Real) {
        let particle = &mut particles[self.target];
        let delta = particle.position - self.anchor;

        let force =
            -self.spring_constant * (delta.magnitude() - self.rest_length) * delta.normalized();

        particle.add_force(force);
    }
}

pub struct ParticleBungee {
    pub target: ParticleHandle,
    pub other: ParticleHandle,
    pub spring_constant: Real,
    pub rest_length: Real,
}

impl ParticleForceGenerator for ParticleBungee {
    fn update_forces(&self, particles: &mut ParticleSet, _duration: Real) {
        let [particle, other] = particles
            .get_disjoint_mut([self.target, self.other])
            .unwrap();

        let delta = particle.position - other.position;
        let delta_squared_magnitude = delta.squared_magnitude();

        if delta_squared_magnitude <= self.rest_length * self.rest_length {
            return;
        }

        let force = -self.spring_constant
            * (delta_squared_magnitude.sqrt() - self.rest_length)
            * delta.normalized();

        particle.add_force(force);
        other.add_force(-force);
    }
}

pub struct ParticleBuoyancy {
    pub target: ParticleHandle,
    pub max_depth: Real,
    pub volume: Real,
    pub liquid_height: Real,
    pub liquid_density: Real,
}

impl ParticleBuoyancy {
    pub fn new(target: ParticleHandle, max_depth: Real, volume: Real, liquid_height: Real) -> Self {
        Self {
            target,
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
    fn update_forces(&self, particles: &mut ParticleSet, _duration: Real) {
        let particle = &mut particles[self.target];
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
pub struct ForceGeneratorSet<T: ParticleTargetedForceGenerator> {
    #[into_iterator]
    #[index]
    #[index_mut]
    inner: SlotMap<ForceGeneratorHandle, T>,
    registry: HashSet<ParticleHandle>,
}

impl<T: ParticleTargetedForceGenerator> ForceGeneratorSet<T> {
    pub fn new() -> Self {
        Self {
            inner: SlotMap::with_key(),
            registry: HashSet::new(),
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            inner: SlotMap::with_capacity_and_key(capacity),
            registry: HashSet::with_capacity(capacity),
        }
    }

    pub fn insert(&mut self, value: T) -> ForceGeneratorHandle {
        self.inner.insert(value)
    }

    pub fn remove(&mut self, key: ForceGeneratorHandle) -> Option<T> {
        self.inner.remove(key)
    }

    pub fn register(&mut self, particle: ParticleHandle) -> bool {
        self.registry.insert(particle)
    }

    pub fn unregister(&mut self, particle: ParticleHandle) -> bool {
        self.registry.remove(&particle)
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

    pub fn get(&self, key: ForceGeneratorHandle) -> Option<&T> {
        self.inner.get(key)
    }

    pub fn get_mut(&mut self, key: ForceGeneratorHandle) -> Option<&mut T> {
        self.inner.get_mut(key)
    }

    pub fn generators(&self) -> impl Iterator<Item = &T> {
        self.inner.values()
    }

    pub fn generators_mut(&mut self) -> impl Iterator<Item = &mut T> {
        self.inner.values_mut()
    }

    pub fn handles(&self) -> impl Iterator<Item = ForceGeneratorHandle> + '_ {
        self.inner.keys()
    }

    pub fn get_disjoint_mut<const N: usize>(
        &mut self,
        keys: [ForceGeneratorHandle; N],
    ) -> Option<[&mut T; N]> {
        self.inner.get_disjoint_mut(keys)
    }

    pub fn contains(&self, key: ForceGeneratorHandle) -> bool {
        self.inner.contains_key(key)
    }

    pub fn iter(&self) -> impl Iterator<Item = (ForceGeneratorHandle, &T)> {
        self.inner.iter()
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (ForceGeneratorHandle, &mut T)> {
        self.inner.iter_mut()
    }

    pub fn drain(&mut self) -> impl Iterator<Item = (ForceGeneratorHandle, T)> + '_ {
        self.inner.drain()
    }
}

impl<T: ParticleTargetedForceGenerator> Default for ForceGeneratorSet<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: ParticleTargetedForceGenerator> ParticleForceGenerator for ForceGeneratorSet<T> {
    fn update_forces(&self, particles: &mut ParticleSet, duration: Real) {
        for fg in self.inner.values() {
            for particle in particles.particles_mut() {
                fg.update_force(particle, duration);
            }
        }
    }
}

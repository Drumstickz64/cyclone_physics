use derive_more::{From, Index, IndexMut, IntoIterator};
use downcast_rs::{impl_downcast, Downcast};
use slotmap::{new_key_type, SlotMap};

use super::{ParticleId, ParticleSet};
use crate::{precision::Real, Vec3};

pub trait ParticleForceGenerator: Downcast {
    fn update_forces(&self, particles: &mut ParticleSet, duration: Real);
}

new_key_type! {
    pub struct ForceGeneratorId;
}

#[derive(Default, IntoIterator, Index, IndexMut, From)]
pub struct ParticleForceGeneratorSet {
    inner: SlotMap<ForceGeneratorId, Box<dyn ParticleForceGenerator>>,
}

impl_downcast!(ParticleForceGenerator);

impl ParticleForceGeneratorSet {
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

    pub fn update_forces(&self, particles: &mut ParticleSet, duration: Real) {
        for generator in self.inner.values() {
            generator.update_forces(particles, duration);
        }
    }

    pub fn insert<F: ParticleForceGenerator + 'static>(&mut self, value: F) -> ForceGeneratorId {
        self.inner.insert(Box::new(value))
    }

    pub fn delete(&mut self, key: ForceGeneratorId) {
        self.inner.remove(key);
    }

    pub fn remove<F: ParticleForceGenerator>(&mut self, key: ForceGeneratorId) -> Option<Box<F>> {
        self.inner.remove(key)?.downcast().ok()
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

    pub fn get<F: ParticleForceGenerator>(&self, key: ForceGeneratorId) -> Option<&F> {
        self.inner.get(key)?.downcast_ref()
    }

    pub fn get_mut<F: ParticleForceGenerator>(&mut self, key: ForceGeneratorId) -> Option<&mut F> {
        self.inner.get_mut(key)?.downcast_mut()
    }

    pub fn capacity(&self) -> usize {
        self.inner.capacity()
    }

    pub fn contains(&self, key: ForceGeneratorId) -> bool {
        self.inner.contains_key(key)
    }

    pub fn reserve(&mut self, additional: usize) {
        self.inner.reserve(additional)
    }
}
pub struct ParticleSpring {
    pub target: ParticleId,
    pub other: ParticleId,
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
    pub target: ParticleId,
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
    pub target: ParticleId,
    pub other: ParticleId,
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
    pub target: ParticleId,
    pub max_depth: Real,
    pub volume: Real,
    pub liquid_height: Real,
    pub liquid_density: Real,
}

impl ParticleBuoyancy {
    pub fn new(target: ParticleId, max_depth: Real, volume: Real, liquid_height: Real) -> Self {
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

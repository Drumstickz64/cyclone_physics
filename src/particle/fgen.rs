use crate::{precision::Real, Vec3};
use std::collections::HashSet;

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

#[derive(Clone, Debug)]
pub struct ForceTargetSet<T: ParticleTargetedForceGenerator> {
    generator: T,
    targets: HashSet<ParticleHandle>,
}

impl<T: ParticleTargetedForceGenerator> ForceTargetSet<T> {
    pub fn new(generator: T) -> Self {
        Self {
            generator,
            targets: HashSet::new(),
        }
    }

    pub fn with_target_capacity(generator: T, capacity: usize) -> Self {
        Self {
            generator,
            targets: HashSet::with_capacity(capacity),
        }
    }

    pub fn add_target(&mut self, particle: ParticleHandle) -> bool {
        self.targets.insert(particle)
    }

    pub fn remove_target(&mut self, particle: ParticleHandle) -> bool {
        self.targets.remove(&particle)
    }

    pub fn clear_targets(&mut self) {
        self.targets.clear()
    }

    pub fn targets_len(&self) -> usize {
        self.targets.len()
    }

    pub fn has_target(&self) -> bool {
        todo!()
    }

    pub fn targets(&self) -> impl Iterator<Item = &ParticleHandle> {
        self.targets.iter()
    }
}

impl<T: ParticleTargetedForceGenerator> ParticleForceGenerator for ForceTargetSet<T> {
    fn update_forces(&self, particles: &mut ParticleSet, duration: Real) {
        for target in self.targets.iter().copied() {
            self.generator
                .update_force(&mut particles[target], duration)
        }
    }
}

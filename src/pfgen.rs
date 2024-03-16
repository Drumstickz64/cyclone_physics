use slotmap::{new_key_type, SlotMap};

use crate::{
    math::vector::Vec3,
    particle::{ParticleKey, ParticleSet},
    precision::Real,
};

new_key_type! {
    pub struct ForceGeneratorKey;
}

// TODO: make into proper struct
pub type ForceGeneratorSet = SlotMap<ForceGeneratorKey, Box<dyn ParticleForceGenerator>>;

pub trait ParticleForceGenerator {
    fn update_force(&mut self, particles: &mut ParticleSet, particle: ParticleKey, duration: Real);
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ParticleForceRegistration {
    pub particle: ParticleKey,
    pub fg: ForceGeneratorKey,
}

pub struct ParticleSpring {
    pub other: ParticleKey,
    pub spring_constant: Real,
    pub rest_length: Real,
}

impl ParticleForceGenerator for ParticleSpring {
    fn update_force(
        &mut self,
        particles: &mut ParticleSet,
        particle_key: ParticleKey,
        _duration: Real,
    ) {
        let other_pos = particles[self.other].position();
        let particle = &mut particles[particle_key];
        let delta = particle.position() - other_pos;

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
        particle_key: ParticleKey,
        _duration: Real,
    ) {
        let particle = &mut particles[particle_key];
        let delta = particle.position() - self.anchor;

        let force =
            -self.spring_constant * (delta.magnitude() - self.rest_length) * delta.normalized();

        particle.add_force(force);
    }
}

pub struct ParticleBungee {
    pub other: ParticleKey,
    pub spring_constant: Real,
    pub rest_length: Real,
}

impl ParticleForceGenerator for ParticleBungee {
    fn update_force(
        &mut self,
        particles: &mut ParticleSet,
        particle_key: ParticleKey,
        _duration: Real,
    ) {
        let other_pos = particles[self.other].position();
        let particle = &mut particles[particle_key];
        let delta = particle.position() - other_pos;
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
        particle: ParticleKey,
        _duration: Real,
    ) {
        let particle = &mut particles[particle];
        let depth = particle.position().y;

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

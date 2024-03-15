use slotmap::{new_key_type, SlotMap};

use crate::{
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
pub struct ParticleForceRegistration {
    pub particle: ParticleKey,
    pub fg: ForceGeneratorKey,
}

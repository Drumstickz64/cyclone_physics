use crate::{
    particle::{ParticleHandle, ParticleSet},
    pfgen::{ForceGeneratorHandle, ForceGeneratorSet, ParticleForceRegistration},
    precision::Real,
};

pub struct ParticlePipeline {
    force_registry: Vec<ParticleForceRegistration>,
}

impl ParticlePipeline {
    pub fn new() -> Self {
        Self {
            force_registry: Vec::new(),
        }
    }

    pub fn step(
        &mut self,
        particles: &mut ParticleSet,
        fgs: &mut ForceGeneratorSet,
        duration: Real,
    ) {
        for reg in self.force_registry.iter().copied() {
            let fg = &mut fgs[reg.fg];
            fg.update_force(particles, reg.particle, duration);
        }

        for particle in particles.particles_mut() {
            particle.integrate(duration);
        }
    }

    pub fn register_force(&mut self, particle: ParticleHandle, fg: ForceGeneratorHandle) {
        self.force_registry
            .push(ParticleForceRegistration { particle, fg })
    }

    pub fn unregister_force(&mut self, particle: ParticleHandle, fg: ForceGeneratorHandle) -> bool {
        let Some(i) = self
            .force_registry
            .iter()
            .position(|reg| reg.particle == particle && reg.fg == fg)
        else {
            return false;
        };

        self.force_registry.swap_remove(i);

        true
    }
}

impl Default for ParticlePipeline {
    fn default() -> Self {
        Self::new()
    }
}

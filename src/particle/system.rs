use crate::precision::Real;

use super::{
    contacts::{ParticleContact, ParticleContactGenerator, ParticleContactResolver},
    fgen::ParticleForceGeneratorSet,
    ParticleSet,
};

#[derive(Debug, Clone)]
pub struct ParticlePhysicsSystem {
    resolver: ParticleContactResolver,
    contacts: Vec<ParticleContact>,
    contacts_used: usize,
    calculate_iterations: bool,
}

impl ParticlePhysicsSystem {
    pub fn new(max_contacts: usize, iterations: u32) -> Self {
        Self {
            resolver: ParticleContactResolver::new(iterations),
            contacts: vec![ParticleContact::default(); max_contacts],
            contacts_used: 0,
            calculate_iterations: iterations == 0,
        }
    }

    pub fn step(
        &mut self,
        particles: &mut ParticleSet,
        generators: &mut ParticleForceGeneratorSet,
        duration: Real,
    ) {
        generators.update_forces(particles, duration);

        for particle in particles.particles_mut() {
            particle.integrate(duration);
        }

        if self.contacts_used > 0 {
            if self.calculate_iterations {
                self.resolver.iterations = (self.contacts_used * 2) as u32;
            }
            self.resolver.resolve(
                &mut self.contacts[..self.contacts_used],
                particles,
                duration,
            )
        }
    }

    pub fn start_frame(&mut self, particles: &mut ParticleSet) {
        for particle in particles.particles_mut() {
            particle.clear_accumulator();
        }
    }

    pub fn generate_contacts(
        &mut self,
        cg: &impl ParticleContactGenerator,
        particles: &ParticleSet,
    ) {
        let contacts = &mut self.contacts[self.contacts_used..];
        if contacts.is_empty() {
            return;
        }

        let used = cg.add_contacts(contacts, particles);
        self.contacts_used += used;
    }
}

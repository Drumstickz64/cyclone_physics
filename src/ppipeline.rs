use crate::{
    particle::ParticleSet,
    pcontacts::{ParticleContact, ParticleContactGenerator, ParticleContactResolver},
    precision::Real,
};

pub struct ParticlePipeline {
    resolver: ParticleContactResolver,
    contacts: Vec<ParticleContact>,
    contacts_used: usize,
    calculate_iterations: bool,
}

impl ParticlePipeline {
    pub fn new(max_contacts: usize, iterations: u32) -> Self {
        Self {
            resolver: ParticleContactResolver::new(iterations),
            contacts: vec![ParticleContact::default(); max_contacts],
            contacts_used: 0,
            calculate_iterations: iterations == 0,
        }
    }

    pub fn step(&mut self, particles: &mut ParticleSet, duration: Real) {
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

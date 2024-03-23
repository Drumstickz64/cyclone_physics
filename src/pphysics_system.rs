use crate::{
    particle::{ParticleHandle, ParticleSet},
    pcontacts::{ContactGeneratorSet, ParticleContact, ParticleContactResolver},
    pfgen::{ForceGeneratorHandle, ForceGeneratorSet, ParticleForceRegistration},
    precision::Real,
};

pub struct ParticlePhysicsSystem {
    force_registry: Vec<ParticleForceRegistration>,
    resolver: ParticleContactResolver,
    contacts: Vec<ParticleContact>,
    calculate_iterations: bool,
}

impl ParticlePhysicsSystem {
    pub fn new(max_contacts: usize, iterations: u32) -> Self {
        Self {
            force_registry: Vec::new(),
            resolver: ParticleContactResolver::new(iterations),
            contacts: vec![ParticleContact::default(); max_contacts],
            calculate_iterations: iterations == 0,
        }
    }

    pub fn step(
        &mut self,
        particles: &mut ParticleSet,
        fgs: &mut ForceGeneratorSet,
        cgs: &mut ContactGeneratorSet,
        duration: Real,
    ) {
        for reg in self.force_registry.iter().copied() {
            let fg = &mut fgs[reg.fg];
            fg.update_force(particles, reg.particle, duration);
        }

        for particle in particles.particles_mut() {
            particle.integrate(duration);
        }

        let used_contacts = self.generate_contacts(cgs, particles);

        if used_contacts > 0 {
            if self.calculate_iterations {
                self.resolver.iterations = (used_contacts * 2) as u32;
            }
            self.resolver
                .resolve(&mut self.contacts[..used_contacts], particles, duration)
        }
    }

    pub fn register_force(&mut self, particle: ParticleHandle, fg: ForceGeneratorHandle) {
        self.force_registry
            .push(ParticleForceRegistration { particle, fg })
    }

    // we return bool instead of Option because we don't want to expose ParticleForceRegistration
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

    pub fn start_frame(&mut self, particles: &mut ParticleSet) {
        for particle in particles.particles_mut() {
            particle.clear_accumulator();
        }
    }

    pub fn generate_contacts(
        &mut self,
        cgs: &mut ContactGeneratorSet,
        particles: &ParticleSet,
    ) -> usize {
        let max_contacts = self.contacts.len();
        let mut contacts = &mut self.contacts[..];

        for contact_generator in cgs.generators() {
            let used = contact_generator.add_contacts(contacts, particles);
            contacts = &mut contacts[used..];
            if contacts.is_empty() {
                break;
            }
        }

        // return the number of contacts used
        max_contacts - contacts.len()
    }
}

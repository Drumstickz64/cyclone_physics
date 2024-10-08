use crate::{precision::Real, Vec3};

use super::{Particle, ParticleId, ParticleSet};

pub trait ParticleContactGenerator {
    fn add_contacts(&self, contacts: &mut [ParticleContact], particles: &ParticleSet) -> usize;
}

#[derive(Debug, Clone, Default)]
pub struct ParticleContact {
    pub particle_a: ParticleId,
    pub particle_b: Option<ParticleId>,
    pub data: ParticleContactData,
}

#[derive(Debug, Clone, Default)]
pub struct ParticleContactData {
    pub restitution: Real,
    pub normal: Vec3,
    pub penetration: Real,
    pub particle_a_movement: Vec3,
    pub particle_b_movement: Vec3,
}

impl ParticleContactData {
    pub fn new(restitution: Real, normal: Vec3, penetration: Real) -> Self {
        Self {
            restitution,
            normal,
            penetration,
            particle_a_movement: Vec3::ZERO,
            particle_b_movement: Vec3::ZERO,
        }
    }
}

impl ParticleContact {
    pub fn new(
        particle_a: ParticleId,
        particle_b: Option<ParticleId>,
        restitution: Real,
        normal: Vec3,
        penetration: Real,
    ) -> Self {
        Self {
            particle_a,
            particle_b,
            data: ParticleContactData {
                restitution,
                normal,
                penetration,
                particle_a_movement: Vec3::ZERO,
                particle_b_movement: Vec3::ZERO,
            },
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct ParticleContactResolver {
    pub iterations: u32,
    pub iterations_used: u32,
}

impl ParticleContactResolver {
    pub fn new(iterations: u32) -> Self {
        Self {
            iterations,
            iterations_used: 0,
        }
    }

    pub fn resolve(
        &mut self,
        contacts: &mut [ParticleContact],
        particles: &mut ParticleSet,
        duration: Real,
    ) {
        self.iterations_used = 0;

        while self.iterations_used < self.iterations {
            let num_contacts = contacts.len();
            let mut max = Real::MAX;
            let mut max_idx = num_contacts;

            for (i, contact) in contacts.iter_mut().enumerate() {
                let sep_vel = Self::calculate_seperating_velocity(
                    &particles[contact.particle_a],
                    contact.particle_b.map(|particle| &particles[particle]),
                    contact.data.normal,
                );

                if sep_vel < max && sep_vel < 0.0 || contact.data.penetration > 0.0 {
                    max = sep_vel;
                    max_idx = i;
                }

                if max_idx == num_contacts {
                    break;
                }

                if let Some(particle_b) = contact.particle_b {
                    let [particle_a, particle_b] = particles
                        .get_disjoint_mut([contact.particle_a, particle_b])
                        .unwrap();

                    Self::resolve_contact(
                        particle_a,
                        Some(particle_b),
                        duration,
                        &mut contact.data,
                    );
                } else {
                    Self::resolve_contact(
                        &mut particles[contact.particle_a],
                        None,
                        duration,
                        &mut contact.data,
                    );
                }

                self.iterations_used += 1;
            }
        }
    }

    fn resolve_contact(
        particle_a: &mut Particle,
        mut particle_b: Option<&mut Particle>,
        duration: Real,
        contact_data: &mut ParticleContactData,
    ) {
        Self::resolve_velocity(
            particle_a,
            particle_b.as_deref_mut(),
            duration,
            contact_data,
        );
        Self::resolve_interpenetration(particle_a, particle_b, duration, contact_data);
    }

    fn resolve_velocity(
        particle_a: &mut Particle,
        particle_b: Option<&mut Particle>,
        duration: Real,
        contact_data: &ParticleContactData,
    ) {
        let seperating_velocity = Self::calculate_seperating_velocity(
            particle_a,
            particle_b.as_deref(),
            contact_data.normal,
        );

        if seperating_velocity > 0.0 {
            return;
        }

        let mut new_sep_velocity = -contact_data.restitution * seperating_velocity;

        // Check the velocity buildup due to acceleration only.
        let acc_caused_velocity = match particle_b.as_ref() {
            Some(particle_b) => particle_a.acceleration - particle_b.acceleration,
            None => particle_a.acceleration,
        };

        let acc_caused_sep_velocity = acc_caused_velocity.dot(contact_data.normal) * duration;
        // If we’ve got a closing velocity due to aceleration buildup,
        // remove it from the new separating velocity.
        if acc_caused_sep_velocity < 0.0 {
            new_sep_velocity += acc_caused_sep_velocity * contact_data.restitution;

            // Make sure we haven’t removed more than was
            // there to remove.
            if new_sep_velocity < 0.0 {
                new_sep_velocity = 0.0;
            }
        }

        let delta_velocity = new_sep_velocity - seperating_velocity;

        let total_inv_mass = match particle_b.as_ref() {
            Some(particle_b) => particle_a.inverse_mass + particle_b.inverse_mass,
            None => particle_a.inverse_mass,
        };

        if total_inv_mass <= 0.0 {
            return;
        }

        let impulse = delta_velocity / total_inv_mass;
        let impulse_per_inv_mass = impulse * contact_data.normal;

        particle_a.velocity += impulse_per_inv_mass * particle_a.inverse_mass;

        if let Some(particle_b) = particle_b {
            particle_b.velocity += impulse_per_inv_mass * particle_b.inverse_mass;
        }
    }

    fn resolve_interpenetration(
        particle_a: &mut Particle,
        mut particle_b: Option<&mut Particle>,
        _duration: Real,
        contact_data: &mut ParticleContactData,
    ) {
        if contact_data.penetration <= 0.0 {
            return;
        }

        let total_inv_mass = match &particle_b {
            Some(particle_b) => particle_a.inverse_mass + particle_b.inverse_mass,
            None => particle_a.inverse_mass,
        };

        if total_inv_mass <= 0.0 {
            return;
        }

        let move_per_inv_mass = contact_data.normal * (contact_data.penetration / total_inv_mass);
        contact_data.particle_a_movement = move_per_inv_mass * particle_a.inverse_mass;
        contact_data.particle_b_movement = match particle_b.as_mut() {
            Some(particle_b) => move_per_inv_mass * particle_b.inverse_mass,
            None => Vec3::ZERO,
        };

        particle_a.position += contact_data.particle_a_movement;
        if let Some(particle_b) = particle_b {
            particle_b.position += contact_data.particle_b_movement;
        }
    }

    pub fn calculate_seperating_velocity(
        particle_a: &Particle,
        particle_b: Option<&Particle>,
        contact_normal: Vec3,
    ) -> Real {
        let relative_velocity = match particle_b {
            Some(particle_b) => particle_a.velocity - particle_b.velocity,
            None => particle_a.velocity,
        };

        relative_velocity.dot(contact_normal)
    }
}

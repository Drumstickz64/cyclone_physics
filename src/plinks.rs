use crate::{
    math::vector::Vec3,
    particle::{ParticleHandle, ParticleSet},
    pcontacts::{ParticleContact, ParticleContactData},
    precision::Real,
    ParticleContactGenerator,
};

#[derive(Debug, Clone)]
pub struct ParticleCable {
    pub particle_a: ParticleHandle,
    pub particle_b: ParticleHandle,
    pub max_length: Real,
    pub restitution: Real,
}

impl ParticleContactGenerator for ParticleCable {
    fn create_contact(&self, particles: &ParticleSet) -> Option<ParticleContact> {
        let position_a = particles[self.particle_a].position;
        let position_b = particles[self.particle_b].position;

        let length = position_a.distance_to(position_b);
        if length < self.max_length {
            return None;
        }

        let normal = position_a.direction_to(position_b);
        let penetration = length - self.max_length;

        Some(ParticleContact {
            particle_a: self.particle_a,
            particle_b: Some(self.particle_b),
            data: ParticleContactData::new(self.restitution, normal, penetration),
        })
    }
}

#[derive(Debug, Clone)]
pub struct ParticleRod {
    pub particle_a: ParticleHandle,
    pub particle_b: ParticleHandle,
    pub length: Real,
}

impl ParticleContactGenerator for ParticleRod {
    fn create_contact(&self, particles: &ParticleSet) -> Option<ParticleContact> {
        let position_a = particles[self.particle_a].position;
        let position_b = particles[self.particle_b].position;

        let curr_length = position_a.distance_to(position_b);
        if curr_length == self.length {
            return None;
        }

        let normal = if curr_length > self.length {
            position_a.direction_to(position_b)
        } else {
            position_b.direction_to(position_a)
        };

        let penetration = (curr_length - self.length).abs();
        let restitution = 0.0;

        Some(ParticleContact {
            particle_a: self.particle_a,
            particle_b: Some(self.particle_b),
            data: ParticleContactData::new(restitution, normal, penetration),
        })
    }
}
#[derive(Debug, Clone)]
pub struct ParticleGroundCollider {
    pub particle: ParticleHandle,
    pub particle_radius: Real,
    pub restitution: Real,
}

impl ParticleContactGenerator for ParticleGroundCollider {
    fn create_contact(&self, particles: &ParticleSet) -> Option<ParticleContact> {
        let position = particles[self.particle].position;
        if position.y > self.particle_radius {
            return None;
        }

        let normal = Vec3::Y;
        let penetration = position.y - self.particle_radius;

        Some(ParticleContact {
            particle_a: self.particle,
            particle_b: None,
            data: ParticleContactData::new(self.restitution, normal, penetration),
        })
    }
}

use crate::{precision::Real, Vec3};

use super::{
    contacts::{ParticleContact, ParticleContactData, ParticleContactGenerator},
    ParticleId, ParticleSet,
};

#[derive(Debug, Clone)]
pub struct ParticleCable {
    pub particle_a: ParticleId,
    pub particle_b: ParticleId,
    pub max_length: Real,
    pub restitution: Real,
}

impl ParticleContactGenerator for ParticleCable {
    fn add_contacts(&self, contacts: &mut [ParticleContact], particles: &ParticleSet) -> usize {
        let position_a = particles[self.particle_a].position;
        let position_b = particles[self.particle_b].position;

        let length = position_a.distance_to(position_b);
        if length < self.max_length {
            return 0;
        }

        let normal = position_a.direction_to(position_b);
        let penetration = length - self.max_length;

        contacts[0] = ParticleContact {
            particle_a: self.particle_a,
            particle_b: Some(self.particle_b),
            data: ParticleContactData::new(self.restitution, normal, penetration),
        };

        1
    }
}

#[derive(Debug, Clone)]
pub struct ParticleRod {
    pub particle_a: ParticleId,
    pub particle_b: ParticleId,
    pub length: Real,
}

impl ParticleContactGenerator for ParticleRod {
    fn add_contacts(&self, contacts: &mut [ParticleContact], particles: &ParticleSet) -> usize {
        let position_a = particles[self.particle_a].position;
        let position_b = particles[self.particle_b].position;

        let curr_length = position_a.distance_to(position_b);
        if curr_length == self.length {
            return 0;
        }

        let normal = if curr_length > self.length {
            position_a.direction_to(position_b)
        } else {
            position_b.direction_to(position_a)
        };

        let penetration = (curr_length - self.length).abs();
        let restitution = 0.0;

        contacts[0] = ParticleContact {
            particle_a: self.particle_a,
            particle_b: Some(self.particle_b),
            data: ParticleContactData::new(restitution, normal, penetration),
        };

        1
    }
}

#[derive(Debug, Clone)]
pub struct ParticleCableConstraint {
    pub particle_a: ParticleId,
    pub anchor: Vec3,
    pub max_length: Real,
    pub restitution: Real,
}

impl ParticleContactGenerator for ParticleCableConstraint {
    fn add_contacts(&self, contacts: &mut [ParticleContact], particles: &ParticleSet) -> usize {
        let position_a = particles[self.particle_a].position;

        let length = position_a.distance_to(self.anchor);
        if length < self.max_length {
            return 0;
        }

        let normal = position_a.direction_to(self.anchor);
        let penetration = length - self.max_length;

        contacts[0] = ParticleContact {
            particle_a: self.particle_a,
            particle_b: None,
            data: ParticleContactData::new(self.restitution, normal, penetration),
        };

        1
    }
}

#[derive(Debug, Clone)]
pub struct ParticleRodConstraint {
    pub particle_a: ParticleId,
    pub anchor: Vec3,
    pub length: Real,
}

impl ParticleContactGenerator for ParticleRodConstraint {
    fn add_contacts(&self, contacts: &mut [ParticleContact], particles: &ParticleSet) -> usize {
        let position_a = particles[self.particle_a].position;

        let curr_length = position_a.distance_to(self.anchor);
        if curr_length == self.length {
            return 0;
        }

        let normal = if curr_length > self.length {
            position_a.direction_to(self.anchor)
        } else {
            self.anchor.direction_to(position_a)
        };

        let penetration = (curr_length - self.length).abs();
        let restitution = 0.0;

        contacts[0] = ParticleContact {
            particle_a: self.particle_a,
            particle_b: None,
            data: ParticleContactData::new(restitution, normal, penetration),
        };

        1
    }
}

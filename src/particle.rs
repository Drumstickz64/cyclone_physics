use crate::{math::vector::Vec3, precision::Real};

#[derive(Debug, Clone)]
pub struct Particle {
    pub damping: Real,
    pub inverse_mass: Real,

    position: Vec3,
    velocity: Vec3,
    acceleration: Vec3,
}

impl Particle {
    pub fn new(position: Vec3, mass: Real, damping: Real) -> Self {
        assert_ne!(mass, 0.0, "Particles can't have zero mass");
        let inverse_mass = if mass == Real::INFINITY {
            0.0
        } else {
            mass.recip()
        };

        Self {
            position,
            velocity: Vec3::ZERO,
            acceleration: Vec3::ZERO,
            damping,
            inverse_mass,
        }
    }

    pub fn integrate(&mut self, duration: Real) {
        if self.inverse_mass <= 0.0 {
            return;
        }

        assert_ne!(duration, 0.0);

        self.position += self.velocity * duration;

        let resulting_acc = self.acceleration;

        self.velocity = self.velocity * self.damping.powf(duration) + resulting_acc * duration;
    }

    pub fn get_mass(&self) -> Real {
        if self.inverse_mass == 0.0 {
            return Real::MAX;
        }

        self.inverse_mass.recip()
    }

    pub fn set_mass(&mut self, mass: Real) {
        assert_ne!(mass, 0.0, "Particles can't have zero mass");
        self.inverse_mass = mass.recip();
    }

    pub fn get_kinetic_energy(&self) -> Real {
        if self.inverse_mass == 0.0 {
            return Real::MAX;
        }

        0.5 * self.inverse_mass.recip() * self.velocity.squared_magnitude()
    }
}

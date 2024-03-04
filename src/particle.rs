use crate::{math::vector::Vec3, precision::Real};

#[derive(Debug, Clone)]
pub struct Particle {
    pub position: Vec3,
    pub velocity: Vec3,
    pub acceleration: Vec3,
    pub damping: Real,
    pub inverse_mass: Real,
}

impl Particle {
    pub fn integrate(&mut self, duration: Real) {
        if self.inverse_mass <= 0.0 {
            return;
        }

        assert_ne!(duration, 0.0);

        self.position += self.velocity * duration;

        let resulting_acc = self.acceleration;

        self.velocity = self.velocity * self.damping.powf(duration) + resulting_acc * duration;
    }

    pub fn mass(&self) -> Real {
        if self.inverse_mass == 0.0 {
            return Real::MAX;
        }

        self.inverse_mass.recip()
    }

    pub fn set_mass(&mut self, mass: Real) {
        assert_ne!(mass, 0.0, "Particles can't have zero mass");
        self.inverse_mass = mass.recip();
    }
}

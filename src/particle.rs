use crate::{math::vector::Vec3, precision::Real};

#[derive(Debug, Clone)]
pub struct Particle {
    position: Vec3,
    velocity: Vec3,
    acceleration: Vec3,
    damping: Real,
    inverse_mass: Real,
    force_accumelator: Vec3,
}

impl Particle {
    pub fn new(mass: Real) -> Self {
        assert_ne!(mass, 0.0, "Particles can't have zero mass");
        let inverse_mass = if mass == Real::INFINITY {
            0.0
        } else {
            mass.recip()
        };

        Self {
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            acceleration: Vec3::ZERO,
            damping: 0.99,
            inverse_mass,
            force_accumelator: Vec3::ZERO,
        }
    }

    pub fn with_position(mut self, position: Vec3) -> Self {
        self.position = position;
        self
    }

    pub fn with_velocity(mut self, velocity: Vec3) -> Self {
        self.velocity = velocity;
        self
    }

    pub fn with_acceleration(mut self, acceleration: Vec3) -> Self {
        self.acceleration = acceleration;
        self
    }

    pub fn with_damping(mut self, damping: Real) -> Self {
        self.damping = damping;
        self
    }

    pub fn with_mass(mut self, mass: Real) -> Self {
        self.set_mass(mass);
        self
    }

    pub fn integrate(&mut self, duration: Real) {
        if self.inverse_mass <= 0.0 {
            return;
        }

        assert_ne!(duration, 0.0);

        self.position += self.velocity * duration;

        let resulting_acc = self.acceleration;

        self.velocity = self.velocity * self.damping.powf(duration) + resulting_acc * duration;

        self.clear_accumulator();
    }

    pub fn kinetic_energy(&self) -> Real {
        if self.inverse_mass == 0.0 {
            return Real::MAX;
        }

        0.5 * self.inverse_mass.recip() * self.velocity.squared_magnitude()
    }

    pub fn position(&self) -> Vec3 {
        self.position
    }

    pub fn set_position(&mut self, position: Vec3) {
        self.position = position;
    }

    pub fn velocity(&self) -> Vec3 {
        self.velocity
    }

    pub fn set_velocity(&mut self, velocity: Vec3) {
        self.velocity = velocity;
    }

    pub fn acceleration(&self) -> Vec3 {
        self.acceleration
    }

    pub fn set_acceleration(&mut self, acceleration: Vec3) {
        self.acceleration = acceleration;
    }

    pub fn damping(&self) -> f32 {
        self.damping
    }

    pub fn set_damping(&mut self, damping: Real) {
        self.damping = damping;
    }

    pub fn inverse_mass(&self) -> f32 {
        self.inverse_mass
    }

    pub fn set_inverse_mass(&mut self, inverse_mass: Real) {
        self.inverse_mass = inverse_mass;
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

    pub fn clear_accumulator(&mut self) {
        self.force_accumelator = Vec3::ZERO;
    }
}

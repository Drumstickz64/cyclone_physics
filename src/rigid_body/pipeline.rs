use crate::precision::Real;

use super::RigidBodySet;

pub struct RigidBodyPipeline;

impl RigidBodyPipeline {
    pub fn new() -> Self {
        Self
    }

    pub fn start_frame(&mut self, bodies: &mut RigidBodySet) {
        for body in bodies.bodies_mut() {
            body.clear_accumelators();
            body.update_derived_data();
        }
    }

    pub fn step(&mut self, bodies: &mut RigidBodySet, duration: Real) {
        self.integrate(bodies, duration);
    }

    pub fn integrate(&mut self, bodies: &mut RigidBodySet, duration: Real) {
        for body in bodies.bodies_mut() {
            body.integrate(duration);
        }
    }
}

impl Default for RigidBodyPipeline {
    fn default() -> Self {
        Self::new()
    }
}

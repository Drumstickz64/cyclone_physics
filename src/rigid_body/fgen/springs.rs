use crate::{
    precision::Real,
    rigid_body::{RigidBodyId, RigidBodySet},
    Vec3,
};

#[derive(Debug, Clone)]
pub struct Spring {
    pub body_a: RigidBodyId,
    pub body_b: RigidBodyId,
    pub connection_point_a: Vec3,
    pub connection_point_b: Vec3,
    pub spring_constant: Real,
    pub rest_length: Real,
}

impl Spring {
    pub fn add_forces(&self, bodies: &mut RigidBodySet) {
        let [body_a, body_b] = bodies.get_disjoint_mut([self.body_a, self.body_b]).unwrap();
        let connect_point_a_ws = body_a.get_point_in_world_space(self.connection_point_a);
        let connect_point_b_ws = body_b.get_point_in_world_space(self.connection_point_b);

        let delta = connect_point_b_ws - connect_point_a_ws;
        let direction = delta.normalized();

        // Hook's law
        let force = self.spring_constant * (delta.magnitude() - self.rest_length) * -direction;

        body_a.add_force_at_point(force, connect_point_a_ws);
        body_b.add_force_at_point(-force, connect_point_b_ws);
    }
}

#[derive(Debug, Clone)]
pub struct AnchoredSpring {
    pub target: RigidBodyId,
    pub anchor: Vec3,
    pub connection_point: Vec3,
    pub spring_constant: Real,
    pub rest_length: Real,
}

impl AnchoredSpring {
    pub fn add_forces(&self, bodies: &mut RigidBodySet) {
        let body = &mut bodies[self.target];
        let connect_point_ws = body.get_point_in_world_space(self.connection_point);

        let delta = connect_point_ws - self.anchor;

        // Hook's law
        let force =
            self.spring_constant * (delta.magnitude() - self.rest_length) * -delta.normalized();

        body.add_force_at_point(force, connect_point_ws);
    }
}

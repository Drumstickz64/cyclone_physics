use std::collections::HashSet;

use cyclone_physics::{
    precision::Real,
    rigid_body::{
        collide_broad::{BoundingSphere, Bvh},
        RigidBody, RigidBodyId, RigidBodySet,
    },
    Vec3,
};
use macroquad::prelude::{Vec3 as MVec3, *};

const INITIAL_POSITION1: Vec3 = Vec3::new(-5.0, 0.0, 0.0);
const INITIAL_POSITION2: Vec3 = Vec3::new(5.0, 0.0, 0.0);
const INITIAL_POSITION3: Vec3 = Vec3::new(0.0, 0.0, 0.0);
const RADIUS: Real = 1.0;

#[macroquad::main("Broad Collision Test")]
async fn main() {
    set_camera(&Camera3D {
        position: vec3(0.0, 5.0, 15.0),
        up: MVec3::Y,
        ..Default::default()
    });

    let mut bodies = RigidBodySet::new();
    let body1 = bodies.insert(RigidBody::new(1.0).with_position(INITIAL_POSITION1));
    let body2 = bodies.insert(RigidBody::new(1.0).with_position(INITIAL_POSITION2));
    let body3 = bodies.insert(RigidBody::new(1.0).with_position(INITIAL_POSITION3));

    // debug_construct_bvh(&bodies);
    let mut bvh = construct_bvh(&bodies);

    loop {
        if is_key_down(KeyCode::A) {
            bodies[body1].position += Vec3::NEG_X * 0.2;
        } else if is_key_down(KeyCode::D) {
            bodies[body1].position += Vec3::X * 0.2;
        }

        if is_key_down(KeyCode::Left) {
            bodies[body2].position += Vec3::NEG_X * 0.2;
        } else if is_key_down(KeyCode::Right) {
            bodies[body2].position += Vec3::X * 0.2;
        }

        if is_key_pressed(KeyCode::Space) {
            bodies.remove(body3);
            bvh.remove_body(body3);
        }

        bvh.update(&bodies);
        let mut contacts = Vec::new();
        bvh.generate_potential_contacts(&mut contacts);

        if is_key_pressed(KeyCode::P) {
            dbg!(&bvh);
        }

        let in_contact: HashSet<RigidBodyId> = contacts
            .into_iter()
            .flat_map(|c| [c.body_a, c.body_b])
            .collect();

        clear_background(LIGHTGRAY);

        for (id, body) in bodies.iter() {
            let color = if in_contact.contains(&id) { RED } else { GREEN };
            draw_sphere(tovec(body.position), RADIUS, None, color);
        }

        next_frame().await
    }
}

fn construct_bvh(bodies: &RigidBodySet) -> Bvh<BoundingSphere> {
    let mut iterator = bodies.iter();
    let (first_id, first) = iterator.next().unwrap();
    let mut bvh = Bvh::new(first_id, BoundingSphere::new(first.position, 1.0));
    for (id, body) in iterator {
        bvh.insert(id, BoundingSphere::new(body.position, 1.0));
    }

    bvh
}

fn tovec(v: Vec3) -> MVec3 {
    vec3(v.x, v.y, v.z)
}

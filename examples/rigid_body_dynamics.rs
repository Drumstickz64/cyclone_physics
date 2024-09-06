use cyclone_physics::{
    precision::Real,
    rigid_body::{PhysicsSystem, RigidBody, RigidBodySet},
    Mat3, Quat, Vec3,
};
use macroquad::prelude::{Vec3 as MVec3, *};

const CUBE_MASS: Real = 1.0;
const CUBE_SIZE: Real = 1.0;
const PROPEL_FORCE: Real = 5.0;
const BREAK_FORCE: Real = 5.0;
const TURN_TORQUE: Real = 5.0;
const PROPEL_POSITION_RELATIVE: Vec3 = Vec3::new(0.0, 0.0, 0.5);

#[macroquad::main("Rigid Body Dynamics")]
async fn main() {
    let camera = Camera3D {
        position: vec3(0.0, 2.5, 5.0),
        up: MVec3::Y,
        ..Default::default()
    };
    set_camera(&camera);

    let mut bodies = RigidBodySet::new();
    let mut system = PhysicsSystem::new();

    let cube = bodies.insert(
        RigidBody::new(CUBE_MASS).with_inertia_tensor(calc_inertia_tensor(CUBE_MASS, CUBE_SIZE)),
    );

    loop {
        system.start_frame(&mut bodies);

        if is_key_pressed(KeyCode::R) {
            bodies[cube].position = Vec3::ZERO;
            bodies[cube].orientation = Quat::IDENTITY;
            bodies[cube].velocity = Vec3::ZERO;
            bodies[cube].angular_velocity = Vec3::ZERO;
        }

        if is_key_down(KeyCode::A) || is_key_down(KeyCode::Left) {
            bodies[cube].add_torque(Vec3::Y * -TURN_TORQUE);
        } else if is_key_down(KeyCode::D) || is_key_down(KeyCode::Right) {
            bodies[cube].add_torque(Vec3::Y * TURN_TORQUE);
        }

        if is_key_down(KeyCode::Space) {
            let force = bodies[cube]
                .transform_matrix()
                .transform_direction(-PROPEL_POSITION_RELATIVE)
                * PROPEL_FORCE;
            bodies[cube].add_force_at_body_point(force, PROPEL_POSITION_RELATIVE);
        }

        if is_key_down(KeyCode::C) {
            let dir = -bodies[cube].velocity.normalized();
            bodies[cube].add_force(BREAK_FORCE * dir);
        }

        system.step(&mut bodies, get_frame_time());

        clear_background(LIGHTGRAY);

        draw_body(&bodies[cube], CUBE_SIZE);

        next_frame().await;
    }
}

fn calc_inertia_tensor(mass: Real, size: Real) -> Mat3 {
    let moment = 1.0 / 12.0 * mass * (2.0 * size * size);
    Mat3::new([moment, 0.0, 0.0, 0.0, moment, 0.0, 0.0, 0.0, moment])
}

fn convert_vec3(v: Vec3) -> MVec3 {
    vec3(v.x, v.y, v.z)
}

fn draw_body(body: &RigidBody, size: Real) {
    let half_size = size / 2.0;
    let vertices = [
        Vec3::new(-half_size, -half_size, -half_size),
        Vec3::new(-half_size, -half_size, half_size),
        Vec3::new(-half_size, half_size, -half_size),
        Vec3::new(-half_size, half_size, half_size),
        Vec3::new(half_size, -half_size, -half_size),
        Vec3::new(half_size, -half_size, half_size),
        Vec3::new(half_size, half_size, -half_size),
        Vec3::new(half_size, half_size, half_size),
    ];

    for vertex in vertices.iter().copied() {
        let vertex_ws = body.get_point_in_world_space(vertex);
        draw_sphere(convert_vec3(vertex_ws), 0.05, None, RED);
    }

    draw_sphere(
        convert_vec3(body.get_point_in_world_space(PROPEL_POSITION_RELATIVE)),
        0.05,
        None,
        SKYBLUE,
    );
}

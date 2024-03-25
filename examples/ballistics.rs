use cyclone_physics::{particle::Particle, Vec3};
use macroquad::prelude::{Vec3 as MVec3, *};

const INITIAL_POS: Vec3 = Vec3::new(0.0, 1.5, 0.0);

#[macroquad::main("Ballistics")]
async fn main() {
    let mut particle = make_particle(ShotType::Pistol);

    set_camera(&Camera3D {
        position: vec3(-25.0, 8.0, 5.0),
        up: MVec3::Y,
        target: vec3(0.0, 5.0, 22.0),
        ..Default::default()
    });

    loop {
        clear_background(LIGHTGRAY);

        if is_key_pressed(KeyCode::Q) {
            particle = make_particle(ShotType::Pistol);
        } else if is_key_pressed(KeyCode::W) {
            particle = make_particle(ShotType::Artillery);
        } else if is_key_pressed(KeyCode::E) {
            particle = make_particle(ShotType::FireBall);
        } else if is_key_pressed(KeyCode::R) {
            particle = make_particle(ShotType::Laser);
        }

        particle.integrate(get_frame_time());

        draw_sphere(
            vec3(
                particle.position.x,
                particle.position.y,
                particle.position.z,
            ),
            1.0,
            None,
            RED,
        );

        next_frame().await
    }
}

#[derive(Debug, Clone, Copy)]
enum ShotType {
    Pistol,
    Artillery,
    FireBall,
    Laser,
}

fn make_particle(shot_type: ShotType) -> Particle {
    match shot_type {
        ShotType::Pistol => Particle::new(2.0)
            .with_position(INITIAL_POS)
            .with_velocity(Vec3::new(0.0, 0.0, 35.0))
            .with_acceleration(Vec3::new(0.0, -1.0, 0.0))
            .with_damping(0.99),

        ShotType::Artillery => Particle::new(200.0)
            .with_position(INITIAL_POS)
            .with_velocity(Vec3::new(0.0, 30.0, 40.0))
            .with_acceleration(Vec3::new(0.0, -20.0, 0.0))
            .with_damping(0.99),

        ShotType::FireBall => Particle::new(1.0)
            .with_position(INITIAL_POS)
            .with_velocity(Vec3::new(0.0, 0.0, 10.0))
            .with_acceleration(Vec3::new(0.0, 0.6, 0.0))
            .with_damping(0.9),
        ShotType::Laser => Particle::new(0.1)
            .with_position(INITIAL_POS)
            .with_velocity(Vec3::new(0.0, 0.0, 100.0))
            .with_acceleration(Vec3::ZERO)
            .with_damping(0.99),
    }
}

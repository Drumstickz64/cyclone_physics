use cyclone_physics::{math::vector::Vec3 as CVec3, Particle};
use macroquad::prelude::*;

const INITIAL_POS: CVec3 = CVec3::new(0.0, 1.5, 0.0);

#[macroquad::main("Ballistics")]
async fn main() {
    let mut particle = make_particle(ShotType::Pistol);

    loop {
        clear_background(LIGHTGRAY);

        set_camera(&Camera3D {
            position: vec3(-25.0, 8.0, 5.0),
            up: Vec3::Y,
            target: vec3(0.0, 5.0, 22.0),
            ..Default::default()
        });

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
        ShotType::Pistol => Particle {
            position: INITIAL_POS,
            velocity: CVec3::new(0.0, 0.0, 35.0),
            acceleration: CVec3::new(0.0, -1.0, 0.0),
            damping: 0.99,
            inverse_mass: 1.0 / 2.0,
        },

        ShotType::Artillery => Particle {
            position: INITIAL_POS,
            velocity: CVec3::new(0.0, 30.0, 40.0),
            acceleration: CVec3::new(0.0, -20.0, 0.0),
            damping: 0.99,
            inverse_mass: 1.0 / 200.0,
        },

        ShotType::FireBall => Particle {
            position: INITIAL_POS,
            velocity: CVec3::new(0.0, 0.0, 10.0),
            acceleration: CVec3::new(0.0, 0.6, 0.0),
            damping: 0.9,
            inverse_mass: 1.0,
        },

        ShotType::Laser => Particle {
            position: INITIAL_POS,
            velocity: CVec3::new(0.0, 0.0, 100.0),
            acceleration: CVec3::ZERO,
            damping: 0.99,
            inverse_mass: 1.0 / 0.1,
        },
    }
}
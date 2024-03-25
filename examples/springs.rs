use cyclone_physics::{
    particle::{
        fgen::{ParticleAnchoredSpring, ParticleForceGenerator},
        Particle, ParticlePipeline, ParticleSet,
    },
    Vec3,
};

use macroquad::prelude::*;

const PLAYER_MASS: f32 = 10.0;
const GRAVITY: Vec3 = Vec3::new(0.0, 250.0, 0.0);
const PLAYER_DAMPING: f32 = 0.75;
const SPRING_CONSTANT: f32 = 25.0;
const SPRING_REST_LENGTH: f32 = 150.0;
const SPRING_THICKNESS: f32 = 3.0;
const MOUSE_PARTICLE_RADIUS: f32 = 25.0;
const PLAYER_PARTICLE_RADIUS: f32 = 35.0;
const MOUSE_PARTICLE_COLOR: Color = RED;
const PLAYER_PARTICLE_COLOR: Color = BLUE;
const SPRING_COLOR: Color = GREEN;

#[macroquad::main("Springs")]
async fn main() {
    let mut particles = ParticleSet::new();

    let player_particle = particles.insert(
        Particle::new(PLAYER_MASS)
            .with_position(initial_player_pos())
            .with_acceleration(GRAVITY)
            .with_damping(PLAYER_DAMPING),
    );

    let mut spring = ParticleAnchoredSpring {
        target: player_particle,
        anchor: mouse_pos_vec(),
        spring_constant: SPRING_CONSTANT,
        rest_length: SPRING_REST_LENGTH,
    };

    let mut pipeline = ParticlePipeline::new(1024, 0);

    loop {
        clear_background(BLACK);

        let duration = get_frame_time();
        let mouse_pos = mouse_pos_vec();

        pipeline.start_frame(&mut particles);

        spring.anchor = mouse_pos;
        spring.update_forces(&mut particles, duration);

        pipeline.step(&mut particles, duration);

        let player_pos = particles[player_particle].position;

        draw_line(
            player_pos.x,
            player_pos.y,
            mouse_pos.x,
            mouse_pos.y,
            SPRING_THICKNESS,
            SPRING_COLOR,
        );

        draw_circle(
            mouse_pos.x,
            mouse_pos.y,
            MOUSE_PARTICLE_RADIUS,
            MOUSE_PARTICLE_COLOR,
        );

        draw_circle(
            player_pos.x,
            player_pos.y,
            PLAYER_PARTICLE_RADIUS,
            PLAYER_PARTICLE_COLOR,
        );

        next_frame().await;
    }
}

fn initial_player_pos() -> Vec3 {
    Vec3::new(screen_width() / 2.0, screen_height() / 1.5, 0.0)
}

fn mouse_pos_vec() -> Vec3 {
    let (mouse_x, mouse_y) = mouse_position();

    Vec3::new(mouse_x, mouse_y, 0.0)
}

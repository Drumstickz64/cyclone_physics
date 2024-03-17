use cyclone_physics::{math::vector::Vec3, particle::Particle, precision::Real};
use macroquad::prelude::{Vec3 as MVec3, *};

const MAX_FIREWORKS: usize = 16384;
const SIZE: Real = 1.5;

#[macroquad::main("Fireworks Demo")]
async fn main() {
    let mut state = State::new();
    let rules = firework_rules();

    set_camera(&Camera3D {
        position: MVec3::new(0.0, 4.0, 25.0),
        target: MVec3::new(0.0, 4.0, 0.0),
        up: MVec3::Y,
        ..Default::default()
    });

    loop {
        for key in get_keys_down().iter() {
            match key {
                KeyCode::Key0 => create_firework(
                    &mut state,
                    &rules[rand::gen_range(0, 9)],
                    random_initial_position(),
                    Vec3::ZERO,
                ),
                KeyCode::Key1 => {
                    create_firework(&mut state, &rules[0], random_initial_position(), Vec3::ZERO)
                }
                KeyCode::Key2 => {
                    create_firework(&mut state, &rules[1], random_initial_position(), Vec3::ZERO)
                }
                KeyCode::Key3 => {
                    create_firework(&mut state, &rules[2], random_initial_position(), Vec3::ZERO)
                }
                KeyCode::Key4 => {
                    create_firework(&mut state, &rules[3], random_initial_position(), Vec3::ZERO)
                }
                KeyCode::Key5 => {
                    create_firework(&mut state, &rules[4], random_initial_position(), Vec3::ZERO)
                }
                KeyCode::Key6 => {
                    create_firework(&mut state, &rules[5], random_initial_position(), Vec3::ZERO)
                }
                KeyCode::Key7 => {
                    create_firework(&mut state, &rules[6], random_initial_position(), Vec3::ZERO)
                }
                KeyCode::Key8 => {
                    create_firework(&mut state, &rules[7], random_initial_position(), Vec3::ZERO)
                }
                KeyCode::Key9 => {
                    create_firework(&mut state, &rules[8], random_initial_position(), Vec3::ZERO)
                }
                _ => {}
            }
        }

        update(&mut state, &rules);

        draw(&state);

        next_frame().await
    }
}

fn create_firework(
    state: &mut State,
    rule: &FireworkRule,
    initial_position: Vec3,
    initial_velocity: Vec3,
) {
    state.fireworks[state.next_firework] = rule.create(initial_position, initial_velocity);
    state.next_firework = (state.next_firework + 1) % MAX_FIREWORKS;
}

fn update(state: &mut State, rules: &[FireworkRule]) {
    let duration = get_frame_time();
    if duration <= 0.0 {
        return;
    }

    for i in 0..state.fireworks.len() {
        let firework = &mut state.fireworks[i];
        if firework.kind == 0 {
            continue;
        }

        let has_expired = firework.update(duration);
        if has_expired {
            let initial_position = firework.particle.position();
            let initial_velocity = firework.particle.velocity();

            let payloads = &rules[state.fireworks[i].kind - 1].payloads;
            state.fireworks[i].kind = 0;

            for payload in payloads.iter() {
                for _ in 0..payload.count {
                    let rule = &rules[payload.kind - 1];
                    create_firework(state, rule, initial_position, initial_velocity);
                }
            }
        }
    }
}

fn draw(state: &State) {
    clear_background(BLACK);

    for firework in state.fireworks.iter() {
        let color = match firework.kind {
            0 => continue,
            1 => Color::new(1.0, 0.0, 0.0, 1.0),
            2 => Color::new(1.0, 0.5, 0.0, 1.0),
            3 => Color::new(1.0, 1.0, 0.0, 1.0),
            4 => Color::new(0.0, 1.0, 0.0, 1.0),
            5 => Color::new(0.0, 1.0, 1.0, 1.0),
            6 => Color::new(0.4, 0.4, 1.0, 1.0),
            7 => Color::new(1.0, 0.0, 1.0, 1.0),
            8 => Color::new(1.0, 1.0, 1.0, 1.0),
            9 => Color::new(1.0, 0.5, 0.5, 1.0),
            _ => unreachable!(),
        };

        let pos = firework.particle.position();
        draw_cube(vec_to_mvec(pos), MVec3::new(SIZE, SIZE, 0.01), None, color)
    }
}

type ParticleKind = usize;

struct State {
    fireworks: Vec<Firework>,
    next_firework: usize,
}

impl State {
    pub fn new() -> Self {
        Self {
            next_firework: 0,
            fireworks: vec![Firework::default(); MAX_FIREWORKS],
        }
    }
}

#[derive(Clone)]
struct Firework {
    particle: Particle,
    kind: ParticleKind,
    age: Real,
}

impl Default for Firework {
    fn default() -> Self {
        Self {
            particle: Particle::new(1.0),
            kind: 0,
            age: 0.0,
        }
    }
}

impl Firework {
    pub fn update(&mut self, duration: Real) -> bool {
        self.particle.integrate(duration);

        self.age -= duration;

        self.age < 0.0 || self.particle.position().y < 0.0
    }
}

struct FireworkRule {
    kind: ParticleKind,
    min_age: Real,
    max_age: Real,
    min_velocity: Vec3,
    max_velocity: Vec3,
    damping: Real,
    payloads: Vec<Payload>,
}

impl FireworkRule {
    pub fn create(&self, initial_position: Vec3, initial_velocity: Vec3) -> Firework {
        let age = rand::gen_range(self.min_age, self.max_age);
        let position = initial_position;
        let velocity = initial_velocity + random_vector(self.min_velocity, self.max_velocity);

        Firework {
            particle: Particle::new(1.0)
                .with_position(position)
                .with_velocity(velocity)
                .with_acceleration(cyclone_physics::consts::GRAVITY)
                .with_damping(self.damping),

            kind: self.kind,
            age,
        }
    }
}

struct Payload {
    kind: ParticleKind,
    count: u32,
}

fn firework_rules() -> [FireworkRule; 9] {
    [
        FireworkRule {
            kind: 1,
            min_age: 0.5,
            max_age: 1.4,
            min_velocity: Vec3::new(-5.0, 25.0, -5.0),
            max_velocity: Vec3::new(5.0, 28.0, 5.0),
            damping: 0.1,
            payloads: vec![Payload { kind: 3, count: 5 }, Payload { kind: 5, count: 5 }],
        },
        FireworkRule {
            kind: 2,
            min_age: 0.5,
            max_age: 1.0,
            min_velocity: Vec3::new(-5.0, 10.0, -5.0),
            max_velocity: Vec3::new(5.0, 20.0, 5.0),
            damping: 0.8,
            payloads: vec![Payload { kind: 4, count: 2 }],
        },
        FireworkRule {
            kind: 3,
            min_age: 0.5,
            max_age: 1.5,
            min_velocity: Vec3::new(-5.0, -5.0, -5.0),
            max_velocity: Vec3::new(5.0, 5.0, 5.0),
            damping: 0.1,
            payloads: Vec::new(),
        },
        FireworkRule {
            kind: 4,
            min_age: 0.25,
            max_age: 0.5,
            min_velocity: Vec3::new(-20.0, 5.0, -5.0),
            max_velocity: Vec3::new(20.0, 5.0, 5.0),
            damping: 0.2,
            payloads: Vec::new(),
        },
        FireworkRule {
            kind: 5,
            min_age: 0.5,
            max_age: 1.0,
            min_velocity: Vec3::new(-20.0, 2.0, -5.0),
            max_velocity: Vec3::new(20.0, 18.0, 5.0),
            damping: 0.01,
            payloads: vec![Payload { kind: 3, count: 5 }],
        },
        FireworkRule {
            kind: 6,
            min_age: 3.0,
            max_age: 5.0,
            min_velocity: Vec3::new(-5.0, -5.0, -5.0),
            max_velocity: Vec3::new(5.0, 10.0, 5.0),
            damping: 0.95,
            payloads: Vec::new(),
        },
        FireworkRule {
            kind: 7,
            min_age: 4.0,
            max_age: 5.0,
            min_velocity: Vec3::new(-5.0, 50.0, -5.0),
            max_velocity: Vec3::new(5.0, 60.0, 5.0),
            damping: 0.01,
            payloads: vec![Payload { kind: 8, count: 10 }],
        },
        FireworkRule {
            kind: 8,
            min_age: 0.25,
            max_age: 0.5,
            min_velocity: Vec3::NEG_ONE,
            max_velocity: Vec3::ONE,
            damping: 0.01,
            payloads: Vec::new(),
        },
        FireworkRule {
            kind: 9,
            min_age: 3.0,
            max_age: 5.0,
            min_velocity: Vec3::new(-15.0, 10.0, -5.0),
            max_velocity: Vec3::new(15.0, 15.0, 5.0),
            damping: 0.95,
            payloads: Vec::new(),
        },
    ]
}

fn random_vector(start: Vec3, end: Vec3) -> Vec3 {
    Vec3::new(
        rand::gen_range(start.x, end.x),
        rand::gen_range(start.y, end.y),
        rand::gen_range(start.z, end.z),
    )
}

fn random_initial_position() -> Vec3 {
    let x = rand::gen_range(-3, 3);

    Vec3::new(5.0 * x as Real, 0.0, 0.0)
}

fn vec_to_mvec(v: Vec3) -> MVec3 {
    MVec3::new(v.x, v.y, v.z)
}

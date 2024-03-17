use std::ops::{Index, IndexMut};

use slotmap::{new_key_type, SlotMap};

use crate::{
    math::vector::Vec3,
    particle::{ParticleHandle, ParticleSet},
    precision::Real,
};

pub trait ParticleForceGenerator {
    fn update_force(
        &mut self,
        particles: &mut ParticleSet,
        particle: ParticleHandle,
        duration: Real,
    );
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ParticleForceRegistration {
    pub particle: ParticleHandle,
    pub fg: ForceGeneratorHandle,
}

pub struct ParticleSpring {
    pub other: ParticleHandle,
    pub spring_constant: Real,
    pub rest_length: Real,
}

impl ParticleForceGenerator for ParticleSpring {
    fn update_force(
        &mut self,
        particles: &mut ParticleSet,
        particle_key: ParticleHandle,
        _duration: Real,
    ) {
        let other_pos = particles[self.other].position();
        let particle = &mut particles[particle_key];
        let delta = particle.position() - other_pos;

        let force =
            -self.spring_constant * (delta.magnitude() - self.rest_length) * delta.normalized();

        particle.add_force(force);
    }
}

pub struct ParticleAnchoredSpring {
    pub anchor: Vec3,
    pub spring_constant: Real,
    pub rest_length: Real,
}

impl ParticleForceGenerator for ParticleAnchoredSpring {
    fn update_force(
        &mut self,
        particles: &mut ParticleSet,
        particle_key: ParticleHandle,
        _duration: Real,
    ) {
        let particle = &mut particles[particle_key];
        let delta = particle.position() - self.anchor;

        let force =
            -self.spring_constant * (delta.magnitude() - self.rest_length) * delta.normalized();

        particle.add_force(force);
    }
}

pub struct ParticleBungee {
    pub other: ParticleHandle,
    pub spring_constant: Real,
    pub rest_length: Real,
}

impl ParticleForceGenerator for ParticleBungee {
    fn update_force(
        &mut self,
        particles: &mut ParticleSet,
        particle_key: ParticleHandle,
        _duration: Real,
    ) {
        let other_pos = particles[self.other].position();
        let particle = &mut particles[particle_key];
        let delta = particle.position() - other_pos;
        let delta_mag = delta.magnitude();

        if delta_mag <= self.rest_length {
            return;
        }

        let force = -self.spring_constant * (-self.rest_length) * delta.normalized();

        particle.add_force(force);
    }
}

pub struct ParticleBuoyancy {
    pub max_depth: Real,
    pub volume: Real,
    pub liquid_height: Real,
    pub liquid_density: Real,
}

impl ParticleBuoyancy {
    pub fn new(max_depth: Real, volume: Real, liquid_height: Real) -> Self {
        Self {
            max_depth,
            volume,
            liquid_height,
            liquid_density: 1000.0,
        }
    }

    pub fn with_liquid_density(mut self, liquid_density: Real) -> Self {
        self.liquid_density = liquid_density;
        self
    }
}

impl ParticleForceGenerator for ParticleBuoyancy {
    fn update_force(
        &mut self,
        particles: &mut ParticleSet,
        particle: ParticleHandle,
        _duration: Real,
    ) {
        let particle = &mut particles[particle];
        let depth = particle.position().y;

        if depth >= self.liquid_height + self.max_depth {
            return;
        }

        let buoyancy_force = if depth <= self.liquid_height - self.max_depth {
            self.volume * self.liquid_density
        } else {
            let degree_of_submersion =
                (depth - self.liquid_height - self.max_depth) / 2.0 * self.max_depth;
            degree_of_submersion * self.volume * self.liquid_density
        };

        particle.add_force(Vec3::Y * buoyancy_force);
    }
}

new_key_type! {
    pub struct ForceGeneratorHandle;
}

pub struct ForceGeneratorSet {
    inner: SlotMap<ForceGeneratorHandle, Box<dyn ParticleForceGenerator>>,
}

impl Default for ForceGeneratorSet {
    fn default() -> Self {
        Self::new()
    }
}

impl ForceGeneratorSet {
    pub fn new() -> Self {
        Self {
            inner: SlotMap::with_key(),
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            inner: SlotMap::with_capacity_and_key(capacity),
        }
    }

    pub fn insert(&mut self, value: Box<dyn ParticleForceGenerator>) -> ForceGeneratorHandle {
        self.inner.insert(value)
    }

    pub fn remove(&mut self, key: ForceGeneratorHandle) -> Option<Box<dyn ParticleForceGenerator>> {
        self.inner.remove(key)
    }

    pub fn clear(&mut self) {
        self.inner.clear()
    }

    pub fn len(&self) -> usize {
        self.inner.len()
    }

    pub fn get(&self, key: ForceGeneratorHandle) -> Option<&dyn ParticleForceGenerator> {
        self.inner.get(key).map(|fg| fg.as_ref())
    }

    pub fn get_mut(
        &mut self,
        key: ForceGeneratorHandle,
    ) -> Option<&mut Box<dyn ParticleForceGenerator>> {
        self.inner.get_mut(key)
    }

    pub fn capacity(&self) -> usize {
        self.inner.capacity()
    }

    pub fn force_generators(&self) -> ForceGenerators<'_> {
        ForceGenerators(self.inner.values())
    }

    pub fn force_generators_mut(&mut self) -> ForceGeneratorsMut<'_> {
        ForceGeneratorsMut(self.inner.values_mut())
    }

    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    pub fn handles(&self) -> Handles {
        Handles(self.inner.keys())
    }

    pub fn get_disjoint_mut<const N: usize>(
        &mut self,
        keys: [ForceGeneratorHandle; N],
    ) -> Option<[&mut Box<dyn ParticleForceGenerator>; N]> {
        self.inner.get_disjoint_mut(keys)
    }

    pub fn contains_handle(&self, key: ForceGeneratorHandle) -> bool {
        self.inner.contains_key(key)
    }

    pub fn iter(&self) -> Iter {
        Iter(self.inner.iter())
    }

    pub fn iter_mut(&mut self) -> IterMut {
        IterMut(self.inner.iter_mut())
    }

    pub fn reserve(&mut self, additional: usize) {
        self.inner.reserve(additional)
    }

    pub fn drain(&mut self) -> Drain {
        Drain(self.inner.drain())
    }
}

impl Index<ForceGeneratorHandle> for ForceGeneratorSet {
    type Output = dyn ParticleForceGenerator;

    fn index(&self, index: ForceGeneratorHandle) -> &Self::Output {
        self.inner[index].as_ref()
    }
}

impl IndexMut<ForceGeneratorHandle> for ForceGeneratorSet {
    fn index_mut(&mut self, index: ForceGeneratorHandle) -> &mut Self::Output {
        self.inner[index].as_mut()
    }
}

pub struct ForceGenerators<'a>(
    slotmap::basic::Values<'a, ForceGeneratorHandle, Box<dyn ParticleForceGenerator>>,
);

impl<'a> Iterator for ForceGenerators<'a> {
    type Item = &'a dyn ParticleForceGenerator;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next().map(|fg| fg.as_ref())
    }
}

impl<'a> ExactSizeIterator for ForceGenerators<'a> {}

pub struct ForceGeneratorsMut<'a>(
    slotmap::basic::ValuesMut<'a, ForceGeneratorHandle, Box<dyn ParticleForceGenerator>>,
);

impl<'a> Iterator for ForceGeneratorsMut<'a> {
    type Item = &'a mut Box<dyn ParticleForceGenerator>;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next()
    }
}

impl<'a> ExactSizeIterator for ForceGeneratorsMut<'a> {}

pub struct Iter<'a>(
    slotmap::basic::Iter<'a, ForceGeneratorHandle, Box<dyn ParticleForceGenerator>>,
);

impl<'a> Iterator for Iter<'a> {
    type Item = (ForceGeneratorHandle, &'a dyn ParticleForceGenerator);

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next().map(|(handle, fg)| (handle, fg.as_ref()))
    }
}

impl<'a> ExactSizeIterator for Iter<'a> {}

pub struct IterMut<'a>(
    slotmap::basic::IterMut<'a, ForceGeneratorHandle, Box<dyn ParticleForceGenerator>>,
);

impl<'a> Iterator for IterMut<'a> {
    type Item = (
        ForceGeneratorHandle,
        &'a mut Box<dyn ParticleForceGenerator>,
    );

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next()
    }
}

impl<'a> ExactSizeIterator for IterMut<'a> {}

pub struct Drain<'a>(
    slotmap::basic::Drain<'a, ForceGeneratorHandle, Box<dyn ParticleForceGenerator>>,
);

impl<'a> Iterator for Drain<'a> {
    type Item = (ForceGeneratorHandle, Box<dyn ParticleForceGenerator>);

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next()
    }
}

impl<'a> ExactSizeIterator for Drain<'a> {}

pub struct Handles<'a>(
    slotmap::basic::Keys<'a, ForceGeneratorHandle, Box<dyn ParticleForceGenerator>>,
);

impl<'a> Iterator for Handles<'a> {
    type Item = ForceGeneratorHandle;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next()
    }
}

impl<'a> ExactSizeIterator for Handles<'a> {}

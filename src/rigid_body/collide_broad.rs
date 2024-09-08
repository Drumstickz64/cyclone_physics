use slotmap::{new_key_type, SlotMap};

use crate::{consts::PI, precision::Real, Vec3};

use super::{RigidBodyId, RigidBodySet};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PotentialContact {
    pub body_a: RigidBodyId,
    pub body_b: RigidBodyId,
}

pub trait BoundingVolume: Clone {
    fn overlaps(&self, other: &Self) -> bool;
    fn size(&self) -> Real;
    fn new_enclosing(one: &Self, two: &Self) -> Self;
    fn get_growth(&self, new_volume: &Self) -> Real;
    fn set_position(&mut self, new_position: Vec3);
}

new_key_type! {
    pub struct BvhNodeId;
}

#[derive(Debug, Clone)]
pub struct Bvh<BoundingVolumeType: BoundingVolume> {
    nodes: SlotMap<BvhNodeId, BvhNode<BoundingVolumeType>>,
    root: BvhNodeId,
}

impl<BoundingVolumeType: BoundingVolume> Bvh<BoundingVolumeType> {
    pub fn new(root: RigidBodyId, volume: BoundingVolumeType) -> Self {
        let mut nodes = SlotMap::with_key();
        let root = nodes.insert(BvhNode::new_leaf(root, volume));
        Self { nodes, root }
    }

    pub fn generate_potential_contacts(&self, contacts: &mut Vec<PotentialContact>) {
        self.generate_potential_contacts_at(self.root, contacts);
    }

    fn generate_potential_contacts_at(&self, id: BvhNodeId, contacts: &mut Vec<PotentialContact>) {
        let BvhNodeData::Branch { left, right } = self.nodes[id].data else {
            return;
        };

        self.generate_potential_contacts_between(left, right, contacts);
        self.generate_potential_contacts_at(left, contacts);
        self.generate_potential_contacts_at(right, contacts);
    }

    fn generate_potential_contacts_between(
        &self,
        id_a: BvhNodeId,
        id_b: BvhNodeId,
        contacts: &mut Vec<PotentialContact>,
    ) {
        let node_a = &self.nodes[id_a];
        let node_b = &self.nodes[id_b];

        // Early out if we don’t overlap
        if !node_a.overlaps(node_b) {
            return;
        }

        match (node_a.data, node_b.data) {
            // If we’re both at leaf nodes, then we have a potential contact.
            (BvhNodeData::Leaf { body: self_body }, BvhNodeData::Leaf { body: other_body }) => {
                contacts.push(PotentialContact {
                    body_a: self_body,
                    body_b: other_body,
                });
            }
            // Determine which node to descend into.
            // If either is a leaf, then we descend the other.
            (BvhNodeData::Branch { left, right }, BvhNodeData::Leaf { .. }) => {
                self.generate_potential_contacts_between(left, id_b, contacts);
                self.generate_potential_contacts_between(right, id_b, contacts);
            }
            (BvhNodeData::Leaf { .. }, BvhNodeData::Branch { left, right }) => {
                self.generate_potential_contacts_between(left, id_a, contacts);
                self.generate_potential_contacts_between(right, id_a, contacts);
            }
            // If both are branches, then we use the one with the largest size.
            (
                BvhNodeData::Branch {
                    left: left_a,
                    right: right_a,
                },
                BvhNodeData::Branch {
                    left: left_b,
                    right: right_b,
                },
            ) => {
                if node_a.volume.size() >= node_b.volume.size() {
                    self.generate_potential_contacts_between(left_a, id_b, contacts);
                    self.generate_potential_contacts_between(right_a, id_b, contacts);
                } else {
                    self.generate_potential_contacts_between(left_b, id_a, contacts);
                    self.generate_potential_contacts_between(right_b, id_a, contacts);
                }
            }
        }
    }

    pub fn insert(&mut self, new_body: RigidBodyId, new_volume: BoundingVolumeType) -> BvhNodeId {
        self.insert_at(self.root, new_body, new_volume)
    }

    fn insert_at(
        &mut self,
        id: BvhNodeId,
        new_body: RigidBodyId,
        new_volume: BoundingVolumeType,
    ) -> BvhNodeId {
        match self.nodes[id].data {
            // If we are a leaf, then the only option is to spawn two
            // new children and place the new body in one.
            BvhNodeData::Leaf { .. } => {
                let clone_id = self
                    .nodes
                    .insert(self.nodes[id].clone().with_parent(Some(id)));

                let new_id = self
                    .nodes
                    .insert(BvhNode::new_leaf(new_body, new_volume).with_parent(Some(id)));

                self.nodes[id] = BvhNode::new_branch(
                    clone_id,
                    new_id,
                    &self.nodes[clone_id].volume,
                    &self.nodes[new_id].volume,
                )
                .with_parent(self.nodes[id].parent);

                new_id
            }
            // Otherwise, we need to work out which child gets to keep
            // the inserted body. We give it to whoever would grow the
            // least to incorporate it.
            BvhNodeData::Branch { left, right } => {
                if self.nodes[left].volume.get_growth(&new_volume)
                    < self.nodes[right].volume.get_growth(&new_volume)
                {
                    self.insert_at(left, new_body, new_volume)
                } else {
                    self.insert_at(right, new_body, new_volume)
                }
            }
        }
    }

    pub fn remove_node(&mut self, id: BvhNodeId) {
        // If we don't have a parent, then we ignore the sibling processing.
        if let Some(parent) = self.nodes[id].parent {
            // Find our sibling
            let BvhNodeData::Branch { left, right } = self.nodes[parent].data else {
                unreachable!("parent nodes must can't be leaves");
            };
            let sibling = if id == left { right } else { left };

            // Write its data to our parent
            self.nodes[parent].volume = self.nodes[sibling].volume.clone();
            self.nodes[parent].data = self.nodes[sibling].data;

            if parent == self.root {
                self.root = parent;
            }

            // Delete the sibling
            self.nodes.remove(sibling);

            // Recalculate the parent's bounding volume
            self.recalculate_bounding_volume(parent);
        }

        if let BvhNodeData::Branch { left, right } = self.nodes[id].data {
            // Delete our children, we remove their
            // parent data so we don't try to process their siblings
            // as they are deleted
            self.nodes[left].parent = None;
            self.remove_node(left);
            self.nodes[right].parent = None;
            self.remove_node(right);
        }

        self.nodes.remove(id);
    }

    pub fn remove_body(&mut self, body: RigidBodyId) -> Option<BvhNodeId> {
        let id = self.nodes.iter().find_map(|(id, node)| {
            let BvhNodeData::Leaf { body: node_body } = node.data else {
                return None;
            };

            if node_body != body {
                return None;
            }

            Some(id)
        })?;

        self.remove_node(id);
        Some(id)
    }

    pub fn update(&mut self, bodies: &RigidBodySet) {
        self.update_at(self.root, bodies);
    }

    fn update_at(&mut self, id: BvhNodeId, bodies: &RigidBodySet) {
        match self.nodes[id].data {
            BvhNodeData::Leaf { body } => self.nodes[id].volume.set_position(bodies[body].position),
            BvhNodeData::Branch { left, right } => {
                self.update_at(left, bodies);
                self.update_at(right, bodies);
                self.nodes[id].volume = BoundingVolumeType::new_enclosing(
                    &self.nodes[left].volume,
                    &self.nodes[right].volume,
                )
            }
        }
    }

    fn recalculate_bounding_volume(&mut self, id: BvhNodeId) {
        let BvhNodeData::Branch { left, right } = self.nodes[id].data else {
            return;
        };

        self.nodes[id].volume =
            BoundingVolumeType::new_enclosing(&self.nodes[left].volume, &self.nodes[right].volume);

        if let Some(parent) = self.nodes[id].parent {
            self.recalculate_bounding_volume(parent);
        }
    }
}

#[derive(Debug, Clone)]
struct BvhNode<BoundingVolumeType: BoundingVolume> {
    volume: BoundingVolumeType,
    parent: Option<BvhNodeId>,
    data: BvhNodeData,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum BvhNodeData {
    Leaf { body: RigidBodyId },
    Branch { left: BvhNodeId, right: BvhNodeId },
}

impl<BoundingVolumeType: BoundingVolume> BvhNode<BoundingVolumeType> {
    fn new_leaf(body: RigidBodyId, volume: BoundingVolumeType) -> Self {
        Self {
            volume,
            parent: None,
            data: BvhNodeData::Leaf { body },
        }
    }

    fn new_branch(
        left: BvhNodeId,
        right: BvhNodeId,
        left_volume: &BoundingVolumeType,
        right_volume: &BoundingVolumeType,
    ) -> Self {
        Self {
            volume: BoundingVolumeType::new_enclosing(left_volume, right_volume),
            parent: None,
            data: BvhNodeData::Branch { left, right },
        }
    }

    fn with_parent(mut self, parent: Option<BvhNodeId>) -> Self {
        self.parent = parent;
        self
    }

    fn overlaps(&self, other: &Self) -> bool {
        self.volume.overlaps(&other.volume)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundingSphere {
    center: Vec3,
    radius: Real,
}

impl BoundingSphere {
    pub fn new(center: Vec3, radius: Real) -> Self {
        Self { center, radius }
    }
}

impl BoundingVolume for BoundingSphere {
    fn overlaps(&self, other: &Self) -> bool {
        let sum_of_radii = self.radius + other.radius;
        self.center.distance_to_squared(other.center) < sum_of_radii * sum_of_radii
    }

    fn size(&self) -> Real {
        1.333333 * PI * self.radius * self.radius * self.radius
    }

    fn new_enclosing(one: &Self, two: &Self) -> Self {
        let center_offset = two.center - one.center;
        let distance_squared = center_offset.squared_magnitude();
        let radius_diff = two.radius - one.radius;

        // Check whether the larger sphere encloses the small one.
        // in which case we just return it
        if radius_diff * radius_diff >= distance_squared {
            return if one.radius > two.radius { *one } else { *two };
        }

        // Otherwise, we need to work with partially
        // overlapping spheres.
        let distance = distance_squared.sqrt();
        let radius = (distance + one.radius + two.radius) * 0.5;
        let mut center = one.center;
        if distance > 0.0 {
            center += center_offset * ((radius - one.radius) / distance);
        }

        Self { center, radius }
    }

    fn get_growth(&self, new_volume: &Self) -> Real {
        let new_sphere = Self::new_enclosing(self, new_volume);

        // We return a value proportional to the change in surface
        // area of the sphere.
        new_sphere.radius * new_sphere.radius - self.radius * self.radius
    }

    fn set_position(&mut self, new_position: Vec3) {
        self.center = new_position;
    }
}

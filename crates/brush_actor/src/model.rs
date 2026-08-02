use glam::Vec3;

use crate::{compile_brush, Brush, Material, Mesh};

#[repr(usize)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Bone {
    Pelvis = 0,
    Torso,
    Head,
    LeftUpperArm,
    LeftForearm,
    LeftHand,
    RightUpperArm,
    RightForearm,
    RightHand,
    LeftThigh,
    LeftShin,
    LeftAnkle,
    LeftFoot,
    RightThigh,
    RightShin,
    RightAnkle,
    RightFoot,
}

impl Bone {
    pub const COUNT: usize = 17;
}

#[derive(Clone, Copy, Debug)]
pub struct Morphology {
    pub shoulder_width: f32,
    pub torso_height: f32,
    pub head_height: f32,
    pub upper_arm_length: f32,
    pub forearm_length: f32,
    pub hand_length: f32,
    pub thigh_length: f32,
    pub shin_length: f32,
    pub ankle_length: f32,
    pub foot_length: f32,
    pub heel_offset: f32,
    pub ball_offset: f32,
    pub hunch: f32,
}

pub struct Part {
    pub name: &'static str,
    pub bone: Bone,
    pub mesh: Mesh,
}

pub struct ActorModel {
    pub name: &'static str,
    pub morphology: Morphology,
    pub parts: Vec<Part>,
}

impl ActorModel {
    pub fn marine() -> Self {
        let morphology = Morphology {
            shoulder_width: 0.66,
            torso_height: 0.64,
            head_height: 0.31,
            upper_arm_length: 0.38,
            forearm_length: 0.34,
            hand_length: 0.16,
            thigh_length: 0.55,
            shin_length: 0.48,
            ankle_length: 0.22,
            foot_length: 0.34,
            heel_offset: -0.027,
            ball_offset: 0.28,
            hunch: 0.03,
        };
        let mut parts = Vec::new();
        parts.push(part(
            "marine pelvis",
            Bone::Pelvis,
            vec![
                Brush::beveled_box(
                    Vec3::ZERO,
                    Vec3::new(0.25, 0.16, 0.16),
                    0.05,
                    Material::MarineCloth,
                ),
                Brush::cuboid(
                    Vec3::new(0.0, 0.08, 0.0),
                    Vec3::new(0.28, 0.045, 0.18),
                    Material::Metal,
                ),
            ],
        ));
        parts.push(part(
            "marine torso",
            Bone::Torso,
            vec![
                Brush::beveled_box(
                    Vec3::ZERO,
                    Vec3::new(0.32, 0.32, 0.20),
                    0.075,
                    Material::MarineArmor,
                ),
                Brush::beveled_box(
                    Vec3::new(0.0, 0.08, 0.19),
                    Vec3::new(0.23, 0.16, 0.045),
                    0.025,
                    Material::Metal,
                ),
                Brush::cuboid(
                    Vec3::new(0.0, -0.26, 0.0),
                    Vec3::new(0.22, 0.08, 0.15),
                    Material::MarineCloth,
                ),
            ],
        ));
        parts.push(part(
            "marine head",
            Bone::Head,
            vec![
                Brush::ellipsoid(
                    Vec3::new(0.0, -0.015, 0.0),
                    Vec3::new(0.13, 0.16, 0.12),
                    8,
                    Material::Skin,
                ),
                Brush::beveled_box(
                    Vec3::new(0.0, 0.10, -0.005),
                    Vec3::new(0.16, 0.075, 0.15),
                    0.035,
                    Material::MarineArmor,
                ),
                Brush::beveled_box(
                    Vec3::new(0.0, 0.045, 0.125),
                    Vec3::new(0.125, 0.035, 0.035),
                    0.012,
                    Material::Metal,
                ),
            ],
        ));
        add_limb_pair(
            &mut parts,
            Bone::LeftUpperArm,
            Bone::RightUpperArm,
            "marine upper arm",
            morphology.upper_arm_length,
            Vec3::new(0.115, morphology.upper_arm_length * 0.5, 0.12),
            Material::MarineArmor,
            0.035,
        );
        add_limb_pair(
            &mut parts,
            Bone::LeftForearm,
            Bone::RightForearm,
            "marine forearm",
            morphology.forearm_length,
            Vec3::new(0.095, morphology.forearm_length * 0.5, 0.10),
            Material::MarineCloth,
            0.025,
        );
        add_limb_pair(
            &mut parts,
            Bone::LeftHand,
            Bone::RightHand,
            "marine hand",
            morphology.hand_length,
            Vec3::new(0.09, morphology.hand_length * 0.5, 0.085),
            Material::Skin,
            0.025,
        );
        add_leg_pair(
            &mut parts,
            &morphology,
            Vec3::new(0.14, morphology.thigh_length * 0.5, 0.16),
            Vec3::new(0.12, morphology.shin_length * 0.5, 0.135),
            Vec3::new(0.13, morphology.ankle_length * 0.5, 0.15),
            Material::MarineCloth,
            Material::MarineArmor,
            Material::Leather,
        );
        add_foot_pair(
            &mut parts,
            &morphology,
            Vec3::new(0.15, 0.11, morphology.foot_length * 0.5),
            Material::Leather,
        );
        Self {
            name: "RANGER MARINE",
            morphology,
            parts,
        }
    }

    pub fn ogre() -> Self {
        let morphology = Morphology {
            shoulder_width: 1.32,
            torso_height: 1.08,
            head_height: 0.48,
            upper_arm_length: 0.70,
            forearm_length: 0.62,
            hand_length: 0.28,
            thigh_length: 0.88,
            shin_length: 0.74,
            ankle_length: 0.34,
            foot_length: 0.58,
            heel_offset: -0.046,
            ball_offset: 0.48,
            hunch: 0.26,
        };
        let mut parts = Vec::new();
        parts.push(part(
            "ogre pelvis",
            Bone::Pelvis,
            vec![
                Brush::ellipsoid(
                    Vec3::ZERO,
                    Vec3::new(0.52, 0.27, 0.40),
                    8,
                    Material::OgreSkin,
                ),
                Brush::beveled_box(
                    Vec3::new(0.0, -0.13, 0.18),
                    Vec3::new(0.48, 0.18, 0.10),
                    0.05,
                    Material::Leather,
                ),
            ],
        ));
        parts.push(part(
            "ogre torso",
            Bone::Torso,
            vec![
                Brush::ellipsoid(
                    Vec3::new(0.0, 0.10, -0.04),
                    Vec3::new(0.65, 0.55, 0.38),
                    10,
                    Material::OgreSkin,
                ),
                Brush::ellipsoid(
                    Vec3::new(0.0, -0.24, 0.30),
                    Vec3::new(0.62, 0.52, 0.53),
                    10,
                    Material::OgreBelly,
                ),
                Brush::beveled_box(
                    Vec3::new(0.0, 0.17, -0.30),
                    Vec3::new(0.50, 0.16, 0.10),
                    0.06,
                    Material::Leather,
                ),
            ],
        ));
        parts.push(part(
            "ogre head",
            Bone::Head,
            vec![
                Brush::ellipsoid(
                    Vec3::new(0.0, 0.02, 0.0),
                    Vec3::new(0.28, 0.25, 0.27),
                    8,
                    Material::OgreSkin,
                ),
                Brush::beveled_box(
                    Vec3::new(0.0, -0.12, 0.25),
                    Vec3::new(0.25, 0.10, 0.18),
                    0.04,
                    Material::OgreBelly,
                ),
                Brush::cuboid(
                    Vec3::new(0.0, 0.07, -0.24),
                    Vec3::new(0.24, 0.07, 0.06),
                    Material::Leather,
                ),
            ],
        ));
        add_limb_pair(
            &mut parts,
            Bone::LeftUpperArm,
            Bone::RightUpperArm,
            "ogre upper arm",
            morphology.upper_arm_length,
            Vec3::new(0.22, morphology.upper_arm_length * 0.5, 0.24),
            Material::OgreSkin,
            0.065,
        );
        add_limb_pair(
            &mut parts,
            Bone::LeftForearm,
            Bone::RightForearm,
            "ogre forearm",
            morphology.forearm_length,
            Vec3::new(0.25, morphology.forearm_length * 0.5, 0.26),
            Material::OgreSkin,
            0.07,
        );
        add_limb_pair(
            &mut parts,
            Bone::LeftHand,
            Bone::RightHand,
            "ogre fist",
            morphology.hand_length,
            Vec3::new(0.25, morphology.hand_length * 0.5, 0.25),
            Material::OgreBelly,
            0.065,
        );
        add_leg_pair(
            &mut parts,
            &morphology,
            Vec3::new(0.26, morphology.thigh_length * 0.5, 0.30),
            Vec3::new(0.22, morphology.shin_length * 0.5, 0.25),
            Vec3::new(0.24, morphology.ankle_length * 0.5, 0.28),
            Material::Leather,
            Material::OgreSkin,
            Material::Leather,
        );
        add_foot_pair(
            &mut parts,
            &morphology,
            Vec3::new(0.29, 0.18, morphology.foot_length * 0.5),
            Material::Leather,
        );
        Self {
            name: "BLOATED OGRE",
            morphology,
            parts,
        }
    }

    pub fn append_pose_mesh(&self, mesh: &mut Mesh, transforms: &[glam::Mat4; Bone::COUNT]) {
        for part in &self.parts {
            mesh.append_transformed(&part.mesh, transforms[part.bone as usize]);
        }
    }

    pub fn mesh_for_pose(&self, transforms: &[glam::Mat4; Bone::COUNT]) -> Mesh {
        let mut mesh = Mesh::default();
        self.append_pose_mesh(&mut mesh, transforms);
        mesh
    }
}

fn part(name: &'static str, bone: Bone, brushes: Vec<Brush>) -> Part {
    let mut mesh = Mesh::default();
    for brush in brushes {
        mesh.append(&compile_brush(&brush));
    }
    Part { name, bone, mesh }
}

#[allow(clippy::too_many_arguments)]
fn add_limb_pair(
    parts: &mut Vec<Part>,
    left: Bone,
    right: Bone,
    name: &'static str,
    length: f32,
    half_extents: Vec3,
    material: Material,
    bevel: f32,
) {
    let center = Vec3::new(0.0, -length * 0.5, 0.0);
    let make = |bone| {
        part(
            name,
            bone,
            vec![Brush::beveled_box(center, half_extents, bevel, material)],
        )
    };
    parts.push(make(left));
    parts.push(make(right));
}

#[allow(clippy::too_many_arguments)]
fn add_leg_pair(
    parts: &mut Vec<Part>,
    morphology: &Morphology,
    thigh_half: Vec3,
    shin_half: Vec3,
    ankle_half: Vec3,
    thigh_material: Material,
    shin_material: Material,
    ankle_material: Material,
) {
    add_limb_pair(
        parts,
        Bone::LeftThigh,
        Bone::RightThigh,
        "thigh",
        morphology.thigh_length,
        thigh_half,
        thigh_material,
        thigh_half.x * 0.30,
    );
    add_limb_pair(
        parts,
        Bone::LeftShin,
        Bone::RightShin,
        "shin",
        morphology.shin_length,
        shin_half,
        shin_material,
        shin_half.x * 0.28,
    );
    add_limb_pair(
        parts,
        Bone::LeftAnkle,
        Bone::RightAnkle,
        "ankle",
        morphology.ankle_length,
        ankle_half,
        ankle_material,
        ankle_half.x * 0.25,
    );
}

fn add_foot_pair(
    parts: &mut Vec<Part>,
    morphology: &Morphology,
    half_extents: Vec3,
    material: Material,
) {
    let center = Vec3::new(0.0, half_extents.y, morphology.foot_length * 0.42);
    for bone in [Bone::LeftFoot, Bone::RightFoot] {
        parts.push(part(
            "foot",
            bone,
            vec![Brush::beveled_box(
                center,
                half_extents,
                half_extents.y * 0.35,
                material,
            )],
        ));
    }
}

use brush_actor::{
    compile_brush, pose_walker, ActorModel, Bone, Brush, Material, PoseDriver, PosePalette,
};
use glam::{Quat, Vec3};
use walker2::{GroundQuery, KneeBend, Walker, WalkerCommand, WalkerSpec};

struct Flat;
impl GroundQuery for Flat {
    fn height_at(&self, _x: f32, _z: f32) -> f32 {
        0.0
    }
}

#[test]
fn convex_brush_compiles_inside_every_plane() {
    let brush = Brush::beveled_box(
        Vec3::new(0.4, -0.2, 0.8),
        Vec3::new(1.0, 0.7, 0.5),
        0.12,
        Material::MarineArmor,
    );
    let mesh = compile_brush(&brush);
    assert!(!mesh.vertices.is_empty());
    assert!(!mesh.indices.is_empty());
    assert_eq!(mesh.indices.len() % 3, 0);
    for vertex in &mesh.vertices {
        assert!(vertex.position.is_finite());
        assert!(vertex.normal.is_normalized());
        for plane in &brush.planes {
            assert!(
                plane.normal.dot(vertex.position) <= plane.distance + 1.0e-3,
                "vertex {:?} escaped plane {:?}",
                vertex.position,
                plane
            );
        }
    }
}

#[test]
fn marine_and_ogre_are_complete_distinct_recipes() {
    let marine = ActorModel::marine();
    let ogre = ActorModel::ogre();
    assert!(marine.parts.len() >= 15);
    assert_eq!(marine.parts.len(), ogre.parts.len());
    assert!(ogre.morphology.shoulder_width > marine.morphology.shoulder_width * 1.5);
    assert!(ogre.morphology.foot_length > marine.morphology.foot_length * 1.5);
    assert!(marine
        .parts
        .iter()
        .all(|part| !part.mesh.indices.is_empty()));
    assert!(ogre.parts.iter().all(|part| !part.mesh.indices.is_empty()));
}

#[test]
fn walking_pose_produces_finite_world_geometry() {
    let ground = Flat;
    let mut walker = Walker::new_at(WalkerSpec::humanoid(), &ground, 0.0, 0.0, 0.0, 0.28);
    for _ in 0..180 {
        walker.step(&ground, WalkerCommand::sprint(Vec3::Z, 0.0), 1.0 / 60.0);
    }
    let model = ActorModel::marine();
    let pose = pose_walker(&walker, &model.morphology);
    assert!(pose
        .transforms
        .iter()
        .flat_map(|matrix| matrix.to_cols_array())
        .all(f32::is_finite));
    let mesh = model.mesh_for_pose(&pose.transforms);
    assert!(mesh.vertices.len() > 100);
    assert!(mesh.indices.len() > 100);
    assert!(mesh.vertices.iter().all(|vertex| {
        vertex.position.is_finite() && vertex.normal.is_finite() && vertex.uv.is_finite()
    }));
}

#[test]
fn persistent_pose_driver_removes_plant_discontinuities() {
    let ground = Flat;
    let mut spec = WalkerSpec::humanoid();
    spec.bend = KneeBend::Normal;
    let mut walker = Walker::new_at(spec, &ground, 0.0, 0.0, 0.0, 0.28);
    let model = ActorModel::marine();
    let mut driver = PoseDriver::new();
    let mut previous_direct: Option<PosePalette> = None;
    let mut previous_smooth: Option<PosePalette> = None;
    let mut max_direct_arm_step = 0.0_f32;
    let mut max_smooth_arm_step = 0.0_f32;
    let mut max_smooth_torso_step = 0.0_f32;

    for tick in 0..720 {
        let command = if tick < 240 {
            WalkerCommand::walk(Vec3::Z, 0.0)
        } else if tick < 480 {
            WalkerCommand::sprint(Vec3::Z, 0.0)
        } else {
            WalkerCommand::walk(Vec3::X, 0.0)
        };
        walker.step(&ground, command, 1.0 / 60.0);
        let direct = pose_walker(&walker, &model.morphology);
        let smooth = driver.update(&walker, &model.morphology, 1.0 / 60.0);
        if let Some(previous) = &previous_direct {
            max_direct_arm_step =
                max_direct_arm_step.max(rotation_step(previous, &direct, Bone::LeftUpperArm));
        }
        if let Some(previous) = &previous_smooth {
            max_smooth_arm_step =
                max_smooth_arm_step.max(rotation_step(previous, &smooth, Bone::LeftUpperArm));
            max_smooth_torso_step =
                max_smooth_torso_step.max(rotation_step(previous, &smooth, Bone::Torso));
        }
        previous_direct = Some(direct);
        previous_smooth = Some(smooth);
    }

    assert!(
        max_smooth_arm_step < max_direct_arm_step * 0.75,
        "smoothed arm step {max_smooth_arm_step:.3} vs direct {max_direct_arm_step:.3}"
    );
    assert!(max_smooth_arm_step < 0.16, "{max_smooth_arm_step}");
    assert!(max_smooth_torso_step < 0.08, "{max_smooth_torso_step}");
}

#[test]
fn arms_swing_contralaterally_without_inheriting_hunch() {
    let ground = Flat;
    let model = ActorModel::ogre();
    let mut walker = Walker::new_at(WalkerSpec::biped(), &ground, 0.0, 0.0, 0.0, 0.5);

    let idle_pose = pose_walker(&walker, &model.morphology);
    let idle_upper = bone_direction(&idle_pose, Bone::RightUpperArm);
    let idle_forearm = bone_direction(&idle_pose, Bone::RightForearm);
    assert!(idle_upper.z.abs() < 1.0e-3, "{idle_upper:?}");
    assert!(idle_forearm.z > 0.20, "{idle_forearm:?}");

    let mut phased_samples = 0;
    for _ in 0..240 {
        walker.step(&ground, WalkerCommand::walk(Vec3::Z, 0.0), 1.0 / 60.0);
        let signals = walker.signals();
        let right_foot =
            Quat::from_rotation_y(-signals.yaw) * (walker.leg_chain(1).foot - signals.pos);
        if right_foot.z.abs() < 0.18 {
            continue;
        }
        let pose = pose_walker(&walker, &model.morphology);
        let left_arm =
            Quat::from_rotation_y(-signals.yaw) * bone_direction(&pose, Bone::LeftUpperArm);
        assert!(
            left_arm.z * right_foot.z > 0.0,
            "left arm {left_arm:?} did not follow contralateral foot {right_foot:?}"
        );
        phased_samples += 1;
    }
    assert!(phased_samples > 20, "{phased_samples}");
}

fn bone_direction(pose: &PosePalette, bone: Bone) -> Vec3 {
    pose.transforms[bone as usize]
        .transform_vector3(Vec3::NEG_Y)
        .normalize()
}

fn rotation_step(previous: &PosePalette, current: &PosePalette, bone: Bone) -> f32 {
    let rotation = |pose: &PosePalette| -> Quat {
        pose.transforms[bone as usize]
            .to_scale_rotation_translation()
            .1
    };
    rotation(previous).angle_between(rotation(current))
}

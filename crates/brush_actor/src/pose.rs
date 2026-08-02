use glam::{Mat4, Quat, Vec3};
use walker2::Walker;

use crate::{Bone, Morphology};

#[derive(Clone)]
pub struct PosePalette {
    pub transforms: [Mat4; Bone::COUNT],
}

impl Default for PosePalette {
    fn default() -> Self {
        Self {
            transforms: [Mat4::IDENTITY; Bone::COUNT],
        }
    }
}

impl PosePalette {
    pub fn interpolate(&self, next: &Self, alpha: f32) -> Self {
        let alpha = alpha.clamp(0.0, 1.0);
        let mut output = Self::default();
        for (index, transform) in output.transforms.iter_mut().enumerate() {
            let (from_scale, from_rotation, from_translation) =
                self.transforms[index].to_scale_rotation_translation();
            let (to_scale, to_rotation, to_translation) =
                next.transforms[index].to_scale_rotation_translation();
            *transform = Mat4::from_scale_rotation_translation(
                from_scale.lerp(to_scale, alpha),
                from_rotation.slerp(to_rotation, alpha),
                from_translation.lerp(to_translation, alpha),
            );
        }
        output
    }
}

#[derive(Clone, Copy, Default)]
struct Spring {
    value: f32,
    velocity: f32,
}

impl Spring {
    fn snap(&mut self, value: f32) {
        self.value = value;
        self.velocity = 0.0;
    }

    fn update(&mut self, target: f32, frequency: f32, dt: f32) {
        let omega = std::f32::consts::TAU * frequency;
        let dt = dt.clamp(0.0, 0.1);
        let error = self.value - target;
        let c = self.velocity + omega * error;
        let decay = (-omega * dt).exp();
        self.value = target + (error + c * dt) * decay;
        self.velocity = (self.velocity - omega * c * dt) * decay;
    }
}

#[derive(Default)]
pub struct PoseDriver {
    initialized: bool,
    torso_twist: Spring,
    arm_swing: [Spring; 2],
    foot_roll: [Spring; 2],
}

impl PoseDriver {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn update(&mut self, walker: &Walker, morphology: &Morphology, dt: f32) -> PosePalette {
        let signals = walker.signals();
        let left_chain = walker.leg_chain(0);
        let right_chain = walker.leg_chain(1);
        let body_rotation = body_rotation(&signals);
        let inverse_body = body_rotation.conjugate();
        let raw_twist = signals
            .support_yaw
            .map(|support| angle_delta(support, signals.yaw).clamp(-0.34, 0.34))
            .unwrap_or(0.0);
        let twist_target = self.torso_twist.value + angle_delta(raw_twist, self.torso_twist.value);
        let left_foot_local = inverse_body * (left_chain.foot - signals.pos);
        let right_foot_local = inverse_body * (right_chain.foot - signals.pos);
        let swing_targets = [
            (right_foot_local.z * 0.75).clamp(-0.65, 0.65),
            (left_foot_local.z * 0.75).clamp(-0.65, 0.65),
        ];
        let foot_roll_targets = [
            foot_roll_target(&signals, &left_chain, morphology, 0),
            foot_roll_target(&signals, &right_chain, morphology, 1),
        ];

        if !self.initialized {
            self.torso_twist.snap(twist_target);
            self.arm_swing[0].snap(swing_targets[0]);
            self.arm_swing[1].snap(swing_targets[1]);
            self.foot_roll[0].snap(foot_roll_targets[0]);
            self.foot_roll[1].snap(foot_roll_targets[1]);
            self.initialized = true;
        } else {
            self.torso_twist.update(twist_target, 4.0, dt);
            self.arm_swing[0].update(swing_targets[0], 4.8, dt);
            self.arm_swing[1].update(swing_targets[1], 4.8, dt);
            self.foot_roll[0].update(foot_roll_targets[0], 7.0, dt);
            self.foot_roll[1].update(foot_roll_targets[1], 7.0, dt);
        }

        compose_pose(
            &signals,
            &left_chain,
            &right_chain,
            morphology,
            self.torso_twist.value,
            [self.arm_swing[0].value, self.arm_swing[1].value],
            [self.foot_roll[0].value, self.foot_roll[1].value],
        )
    }
}

pub fn pose_walker(walker: &Walker, morphology: &Morphology) -> PosePalette {
    let signals = walker.signals();
    let left_chain = walker.leg_chain(0);
    let right_chain = walker.leg_chain(1);
    let inverse_body = body_rotation(&signals).conjugate();
    let support_twist = signals
        .support_yaw
        .map(|support| angle_delta(support, signals.yaw).clamp(-0.34, 0.34))
        .unwrap_or(0.0);
    let arm_swing = [
        ((inverse_body * (right_chain.foot - signals.pos)).z * 0.75).clamp(-0.65, 0.65),
        ((inverse_body * (left_chain.foot - signals.pos)).z * 0.75).clamp(-0.65, 0.65),
    ];
    let foot_roll = [
        foot_roll_target(&signals, &left_chain, morphology, 0),
        foot_roll_target(&signals, &right_chain, morphology, 1),
    ];
    compose_pose(
        &signals,
        &left_chain,
        &right_chain,
        morphology,
        support_twist,
        arm_swing,
        foot_roll,
    )
}

fn compose_pose(
    signals: &walker2::RigSignals,
    left_chain: &walker2::LegChain,
    right_chain: &walker2::LegChain,
    morphology: &Morphology,
    support_twist: f32,
    arm_swing: [f32; 2],
    foot_roll: [f32; 2],
) -> PosePalette {
    let body_rotation = body_rotation(signals);
    let body = Mat4::from_rotation_translation(body_rotation, signals.pos);
    let mut pose = PosePalette::default();

    let pelvis_local = Mat4::from_translation(Vec3::new(0.0, -morphology.torso_height * 0.42, 0.0));
    pose.transforms[Bone::Pelvis as usize] = body * pelvis_local;

    let torso_local = Mat4::from_translation(Vec3::new(0.0, 0.02, 0.0))
        * Mat4::from_quat(Quat::from_rotation_y(support_twist * 0.28))
        * Mat4::from_quat(Quat::from_rotation_x(morphology.hunch));
    let torso = body * torso_local;
    let arm_rotation = body_rotation * Quat::from_rotation_y(support_twist * 0.28);
    pose.transforms[Bone::Torso as usize] = torso;
    pose.transforms[Bone::Head as usize] = torso
        * Mat4::from_translation(Vec3::new(
            0.0,
            morphology.torso_height * 0.52 + morphology.head_height * 0.38,
            morphology.hunch * 0.42,
        ));

    pose_arm(
        &mut pose,
        torso,
        arm_rotation,
        -1.0,
        arm_swing[0],
        morphology,
        [Bone::LeftUpperArm, Bone::LeftForearm, Bone::LeftHand],
    );
    pose_arm(
        &mut pose,
        torso,
        arm_rotation,
        1.0,
        arm_swing[1],
        morphology,
        [Bone::RightUpperArm, Bone::RightForearm, Bone::RightHand],
    );

    pose_leg(
        &mut pose,
        left_chain,
        morphology,
        [
            Bone::LeftThigh,
            Bone::LeftShin,
            Bone::LeftAnkle,
            Bone::LeftFoot,
        ],
        foot_roll[0],
        signals.yaw,
    );
    pose_leg(
        &mut pose,
        right_chain,
        morphology,
        [
            Bone::RightThigh,
            Bone::RightShin,
            Bone::RightAnkle,
            Bone::RightFoot,
        ],
        foot_roll[1],
        signals.yaw,
    );

    pose
}

fn body_rotation(signals: &walker2::RigSignals) -> Quat {
    Quat::from_rotation_y(signals.yaw)
        * Quat::from_rotation_z(signals.roll)
        * Quat::from_rotation_x(signals.pitch)
}

fn pose_arm(
    pose: &mut PosePalette,
    torso: Mat4,
    arm_rotation: Quat,
    side: f32,
    swing: f32,
    morphology: &Morphology,
    bones: [Bone; 3],
) {
    let shoulder = torso.transform_point3(Vec3::new(
        side * morphology.shoulder_width * 0.5,
        morphology.torso_height * 0.30,
        0.0,
    ));
    let upper_direction =
        arm_rotation * Vec3::new(side * 0.08, -swing.cos(), swing.sin()).normalize();
    let elbow = shoulder + upper_direction * morphology.upper_arm_length;
    let forearm_angle = swing * 0.55 + 0.28;
    let forearm_direction = arm_rotation
        * Vec3::new(side * 0.04, -forearm_angle.cos(), forearm_angle.sin()).normalize();
    let wrist = elbow + forearm_direction * morphology.forearm_length;
    let hand_end = wrist + forearm_direction * morphology.hand_length;

    pose.transforms[bones[0] as usize] =
        segment_transform(shoulder, elbow, morphology.upper_arm_length);
    pose.transforms[bones[1] as usize] = segment_transform(elbow, wrist, morphology.forearm_length);
    pose.transforms[bones[2] as usize] = segment_transform(wrist, hand_end, morphology.hand_length);
}

fn pose_leg(
    pose: &mut PosePalette,
    chain: &walker2::LegChain,
    morphology: &Morphology,
    bones: [Bone; 4],
    foot_roll: f32,
    yaw: f32,
) {
    let (ankle, foot_transform) = rolled_foot_transform(chain.foot, yaw, foot_roll, morphology);
    let (knee, lower_split) = solve_two_bone_leg(chain.hip, ankle, morphology, yaw);
    pose.transforms[bones[0] as usize] =
        segment_transform(chain.hip, knee, morphology.thigh_length);
    pose.transforms[bones[1] as usize] =
        segment_transform(knee, lower_split, morphology.shin_length);
    pose.transforms[bones[2] as usize] =
        segment_transform(lower_split, ankle, morphology.ankle_length);
    pose.transforms[bones[3] as usize] = foot_transform;
}

fn foot_roll_target(
    signals: &walker2::RigSignals,
    chain: &walker2::LegChain,
    morphology: &Morphology,
    leg: usize,
) -> f32 {
    let facing = Quat::from_rotation_y(signals.yaw);
    let planar_velocity = Vec3::new(signals.vel.x, 0.0, signals.vel.z);
    let local_velocity = facing.conjugate() * planar_velocity;
    let forward_speed = local_velocity.z.max(0.0);
    let lateral_speed = local_velocity.x.abs();
    let direction_weight = (forward_speed / (forward_speed + lateral_speed + 1.0e-4)).powi(2);
    let speed_weight = (forward_speed / (morphology.foot_length * 3.0).max(1.0e-4)).clamp(0.0, 1.0);
    let weight = direction_weight * speed_weight;

    if !signals.legs[leg].contact {
        let t = signals.legs[leg].swing_t;
        let push_off = 0.42 * (1.0 - smoothstep(0.0, 0.32, t));
        let toe_clearance = -0.24 * smoothstep(0.18, 0.72, t);
        return (push_off + toe_clearance) * weight;
    }

    let foot_local = facing.conjugate() * (chain.foot - signals.pos);
    let support_position = foot_local.z / (morphology.foot_length * 1.35).max(1.0e-4);
    if support_position >= 0.0 {
        -0.32 * smoothstep(0.08, 0.85, support_position) * weight
    } else {
        0.42 * smoothstep(0.10, 0.85, -support_position) * weight
    }
}

fn rolled_foot_transform(
    flat_origin: Vec3,
    yaw: f32,
    pitch: f32,
    morphology: &Morphology,
) -> (Vec3, Mat4) {
    let yaw_rotation = Quat::from_rotation_y(yaw);
    let rotation = yaw_rotation * Quat::from_rotation_x(pitch);
    let pivot_z = if pitch > 0.0 {
        morphology.ball_offset
    } else {
        morphology.heel_offset
    };
    let local_pivot = Vec3::Z * pivot_z;
    let pivot = flat_origin + yaw_rotation * local_pivot;
    let ankle = pivot - rotation * local_pivot;
    (ankle, Mat4::from_rotation_translation(rotation, ankle))
}

fn smoothstep(start: f32, end: f32, value: f32) -> f32 {
    let t = ((value - start) / (end - start)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn solve_two_bone_leg(hip: Vec3, foot: Vec3, morphology: &Morphology, yaw: f32) -> (Vec3, Vec3) {
    let delta = foot - hip;
    let distance = delta.length().max(1.0e-4);
    let direction = delta / distance;
    let mut thigh_length = morphology.thigh_length;
    let mut lower_length = morphology.shin_length + morphology.ankle_length;
    let required_scale = (distance * 1.035 / (thigh_length + lower_length)).max(1.0);
    thigh_length *= required_scale;
    lower_length *= required_scale;

    let solved_distance = distance
        .min((thigh_length + lower_length) * 0.999)
        .max((thigh_length - lower_length).abs() + 1.0e-4);
    let along = (thigh_length * thigh_length - lower_length * lower_length
        + solved_distance * solved_distance)
        / (2.0 * solved_distance);
    let bend = (thigh_length * thigh_length - along * along)
        .max(0.0)
        .sqrt();
    let forward = Quat::from_rotation_y(yaw) * Vec3::Z;
    let mut pole = forward - direction * forward.dot(direction);
    if pole.length_squared() < 1.0e-5 {
        pole = Vec3::Y - direction * direction.y;
    }
    let knee = hip + direction * along + pole.normalize_or(Vec3::Z) * bend;
    let lower_fraction =
        morphology.shin_length / (morphology.shin_length + morphology.ankle_length);
    let lower_split = knee.lerp(foot, lower_fraction);
    (knee, lower_split)
}

fn segment_transform(start: Vec3, end: Vec3, bind_length: f32) -> Mat4 {
    let delta = end - start;
    let length = delta.length().max(1.0e-5);
    let rotation = Quat::from_rotation_arc(Vec3::NEG_Y, delta / length);
    Mat4::from_scale_rotation_translation(
        Vec3::new(1.0, length / bind_length.max(1.0e-5), 1.0),
        rotation,
        start,
    )
}

fn angle_delta(to: f32, from: f32) -> f32 {
    (to - from).sin().atan2((to - from).cos())
}

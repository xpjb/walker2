use glam::Vec3;

/// Which way the leg chain folds. `Normal` is the reverse-joint "chicken
/// walker" silhouette; `Forward` is a human-like knee; `Backward` folds
/// the whole chain rearward.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KneeBend {
    Normal,
    Backward,
    Forward,
}

/// A walk/sprint parameter pair.
#[derive(Clone, Copy, Debug)]
pub struct Pair {
    pub walk: f32,
    pub sprint: f32,
}

impl Pair {
    pub const fn new(walk: f32, sprint: f32) -> Self {
        Self { walk, sprint }
    }

    #[inline]
    pub fn at(self, sprint: bool) -> f32 {
        if sprint { self.sprint } else { self.walk }
    }
}

/// Rate/accel-limited scalar servo parameters (attitude axes).
#[derive(Clone, Copy, Debug)]
pub struct AxisMotor {
    pub max_speed: f32,
    pub max_accel: f32,
    pub stiffness: f32,
}

/// Stance traction envelope: how hard planted feet can push the body.
#[derive(Clone, Copy, Debug)]
pub struct TractionParams {
    pub friction: f32,
    pub max_force: f32,
    pub forward_share: f32,
    pub lateral_share: f32,
    pub brake_normal_boost: f32,
    pub brake_force_boost: f32,
}

/// Radial leg-spring model for planted feet (supports the body, resists
/// compression, damps radial velocity).
#[derive(Clone, Copy, Debug)]
pub struct SlipParams {
    pub stiffness: f32,
    pub damping: f32,
    pub preload: f32,
    pub max_force: f32,
    pub rest_bias: f32,
    pub speed_rest_bias: f32,
    pub command_rest_bias: f32,
    pub preload_window: f32,
    pub max_compression: f32,
}

/// Full data-driven tuning for one walker morphology.
///
/// UNITS ARE CANONICAL, NOT WORLD. Specs describe a hand-tuned baseline
/// (the shipped presets sit around a ~4–6 unit leg); actual size comes
/// exclusively from the `scale` you spawn with, which Froude-scales the
/// whole sim (`world speed = sqrt(scale) * canonical speed`). Resist the
/// urge to author a "small" spec: shrink with `scale`, keep the spec in
/// the regime the planner constants were tuned for.
#[derive(Clone, Copy, Debug)]
pub struct WalkerSpec {
    // -- morphology --------------------------------------------------------
    /// Hip-to-knee link length.
    pub hip_link: f32,
    /// Knee-to-hock link (the compound/reverse segment on `Normal` bends).
    pub reverse_link: f32,
    /// Optional fourth link (0.0 disables).
    pub mid_link: f32,
    /// Final link to the foot.
    pub shin_link: f32,
    /// Absolute hip-to-foot reach limit.
    pub max_reach: f32,
    /// Hip pivot drop below the body origin.
    pub hip_drop: f32,
    /// Preferred hip-to-foot distance in stance; body height above ground
    /// is `stance_len + hip_drop`.
    pub stance_len: f32,
    /// Lateral hip offset from centerline.
    pub hip_width: f32,
    /// Max lateral hip-vs-anchor excursion before a stance foot is dragged.
    pub max_lateral: f32,
    pub bend: KneeBend,
    pub foot_size: Vec3,
    pub com_offset: Vec3,
    // Debug/preview body boxes for `part_poses`.
    pub hull_size: Vec3,
    pub hull_offset: Vec3,
    pub pelvis_size: Vec3,
    pub pelvis_offset: Vec3,

    // -- gait --------------------------------------------------------------
    /// Commanded planar speed.
    pub speed: Pair,
    /// Cadence floor: minimum time between step starts.
    pub step_interval: Pair,
    pub step_height: Pair,
    pub swing_duration: Pair,
    /// Lateral half-spacing of foot targets from the COM line.
    pub step_width: f32,
    /// Raibert forward stride target.
    pub stride_target: Pair,
    /// Minimum downrange commitment of a planned step while moving.
    pub commit_distance: Pair,
    /// Step length below which short-step cost kicks in.
    pub desired_step: Pair,
    pub min_step_commit: f32,
    pub step_cooldown: f32,
    /// (min, max) forward candidate range relative to the hip, moving.
    pub forward_min_move: Pair,
    pub forward_max_move: Pair,
    /// Same, idle.
    pub forward_limits_idle: (f32, f32),
    pub max_planar_move: Pair,
    pub max_planar_idle: f32,

    // -- dynamics ----------------------------------------------------------
    pub traction_walk: TractionParams,
    pub traction_sprint: TractionParams,
    /// Velocity-tracking gain with at least one foot planted.
    pub drive_gain: Pair,
    /// Same, airborne/no-stance.
    pub drive_gain_air: f32,
    pub slip_walk: SlipParams,
    pub slip_sprint: SlipParams,
    pub planar_damping: f32,
    pub footfall_base: f32,
    pub yaw_stiffness: f32,
    pub yaw_damping: f32,
    pub pitch_motor: AxisMotor,
    pub pitch_motor_fast: AxisMotor,
    pub roll_motor: AxisMotor,
    pub roll_motor_fast: AxisMotor,
    /// Courier-class twitch: widens accel clamps and attitude limits.
    pub fast_twitch: bool,
    /// Commanded speed above which the "fast" motor/lean set engages.
    pub sprint_like_speed: f32,

    // -- balance & planning ------------------------------------------------
    /// COM-vs-support soft margins (double-stance, single-stance), forward.
    pub balance_margin_forward: (f32, f32),
    pub balance_margin_lateral: (f32, f32),
    /// Expected balance-cost margin (idle, moving, sprint-like) before
    /// control risk ramps.
    pub risk_margin: (f32, f32, f32),
    pub chain_posture_weight: f32,
    pub preferred_extension: f32,
    pub stance_leash: f32,
    /// Minimum lateral gap between the two feet the planner protects.
    pub min_pair_gap: f32,

    // -- untwist (replaces foot-planned turning) ---------------------------
    /// Lateral spacing (from the other anchor) of an untwist re-plant.
    pub untwist_pair_lateral: f32,
    /// Small forward bias of an untwist re-plant.
    pub untwist_forward_bias: f32,
    /// Support-line yaw error (rad) where untwist stepping begins.
    pub untwist_gate: f32,
    /// Additional error (rad) over which untwist urgency ramps to 1.
    pub untwist_ramp: f32,
}

impl WalkerSpec {
    #[inline]
    pub fn body_height(&self) -> f32 {
        self.stance_len + self.hip_drop
    }

    #[inline]
    pub fn traction(&self, sprint: bool) -> &TractionParams {
        if sprint { &self.traction_sprint } else { &self.traction_walk }
    }

    #[inline]
    pub fn slip(&self, sprint: bool) -> &SlipParams {
        if sprint { &self.slip_sprint } else { &self.slip_walk }
    }

    /// The proven baseline: petrogradrevival's reverse-joint biped mech.
    pub fn biped() -> Self {
        Self {
            hip_link: 1.88,
            reverse_link: 1.42,
            mid_link: 0.0,
            shin_link: 1.56,
            max_reach: 4.74,
            hip_drop: 0.78,
            stance_len: 3.56,
            hip_width: 0.76,
            max_lateral: 0.30,
            bend: KneeBend::Normal,
            foot_size: Vec3::new(0.82, 0.26, 1.30),
            com_offset: Vec3::new(0.0, 0.62, -0.08),
            hull_size: Vec3::new(2.20, 1.40, 1.90),
            hull_offset: Vec3::new(0.0, 0.20, 0.0),
            pelvis_size: Vec3::new(1.30, 0.60, 0.90),
            pelvis_offset: Vec3::new(0.0, -0.35, 0.0),

            speed: Pair::new(5.5, 9.4),
            step_interval: Pair::new(0.46, 0.32),
            step_height: Pair::new(0.60, 0.76),
            swing_duration: Pair::new(0.35, 0.28),
            step_width: 0.54,
            stride_target: Pair::new(1.34, 2.06),
            commit_distance: Pair::new(0.98, 1.50),
            desired_step: Pair::new(3.45, 4.55),
            min_step_commit: 0.35,
            step_cooldown: 0.18,
            forward_min_move: Pair::new(0.10, 0.20),
            forward_max_move: Pair::new(1.62, 2.42),
            forward_limits_idle: (-0.70, 0.82),
            max_planar_move: Pair::new(1.96, 2.70),
            max_planar_idle: 1.45,

            traction_walk: TractionParams {
                friction: 0.82,
                max_force: 15.5,
                forward_share: 0.94,
                lateral_share: 0.56,
                brake_normal_boost: 2.35,
                brake_force_boost: 2.05,
            },
            traction_sprint: TractionParams {
                friction: 0.94,
                max_force: 18.5,
                forward_share: 1.00,
                lateral_share: 0.60,
                brake_normal_boost: 2.60,
                brake_force_boost: 2.25,
            },
            drive_gain: Pair::new(2.25, 2.65),
            drive_gain_air: 0.85,
            slip_walk: SlipParams {
                stiffness: 21.0,
                damping: 2.5,
                preload: 6.0,
                max_force: 15.5,
                rest_bias: 0.17,
                speed_rest_bias: 0.08,
                command_rest_bias: 0.07,
                preload_window: 0.15,
                max_compression: 0.40,
            },
            slip_sprint: SlipParams {
                stiffness: 27.0,
                damping: 2.8,
                preload: 6.8,
                max_force: 18.0,
                rest_bias: 0.22,
                speed_rest_bias: 0.12,
                command_rest_bias: 0.10,
                preload_window: 0.16,
                max_compression: 0.48,
            },
            planar_damping: 0.26,
            footfall_base: 0.72,
            yaw_stiffness: 8.0,
            yaw_damping: 4.2,
            pitch_motor: AxisMotor { max_speed: 0.70, max_accel: 6.6, stiffness: 13.4 },
            pitch_motor_fast: AxisMotor { max_speed: 0.78, max_accel: 7.4, stiffness: 13.4 },
            roll_motor: AxisMotor { max_speed: 0.76, max_accel: 7.2, stiffness: 14.2 },
            roll_motor_fast: AxisMotor { max_speed: 0.84, max_accel: 8.2, stiffness: 14.2 },
            fast_twitch: false,
            sprint_like_speed: 7.0,

            balance_margin_forward: (0.34, 0.18),
            balance_margin_lateral: (0.42, 0.24),
            risk_margin: (0.78, 1.90, 2.25),
            chain_posture_weight: 0.0,
            preferred_extension: 0.72,
            stance_leash: 0.95,
            min_pair_gap: 0.62,

            untwist_pair_lateral: 1.42,
            untwist_forward_bias: 0.58,
            untwist_gate: 0.30,
            untwist_ramp: 0.40,
        }
    }

    /// Long-legged fast strider (second stable preset; proves the spec
    /// actually parameterizes the planner).
    pub fn longstrider() -> Self {
        Self {
            hip_link: 2.62,
            reverse_link: 2.04,
            mid_link: 0.0,
            shin_link: 2.18,
            max_reach: 6.62,
            hip_drop: 1.12,
            stance_len: 5.06,
            hip_width: 0.92,
            max_lateral: 0.38,
            bend: KneeBend::Normal,
            foot_size: Vec3::new(0.68, 0.22, 1.62),
            com_offset: Vec3::new(0.0, 0.62, -0.08),
            hull_size: Vec3::new(2.10, 1.30, 2.10),
            hull_offset: Vec3::new(0.0, 0.20, 0.0),
            pelvis_size: Vec3::new(1.40, 0.60, 0.95),
            pelvis_offset: Vec3::new(0.0, -0.40, 0.0),

            speed: Pair::new(8.4, 15.6),
            step_interval: Pair::new(0.40, 0.26),
            step_height: Pair::new(0.82, 1.06),
            swing_duration: Pair::new(0.32, 0.24),
            step_width: 0.68,
            stride_target: Pair::new(1.88, 3.10),
            commit_distance: Pair::new(1.28, 2.20),
            desired_step: Pair::new(4.25, 6.05),
            min_step_commit: 0.35,
            step_cooldown: 0.15,
            forward_min_move: Pair::new(0.16, 0.32),
            forward_max_move: Pair::new(2.46, 3.70),
            forward_limits_idle: (-1.02, 1.14),
            max_planar_move: Pair::new(2.72, 4.08),
            max_planar_idle: 1.92,

            traction_walk: TractionParams {
                friction: 0.86,
                max_force: 17.0,
                forward_share: 0.96,
                lateral_share: 0.58,
                brake_normal_boost: 2.35,
                brake_force_boost: 2.05,
            },
            traction_sprint: TractionParams {
                friction: 0.98,
                max_force: 21.0,
                forward_share: 1.02,
                lateral_share: 0.62,
                brake_normal_boost: 2.55,
                brake_force_boost: 2.20,
            },
            drive_gain: Pair::new(2.45, 3.05),
            drive_gain_air: 0.85,
            slip_walk: SlipParams {
                stiffness: 24.0,
                damping: 2.7,
                preload: 6.0,
                max_force: 17.0,
                rest_bias: 0.18,
                speed_rest_bias: 0.10,
                command_rest_bias: 0.08,
                preload_window: 0.16,
                max_compression: 0.46,
            },
            slip_sprint: SlipParams {
                stiffness: 31.0,
                damping: 3.0,
                preload: 6.8,
                max_force: 20.0,
                rest_bias: 0.22,
                speed_rest_bias: 0.14,
                command_rest_bias: 0.12,
                preload_window: 0.17,
                max_compression: 0.54,
            },
            planar_damping: 0.18,
            footfall_base: 0.62,
            yaw_stiffness: 7.0,
            yaw_damping: 3.6,
            pitch_motor: AxisMotor { max_speed: 0.70, max_accel: 6.6, stiffness: 13.4 },
            pitch_motor_fast: AxisMotor { max_speed: 0.78, max_accel: 7.4, stiffness: 13.4 },
            roll_motor: AxisMotor { max_speed: 0.76, max_accel: 7.2, stiffness: 14.2 },
            roll_motor_fast: AxisMotor { max_speed: 0.84, max_accel: 8.2, stiffness: 14.2 },
            fast_twitch: false,
            sprint_like_speed: 7.0,

            balance_margin_forward: (0.34, 0.18),
            balance_margin_lateral: (0.42, 0.24),
            risk_margin: (0.78, 1.90, 2.25),
            chain_posture_weight: 0.0,
            preferred_extension: 0.72,
            stance_leash: 0.95,
            min_pair_gap: 0.62,

            untwist_pair_lateral: 1.72,
            untwist_forward_bias: 0.58,
            untwist_gate: 0.30,
            untwist_ramp: 0.40,
        }
    }

    /// EXPERIMENTAL forward-knee humanoid silhouette (derived from the
    /// forward-chain mech tables, narrowed). Canonical units like every
    /// spec — spawn at scale ~0.28 for a human-sized character. Validate
    /// with the examples before trusting it in a game.
    pub fn humanoid() -> Self {
        Self {
            hip_link: 2.22,
            reverse_link: 1.88,
            mid_link: 0.0,
            shin_link: 2.08,
            max_reach: 6.12,
            hip_drop: 1.04,
            stance_len: 4.70,
            hip_width: 0.55,
            max_lateral: 0.30,
            bend: KneeBend::Forward,
            foot_size: Vec3::new(0.50, 0.18, 1.10),
            com_offset: Vec3::new(0.0, 0.54, 0.02),
            hull_size: Vec3::new(1.60, 1.60, 1.10),
            hull_offset: Vec3::new(0.0, 0.55, 0.0),
            pelvis_size: Vec3::new(1.10, 0.55, 0.75),
            pelvis_offset: Vec3::new(0.0, -0.40, 0.0),

            speed: Pair::new(7.4, 13.4),
            step_interval: Pair::new(0.40, 0.27),
            step_height: Pair::new(0.76, 0.96),
            swing_duration: Pair::new(0.35, 0.25),
            step_width: 0.50,
            stride_target: Pair::new(2.38, 3.90),
            commit_distance: Pair::new(1.32, 2.36),
            desired_step: Pair::new(4.15, 6.55),
            min_step_commit: 0.35,
            step_cooldown: 0.12,
            forward_min_move: Pair::new(0.16, 0.30),
            forward_max_move: Pair::new(2.95, 4.65),
            forward_limits_idle: (-1.02, 1.28),
            max_planar_move: Pair::new(3.28, 5.10),
            max_planar_idle: 2.02,

            traction_walk: TractionParams {
                friction: 0.88,
                max_force: 17.5,
                forward_share: 0.96,
                lateral_share: 0.58,
                brake_normal_boost: 2.38,
                brake_force_boost: 2.06,
            },
            traction_sprint: TractionParams {
                friction: 1.00,
                max_force: 22.0,
                forward_share: 1.02,
                lateral_share: 0.62,
                brake_normal_boost: 2.55,
                brake_force_boost: 2.20,
            },
            drive_gain: Pair::new(2.25, 2.65),
            drive_gain_air: 0.85,
            slip_walk: SlipParams {
                stiffness: 25.0,
                damping: 2.8,
                preload: 6.3,
                max_force: 17.5,
                rest_bias: 0.20,
                speed_rest_bias: 0.10,
                command_rest_bias: 0.09,
                preload_window: 0.16,
                max_compression: 0.48,
            },
            slip_sprint: SlipParams {
                stiffness: 33.0,
                damping: 3.1,
                preload: 7.1,
                max_force: 21.0,
                rest_bias: 0.24,
                speed_rest_bias: 0.15,
                command_rest_bias: 0.14,
                preload_window: 0.17,
                max_compression: 0.56,
            },
            planar_damping: 0.26,
            footfall_base: 0.48,
            yaw_stiffness: 9.0,
            yaw_damping: 4.5,
            pitch_motor: AxisMotor { max_speed: 0.70, max_accel: 6.6, stiffness: 13.4 },
            pitch_motor_fast: AxisMotor { max_speed: 0.78, max_accel: 7.4, stiffness: 13.4 },
            roll_motor: AxisMotor { max_speed: 0.76, max_accel: 7.2, stiffness: 14.2 },
            roll_motor_fast: AxisMotor { max_speed: 0.84, max_accel: 8.2, stiffness: 14.2 },
            fast_twitch: false,
            sprint_like_speed: 7.0,

            balance_margin_forward: (0.34, 0.18),
            balance_margin_lateral: (0.42, 0.24),
            risk_margin: (0.78, 1.90, 2.25),
            chain_posture_weight: 0.62,
            preferred_extension: 0.66,
            stance_leash: 0.95,
            min_pair_gap: 0.45,

            untwist_pair_lateral: 1.05,
            untwist_forward_bias: 0.45,
            untwist_gate: 0.30,
            untwist_ramp: 0.40,
        }
    }
}

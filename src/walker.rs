use std::f32::consts::PI;

use glam::{Mat3, Quat, Vec3, Vec4};

use crate::{
    command::WalkerCommand,
    events::FootfallEvent,
    ground::{GroundQuery, ScaledGround},
    rig::{LegChain, LegSignal, MeshKey, PartPose, PartRole, RigSignals, Transform},
    spec::{AxisMotor, KneeBend, WalkerSpec},
    validation::GaitValidation,
};

const GRAVITY: f32 = -9.81;

/// Simulation quality tier. All tiers share the same state layout, so a
/// walker can switch tiers at runtime without popping.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fidelity {
    /// Full planner: candidate grid + cart-pole/effort/posture scoring,
    /// 14 dynamics substeps. Heroes and close-ups.
    Full,
    /// Same physics loop and step triggers, but plants at the nominal
    /// target (with an explicit cart-pole feedforward) instead of
    /// searching candidates; 4 substeps. Gameplay tier for many actors.
    Reduced,
    /// No balance dynamics at all: phase-driven kinematic gait around the
    /// commanded velocity, ground-following body. Crowds and distance.
    Kinematic,
}

impl Fidelity {
    fn substeps(self) -> usize {
        match self {
            Fidelity::Full => 14,
            Fidelity::Reduced => 4,
            Fidelity::Kinematic => 1,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum LegState {
    Stance,
    Swing,
}

#[derive(Clone, Copy)]
struct Leg {
    side: f32,
    hip_local: Vec3,
    foot: Vec3,
    anchor: Vec3,
    swing_start: Vec3,
    swing_target: Vec3,
    compound_len: f32,
    compound_vel: f32,
    compound_ready: bool,
    swing_t: f32,
    state: LegState,
    cooldown: f32,
    compression: f32,
}

impl Leg {
    fn new(side: f32, hip_local: Vec3, origin: Vec3, terrain: &impl GroundQuery) -> Self {
        let foot = terrain.point_at(origin.x + hip_local.x, origin.z + hip_local.z);
        Self {
            side,
            hip_local,
            foot,
            anchor: foot,
            swing_start: foot,
            swing_target: foot,
            compound_len: 0.0,
            compound_vel: 0.0,
            compound_ready: false,
            swing_t: 0.0,
            state: LegState::Stance,
            cooldown: 0.0,
            compression: 0.0,
        }
    }

    fn is_stance(&self) -> bool {
        matches!(self.state, LegState::Stance)
    }
}

/// Per-step planning context derived from the command. Turning is absent
/// by design: facing is applied by the yaw spring, and `untwist_weight`
/// only asks for a comfort re-plant once the feet lag the body too far.
#[derive(Clone, Copy)]
struct Intent {
    input: f32,
    sprint: bool,
    desired_vel: Vec3,
    /// Travel direction (movement when moving, facing when idle).
    forward: Vec3,
    /// BODY right (yaw frame): foot spread, anti-cross spacing, IK poles.
    right: Vec3,
    /// Travel-perpendicular right, sign-aligned with body right. Facing
    /// and travel are decoupled here, so (forward, body-right) can be
    /// parallel (pure strafe); clamps and directional costs use this
    /// orthogonal frame instead so no dimension collapses.
    travel_right: Vec3,
    /// 0.55..1.0: shrinks stride/step-length targets as travel disaligns
    /// from facing — sidesteps are shorter than forward strides.
    align_scale: f32,
    balance: Vec3,
    local_balance: Vec3,
    untwist_weight: f32,
    recover_weight: f32,
}

impl Intent {
    fn wants_untwist(&self) -> bool {
        self.untwist_weight > 0.02
    }

    fn wants_recovery(&self) -> bool {
        self.recover_weight > 0.10
    }
}

pub struct Walker {
    spec: WalkerSpec,
    fidelity: Fidelity,
    /// Uniform size multiplier; 1.0 = the spec's hand-tuned baseline. All
    /// internal state stays in CANONICAL units — only values crossing the
    /// public API are scaled. See `ScaledGround`.
    scale: f32,
    /// CANONICAL position; use `position()` for world space.
    pos: Vec3,
    prev_pos: Vec3,
    vel: Vec3,
    yaw: f32,
    yaw_vel: f32,
    pitch: f32,
    roll: f32,
    pitch_vel: f32,
    roll_vel: f32,
    support_y: f32,
    support_y_vel: f32,
    support_y_ready: bool,
    prev_y_vel: f32,
    prev_pitch_vel: f32,
    prev_roll_vel: f32,
    legs: [Leg; 2],
    next_leg: usize,
    step_timer: f32,
    ride_offset: f32,
    validation: GaitValidation,
    pending_footfalls: Vec<FootfallEvent>,
    actuator_force: f32,
    actuator_motion: f32,
    power_demand: f32,
    time: f32,
}

impl Walker {
    pub fn new(spec: WalkerSpec, terrain: &impl GroundQuery) -> Self {
        Self::new_at(spec, terrain, 0.0, 0.0, 0.0, 1.0)
    }

    /// `x`/`z` are world-space spawn coordinates; `scale` is the uniform
    /// size multiplier (1.0 = spec baseline).
    pub fn new_at(
        spec: WalkerSpec,
        terrain: &impl GroundQuery,
        x: f32,
        z: f32,
        yaw: f32,
        scale: f32,
    ) -> Self {
        let scale = scale.max(0.05);
        let ground = ScaledGround {
            inner: terrain,
            scale,
        };
        let terrain = &ground;
        let x = x / scale;
        let z = z / scale;
        let pos = Vec3::new(x, terrain.height_at(x, z) + spec.body_height(), z);
        let origin = Vec3::new(x, 0.0, z);
        let hip = Vec3::new(spec.hip_width, -spec.hip_drop, 0.0);
        Self {
            spec,
            fidelity: Fidelity::Full,
            scale,
            pos,
            prev_pos: pos,
            vel: Vec3::ZERO,
            yaw,
            yaw_vel: 0.0,
            pitch: 0.0,
            roll: 0.0,
            pitch_vel: 0.0,
            roll_vel: 0.0,
            support_y: pos.y,
            support_y_vel: 0.0,
            support_y_ready: true,
            prev_y_vel: 0.0,
            prev_pitch_vel: 0.0,
            prev_roll_vel: 0.0,
            legs: [
                Leg::new(-1.0, Vec3::new(-hip.x, hip.y, hip.z), origin, terrain),
                Leg::new(1.0, hip, origin, terrain),
            ],
            next_leg: 0,
            step_timer: 0.0,
            ride_offset: 0.0,
            validation: GaitValidation::default(),
            pending_footfalls: Vec::with_capacity(8),
            actuator_force: 0.0,
            actuator_motion: 0.0,
            power_demand: 0.0,
            time: 0.0,
        }
    }

    pub fn reset_at(&mut self, terrain: &impl GroundQuery, x: f32, z: f32, yaw: f32) {
        let fidelity = self.fidelity;
        *self = Self::new_at(self.spec, terrain, x, z, yaw, self.scale);
        self.fidelity = fidelity;
    }

    pub fn spec(&self) -> &WalkerSpec {
        &self.spec
    }

    pub fn scale(&self) -> f32 {
        self.scale
    }

    pub fn fidelity(&self) -> Fidelity {
        self.fidelity
    }

    /// Tiers share state, so switching mid-run is seamless (no teleports;
    /// feet keep their anchors and finish their swings).
    pub fn set_fidelity(&mut self, fidelity: Fidelity) {
        self.fidelity = fidelity;
    }

    /// World-space position.
    pub fn position(&self) -> Vec3 {
        self.pos * self.scale
    }

    /// World-space velocity (Froude: world speed = sqrt(scale) * canonical).
    pub fn velocity(&self) -> Vec3 {
        self.vel * self.scale.sqrt()
    }

    pub fn yaw(&self) -> f32 {
        self.yaw
    }

    pub fn cabin_tilt(&self) -> (f32, f32) {
        (self.pitch, self.roll)
    }

    /// Melee hook: instantaneous world-space velocity change (a shove, a
    /// parry impact, an explosion push). The balance loop turns it into
    /// recovery steps on its own.
    pub fn apply_impulse(&mut self, world_dv: Vec3) {
        self.vel += world_dv / self.scale.sqrt();
    }

    /// Rigidly shift the whole walker (body, feet, swing targets) and
    /// cancel velocity into `block_normal` — the host's wall-collision
    /// resolution. Shifting feet with the body keeps stance constraints
    /// from fighting the pushout; the gait replants naturally afterward.
    pub fn translate_and_block(&mut self, delta: Vec3, block_normal: Vec3) {
        let delta = delta / self.scale;
        self.pos += delta;
        self.prev_pos += delta;
        self.support_y += delta.y;
        for leg in &mut self.legs {
            leg.foot += delta;
            leg.anchor += delta;
            leg.swing_start += delta;
            leg.swing_target += delta;
        }
        if block_normal.length_squared() > 1.0e-6 {
            let inward = self.vel.dot(-block_normal);
            if inward > 0.0 {
                self.vel += block_normal * inward;
            }
        }
    }

    pub fn take_footfalls(&mut self) -> Vec<FootfallEvent> {
        let scale = self.scale;
        std::mem::take(&mut self.pending_footfalls)
            .into_iter()
            .map(|f| FootfallEvent {
                pos: f.pos * scale,
                strength: f.strength,
            })
            .collect()
    }

    pub fn hydraulic_load(&self) -> f32 {
        self.actuator_force
    }

    pub fn hydraulic_motion(&self) -> f32 {
        self.actuator_motion
    }

    pub fn power_demand(&self) -> f32 {
        self.power_demand
    }

    pub fn take_gait_report(&mut self) -> Option<String> {
        self.validation.pending_report.take()
    }

    pub fn validation(&self) -> &GaitValidation {
        &self.validation
    }

    /// Everything a pose layer needs, world space. See `RigSignals`.
    pub fn signals(&self) -> RigSignals {
        let s = self.scale;
        let sv = s.sqrt();
        let leg = |i: usize| LegSignal {
            contact: self.legs[i].is_stance(),
            swing_t: if self.legs[i].is_stance() {
                0.0
            } else {
                self.legs[i].swing_t
            },
            foot: self.legs[i].foot * s,
            target: self.legs[i].swing_target * s,
            side: self.legs[i].side,
        };
        RigSignals {
            pos: self.pos * s,
            vel: self.vel * sv,
            yaw: self.yaw,
            pitch: self.pitch,
            roll: self.roll,
            support_yaw: self.support_yaw(),
            capture_error: (self.capture_point() - self.support_center()) * s,
            balance: self.balance_vector() * s,
            control_risk: self.control_risk(self.vel),
            swing_wave: self.swing_wave(),
            legs: [leg(0), leg(1)],
        }
    }

    /// World-space IK joint positions for one leg (0 = left, 1 = right).
    pub fn leg_chain(&self, idx: usize) -> LegChain {
        let hip = self.hip_world(idx);
        let foot = self.legs[idx].foot + Vec3::Y * 0.10;
        let yaw_rot = Quat::from_rotation_y(self.yaw);
        let forward = yaw_rot * Vec3::Z;
        let right = yaw_rot * Vec3::X;
        let compound_len = if self.legs[idx].compound_ready {
            self.legs[idx].compound_len
        } else {
            desired_compound_len(hip, foot, &self.spec)
        };
        let (knee, hock, mid, end) = solve_chain_with_compound(
            hip,
            foot,
            forward,
            right,
            self.legs[idx].side,
            compound_len,
            &self.spec,
        );
        let s = self.scale;
        LegChain {
            hip: hip * s,
            knee: knee * s,
            hock: hock * s,
            mid: if self.spec.mid_link > 0.01 {
                Some(mid * s)
            } else {
                None
            },
            foot: end * s,
        }
    }

    /// Yaw implied by the support line (planned targets included), if the
    /// feet are far enough apart to define one.
    pub fn support_yaw(&self) -> Option<f32> {
        let pick = |leg: &Leg| {
            if leg.is_stance() {
                leg.anchor
            } else {
                leg.swing_target
            }
        };
        let left = pick(&self.legs[0]);
        let right = pick(&self.legs[1]);
        let lateral = Vec3::new(right.x - left.x, 0.0, right.z - left.z);
        if lateral.length_squared() < 0.10 {
            return None;
        }
        let right_axis = lateral.normalize();
        Some((-right_axis.z).atan2(right_axis.x))
    }

    // ------------------------------------------------------------------
    // Simulation step
    // ------------------------------------------------------------------

    pub fn step(&mut self, terrain: &impl GroundQuery, cmd: WalkerCommand, dt: f32) {
        // Froude scaling: canonical time runs at dt/sqrt(scale), so world
        // accelerations (incl. gravity) are scale-invariant while world
        // speeds/cadence scale by sqrt(scale) — a giant genuinely moves
        // ponderously rather than being a linearly-fast copy.
        let ground = ScaledGround {
            inner: terrain,
            scale: self.scale,
        };
        let terrain = &ground;
        let dt = dt / self.scale.sqrt();
        self.time += dt;

        let mut cmd = cmd;
        cmd.move_dir.y = 0.0;

        if self.fidelity == Fidelity::Kinematic {
            self.step_kinematic(terrain, cmd, dt);
            return;
        }

        self.plan_feet(terrain, cmd, dt);
        self.update_yaw(cmd, dt);

        let desired_speed = self.spec.speed.at(cmd.sprint);
        let raw_desired_vel = cmd.move_dir.normalize_or(Vec3::ZERO) * desired_speed;
        let desired_vel = self.stabilized_desired_vel(raw_desired_vel, cmd);
        self.update_ride_actuator(terrain, raw_desired_vel, desired_vel, desired_speed, dt);

        let substeps = self.fidelity.substeps();
        let sub_dt = dt / substeps as f32;
        for _ in 0..substeps {
            self.prev_pos = self.pos;
            let horiz = Vec3::new(self.vel.x, 0.0, self.vel.z);
            let accel = self.stance_traction_accel(desired_vel, horiz, cmd);
            let slip_accel = self.slip_stance_accel(cmd, desired_vel);
            self.vel += (Vec3::new(accel.x, GRAVITY, accel.z) + slip_accel) * sub_dt;
            self.pos += self.vel * sub_dt;

            self.solve_support_height(terrain, sub_dt);
            self.solve_lateral_balance(sub_dt);
            self.solve_stance_lateral(terrain);
            self.solve_leg_reach(terrain);
            self.keep_above_ground(terrain);

            self.vel = (self.pos - self.prev_pos) / sub_dt;
            let drag = (1.0 - self.spec.planar_damping * sub_dt).max(0.0);
            self.vel.x *= drag;
            self.vel.z *= drag;
            if cmd.move_dir.length_squared() < 0.0025 {
                let planar = Vec3::new(self.vel.x, 0.0, self.vel.z);
                if planar.length_squared() < 0.25 * 0.25 {
                    self.vel.x = 0.0;
                    self.vel.z = 0.0;
                }
            }
        }

        self.update_attitude(terrain, desired_vel, dt);
        self.update_leg_actuators(dt);
        self.update_power_audio_state(desired_vel);
        self.validate_gait(dt);
    }

    /// Kinematic tier: same state, no balance dynamics. Phase/timer-driven
    /// steps around the commanded velocity, ground-following body height,
    /// direct lean. Shares the swing/footfall machinery so switching tiers
    /// mid-run is seamless.
    fn step_kinematic(&mut self, terrain: &impl GroundQuery, cmd: WalkerCommand, dt: f32) {
        self.advance_swings(terrain, cmd.sprint, dt);
        self.update_yaw(cmd, dt);

        let input = cmd.move_dir.length().min(1.0);
        let move_dir = cmd.move_dir.normalize_or(Vec3::ZERO);
        let desired_vel = move_dir * self.spec.speed.at(cmd.sprint);

        // Body follows the command with a soft approach; no traction model.
        self.prev_pos = self.pos;
        let k = (dt * 6.0).clamp(0.0, 1.0);
        self.vel.x += (desired_vel.x - self.vel.x) * k;
        self.vel.z += (desired_vel.z - self.vel.z) * k;
        self.pos.x += self.vel.x * dt;
        self.pos.z += self.vel.z * dt;

        // Ground-follow via the same rate-limited ride-height motor.
        let mut sum = 0.0;
        let mut n = 0.0;
        for leg in &self.legs {
            if leg.is_stance() {
                sum += terrain.height_at(leg.anchor.x, leg.anchor.z);
                n += 1.0;
            }
        }
        let base = if n > 0.0 {
            sum / n
        } else {
            terrain.height_at(self.pos.x, self.pos.z)
        };
        let raw_target = base + self.spec.body_height();
        if !self.support_y_ready {
            self.support_y = self.pos.y;
            self.support_y_vel = 0.0;
            self.support_y_ready = true;
        }
        drive_scalar_motor(
            &mut self.support_y,
            &mut self.support_y_vel,
            raw_target,
            dt,
            2.4,
            9.0,
            6.4,
        );
        self.pos.y = self.support_y;
        self.vel.y = if dt > 0.0 {
            (self.pos.y - self.prev_pos.y) / dt
        } else {
            0.0
        };

        // Direct attitude: terrain lean + acceleration lean.
        let normal = terrain.normal_at(self.pos.x, self.pos.z);
        let terrain_pitch = normal.z.atan2(normal.y) * 0.7;
        let terrain_roll = -normal.x.atan2(normal.y) * 0.7;
        let accel = (desired_vel - Vec3::new(self.vel.x, 0.0, self.vel.z)) * 0.035;
        let local_accel = Quat::from_rotation_y(-self.yaw) * accel;
        let target_pitch = (terrain_pitch - local_accel.z.clamp(-0.20, 0.20)).clamp(-0.20, 0.20);
        let target_roll = (terrain_roll + local_accel.x.clamp(-0.18, 0.18)).clamp(-0.18, 0.18);
        let ka = (dt * 7.0).clamp(0.0, 1.0);
        let prev_pitch = self.pitch;
        let prev_roll = self.roll;
        self.pitch += (target_pitch - self.pitch) * ka;
        self.roll += (target_roll - self.roll) * ka;
        self.pitch_vel = if dt > 0.0 {
            (self.pitch - prev_pitch) / dt
        } else {
            0.0
        };
        self.roll_vel = if dt > 0.0 {
            (self.roll - prev_roll) / dt
        } else {
            0.0
        };

        // Stepping: strict alternation on a timer while moving; untwist /
        // leash re-plant when idle.
        self.step_timer = (self.step_timer - dt).max(0.0);
        let interval = self.spec.step_interval.at(cmd.sprint);
        let any_swing = self.legs.iter().any(|l| !l.is_stance());
        if !any_swing {
            let yaw_rot = Quat::from_rotation_y(self.yaw);
            let right = yaw_rot * Vec3::X;
            let forward = yaw_rot * Vec3::Z;
            let body_flat = Vec3::new(self.pos.x, 0.0, self.pos.z);
            if input >= 0.05 {
                if self.step_timer <= 0.0 {
                    let i = self.next_leg.min(1);
                    if self.legs[i].is_stance() && self.legs[i].cooldown <= 0.0 {
                        let side = self.legs[i].side;
                        let lead = (interval + self.spec.swing_duration.at(cmd.sprint)) * 0.55;
                        let nominal = body_flat
                            + right * (side * self.spec.step_width)
                            + desired_vel * lead
                            + move_dir * (self.spec.stride_target.at(cmd.sprint) * 0.20);
                        let target = terrain.point_at(nominal.x, nominal.z);
                        self.start_swing(i, target);
                        self.step_timer = interval;
                    }
                }
            } else {
                // Idle comfort re-plant: support-line twist or leash breach.
                let twist = self
                    .support_yaw()
                    .map(|sy| angle_delta(self.yaw, sy).abs())
                    .unwrap_or(0.0);
                let mut worst = None;
                for i in 0..2 {
                    if !self.legs[i].is_stance() || self.legs[i].cooldown > 0.0 {
                        continue;
                    }
                    let hip = self.hip_world(i);
                    let dev = Vec3::new(
                        hip.x - self.legs[i].anchor.x,
                        0.0,
                        hip.z - self.legs[i].anchor.z,
                    )
                    .length();
                    if worst.map_or(true, |(_, d)| dev > d) {
                        worst = Some((i, dev));
                    }
                }
                if let Some((i, dev)) = worst {
                    let need_untwist = twist > self.spec.untwist_gate + 0.05;
                    let need_home = dev > self.spec.stance_leash * 0.80;
                    if (need_untwist || need_home) && self.step_timer <= 0.0 {
                        let side = self.legs[i].side;
                        let other = self.legs[1 - i].anchor;
                        let nominal = Vec3::new(other.x, 0.0, other.z)
                            + right * (side * self.spec.untwist_pair_lateral)
                            + forward * self.spec.untwist_forward_bias;
                        let target = terrain.point_at(nominal.x, nominal.z);
                        self.start_swing(i, target);
                        self.step_timer = interval;
                    }
                }
            }
        }

        // Light actuator/audio signals.
        self.actuator_force =
            (self.support_y_vel.abs() * 0.30 + self.yaw_vel.abs() * 0.06).clamp(0.0, 1.0);
        self.actuator_motion = (self.support_y_vel * 0.22 + self.yaw_vel * 0.05).clamp(-1.0, 1.0);
        let speed_n = (Vec3::new(self.vel.x, 0.0, self.vel.z).length() / self.spec.speed.sprint)
            .clamp(0.0, 1.0);
        self.power_demand =
            (self.actuator_force * 0.30 + speed_n * 0.35 + input * 0.15).clamp(0.0, 1.0);
        self.validate_gait(dt);
    }

    // ------------------------------------------------------------------
    // Foot planning
    // ------------------------------------------------------------------

    /// Advance active swings (arc interpolation, landing, footfalls) and
    /// refresh stance anchors against the terrain.
    fn advance_swings(&mut self, terrain: &impl GroundQuery, sprint: bool, dt: f32) {
        let swing_duration = self.spec.swing_duration.at(sprint);
        let body_speed = self.vel.length();
        for i in 0..2 {
            let leg = &mut self.legs[i];
            leg.cooldown = (leg.cooldown - dt).max(0.0);
            match leg.state {
                LegState::Swing => {
                    leg.swing_t += dt / swing_duration;
                    if leg.swing_t >= 1.0 {
                        leg.swing_t = 1.0;
                        leg.state = LegState::Stance;
                        let step_len = Vec3::new(
                            leg.swing_target.x - leg.swing_start.x,
                            0.0,
                            leg.swing_target.z - leg.swing_start.z,
                        )
                        .length();
                        leg.anchor = terrain.point_at(leg.swing_target.x, leg.swing_target.z);
                        leg.foot = leg.anchor;
                        let strength =
                            (self.spec.footfall_base + step_len * 0.18 + body_speed * 0.045)
                                .clamp(0.25, 1.8);
                        let pos = leg.anchor;
                        leg.cooldown = self.spec.step_cooldown;
                        self.pending_footfalls.push(FootfallEvent { pos, strength });
                    } else {
                        let travel = swing_travel_t(leg.swing_t);
                        let p = leg.swing_start.lerp(leg.swing_target, travel);
                        let lift = (PI * leg.swing_t).sin() * self.spec.step_height.at(sprint);
                        leg.foot = terrain.point_at(p.x, p.z) + Vec3::Y * lift;
                    }
                }
                LegState::Stance => {
                    leg.anchor.y = terrain.height_at(leg.anchor.x, leg.anchor.z);
                    leg.foot = leg.anchor;
                }
            }
        }
    }

    fn plan_feet(&mut self, terrain: &impl GroundQuery, cmd: WalkerCommand, dt: f32) {
        self.advance_swings(terrain, cmd.sprint, dt);

        let move_dir = cmd.move_dir.normalize_or(Vec3::ZERO);
        let input = cmd.move_dir.length().min(1.0);
        let raw_move_vel = move_dir * self.spec.speed.at(cmd.sprint);
        let move_vel = self.stabilized_desired_vel(raw_move_vel, cmd);
        let yaw_rot = Quat::from_rotation_y(self.yaw);
        self.plan_step(terrain, cmd, input, move_dir, move_vel, yaw_rot, dt);
    }

    fn plan_step(
        &mut self,
        terrain: &impl GroundQuery,
        cmd: WalkerCommand,
        input: f32,
        move_dir: Vec3,
        desired_vel: Vec3,
        yaw_rot: Quat,
        dt: f32,
    ) {
        self.step_timer = (self.step_timer - dt).max(0.0);
        if self.legs.iter().any(|l| !l.is_stance()) {
            return;
        }
        let step_interval = self.spec.step_interval.at(cmd.sprint);
        let intent = self.intent(cmd, input, move_dir, desired_vel, yaw_rot);
        let balance = self.balance_vector();
        let balance_cost = self.com_balance_cost(self.pitch, self.roll);
        let control_risk = intent.recover_weight;
        let tipping = balance.length() > if cmd.sprint { 1.15 } else { 0.82 }
            || balance_cost > 0.20
            || control_risk > 0.60
            || self.pitch.abs() > 0.105
            || self.roll.abs() > 0.085;
        let intent = if tipping {
            Intent {
                recover_weight: 1.0,
                ..intent
            }
        } else {
            intent
        };
        if input < 0.05 && !intent.wants_recovery() && !intent.wants_untwist() {
            self.step_timer = 0.0;
            return;
        }

        let capture_ahead = intent.balance.dot(intent.forward);
        let stretch = self.forward_stretch(intent.forward);
        let must_step = capture_ahead > if cmd.sprint { 0.48 } else { 0.34 }
            || stretch > if cmd.sprint { 0.82 } else { 0.62 }
            || intent.untwist_weight >= 0.85;
        if self.step_timer > 0.0 && !must_step && !intent.wants_recovery() {
            return;
        }

        let Some(i) = (if intent.wants_recovery() {
            self.choose_recovery_leg(intent.balance.normalize_or(intent.forward))
        } else {
            Some(self.next_leg.min(1))
        }) else {
            return;
        };
        if (!self.legs[i].is_stance() || self.legs[i].cooldown > 0.0) && !intent.wants_recovery() {
            return;
        }
        if !self.legs[i].is_stance() {
            return;
        }
        let target = self.plan_foot_target(i, terrain, intent);
        self.start_swing(i, target);
        self.step_timer = if intent.wants_recovery() {
            step_interval * 0.55
        } else if intent.wants_untwist() {
            step_interval * 1.05
        } else {
            step_interval
        };
    }

    fn intent(
        &self,
        cmd: WalkerCommand,
        input: f32,
        move_dir: Vec3,
        desired_vel: Vec3,
        yaw_rot: Quat,
    ) -> Intent {
        let yaw_forward = yaw_rot * Vec3::Z;
        let untwist_weight = if input < 0.05 {
            self.support_yaw()
                .map(|sy| {
                    ((angle_delta(self.yaw, sy).abs() - self.spec.untwist_gate)
                        / self.spec.untwist_ramp)
                        .clamp(0.0, 1.0)
                })
                .unwrap_or(0.0)
        } else {
            0.0
        };
        let balance = self.balance_vector();
        let raw_desired_vel =
            cmd.move_dir.normalize_or(Vec3::ZERO) * self.spec.speed.at(cmd.sprint);
        let recover_weight = self
            .control_risk(raw_desired_vel)
            .max(self.control_risk(desired_vel) * 0.5);
        let commanded_forward = if input >= 0.05 { move_dir } else { yaw_forward };
        let recovery_forward = balance.normalize_or(commanded_forward);
        let recovery_blend = ((recover_weight - 0.35) / 0.55).clamp(0.0, 1.0);
        let forward = commanded_forward
            .lerp(recovery_forward, recovery_blend)
            .normalize_or(commanded_forward);
        let body_right = yaw_rot * Vec3::X;
        let mut travel_right = Vec3::new(forward.z, 0.0, -forward.x).normalize_or(body_right);
        if travel_right.dot(body_right) < 0.0 {
            travel_right = -travel_right;
        }
        let align = forward.dot(yaw_forward).abs();
        Intent {
            input,
            sprint: cmd.sprint,
            desired_vel,
            forward,
            right: body_right,
            travel_right,
            align_scale: 0.55 + 0.45 * align,
            balance,
            local_balance: Quat::from_rotation_y(-self.yaw) * balance,
            untwist_weight,
            recover_weight,
        }
    }

    fn plan_foot_target(&self, idx: usize, terrain: &impl GroundQuery, intent: Intent) -> Vec3 {
        let side = self.legs[idx].side;
        let hip = self.hip_world(idx);
        let hip_flat = Vec3::new(hip.x, 0.0, hip.z);
        let com = Vec3::new(self.pos.x, 0.0, self.pos.z);
        let capture = self.capture_point();
        let velocity_error = Vec3::new(
            self.vel.x - intent.desired_vel.x,
            0.0,
            self.vel.z - intent.desired_vel.z,
        );
        // Gait initiation: stride and commit grow with actual speed, so
        // the first steps out of a standstill are short instead of a full
        // stride the stance springs then fight (worst when launching
        // sideways, where traction authority is lowest).
        let speed_n = (Vec3::new(self.vel.x, 0.0, self.vel.z).length()
            / self.spec.speed.at(intent.sprint).max(0.1))
        .clamp(0.0, 1.0);
        let stride = self.spec.stride_target.at(intent.sprint)
            * (0.35 + 0.65 * speed_n)
            * intent.align_scale;
        let lateral = intent.right * (side * self.spec.step_width);
        let walk_nominal = {
            let raibert = com + intent.forward * stride + velocity_error * 0.045 + lateral;
            let capture_goal =
                Vec3::new(capture.x, 0.0, capture.z) + intent.forward * 0.58 + lateral;
            raibert.lerp(capture_goal, 0.34)
        };

        // Untwist re-plant: a comfortable pair position built off the
        // OTHER foot's anchor in the current (already magic-rotated) body
        // frame — the replacement for planned turning.
        let other = self.legs[1 - idx.min(1)].anchor;
        let untwist_nominal = Vec3::new(other.x, 0.0, other.z)
            + intent.right * (side * self.spec.untwist_pair_lateral)
            + intent.forward * self.spec.untwist_forward_bias;

        let recovery_axis = intent.balance.normalize_or(intent.forward);
        let support = self.support_center();
        let recovery_len = Vec3::new(capture.x - support.x, 0.0, capture.z - support.z).length();
        let recovery_push =
            (0.56 + recovery_len * 0.46 + intent.local_balance.length() * 0.14).clamp(0.62, 1.62);
        let recovery_nominal = Vec3::new(capture.x, 0.0, capture.z)
            + recovery_axis * recovery_push
            + intent.right * (side * 0.54 + intent.local_balance.x.clamp(-0.38, 0.38) * 0.35);

        let mut nominal = walk_nominal
            .lerp(untwist_nominal, intent.untwist_weight * 0.72)
            .lerp(recovery_nominal, intent.recover_weight);

        let foot = self.legs[idx].foot;
        let planned = Vec3::new(nominal.x - foot.x, 0.0, nominal.z - foot.z);
        let min_commit = self.spec.commit_distance.at(intent.sprint)
            * (0.30 + 0.70 * speed_n)
            * intent.align_scale;
        let downrange = planned.dot(intent.forward);
        if intent.input >= 0.05 && downrange < min_commit {
            nominal += intent.forward * (min_commit - downrange);
        }

        if self.fidelity == Fidelity::Reduced {
            // No candidate search: fold the cart-pole support offset in as
            // explicit feedforward instead (brake plants land ahead,
            // acceleration plants behind, slopes lean uphill).
            let n = terrain.normal_at(self.pos.x, self.pos.z);
            let grade = terrain_grade_accel(n);
            let horiz = Vec3::new(self.vel.x, 0.0, self.vel.z);
            let fast = self.spec.fast_twitch;
            let cap = if fast && intent.sprint {
                15.5
            } else if intent.sprint {
                9.6
            } else {
                6.5
            };
            let command_accel = (intent.desired_vel - horiz).clamp_length_max(cap);
            let required = (command_accel + grade).clamp_length_max(cap + 0.5);
            let omega_sq = 9.81 / self.spec.body_height();
            let offset = (required / omega_sq).clamp_length_max(if fast && intent.sprint {
                3.05
            } else if intent.sprint {
                1.34
            } else {
                0.96
            });
            nominal -= offset * 0.55;
            let clamped = self.clamp_target(nominal, hip_flat, side, intent);
            return terrain.point_at(clamped.x, clamped.z);
        }

        let fore_offsets: &[f32] = if intent.sprint {
            &[-0.36, -0.10, 0.18, 0.52, 0.88]
        } else if intent.wants_untwist() && intent.input < 0.05 {
            &[-0.30, 0.0, 0.30, 0.58]
        } else {
            &[-0.26, 0.0, 0.28, 0.58]
        };
        let lateral_offsets: &[f32] = if intent.wants_untwist() {
            &[-0.12, 0.0, 0.12]
        } else {
            &[-0.10, 0.0, 0.10]
        };
        let mut best = terrain.point_at(nominal.x, nominal.z);
        let mut best_cost = f32::INFINITY;
        for &fore in fore_offsets {
            for &lat in lateral_offsets {
                let raw = nominal + intent.forward * fore + intent.travel_right * lat;
                let clamped = self.clamp_target(raw, hip_flat, side, intent);
                let candidate = terrain.point_at(clamped.x, clamped.z);
                let cost = self.score_target(terrain, idx, candidate, nominal, intent);
                if cost < best_cost {
                    best_cost = cost;
                    best = candidate;
                }
            }
        }
        best
    }

    fn clamp_target(&self, candidate: Vec3, hip_flat: Vec3, side: f32, intent: Intent) -> Vec3 {
        // Clamp in the ORTHOGONAL travel frame (forward, travel_right) so
        // nothing degenerates when travel isn't aligned with facing. The
        // body-frame anti-cross guard lives in `score_target` spacing.
        let forward = intent.forward;
        let right = intent.travel_right;
        let rel = candidate - hip_flat;
        let (min_forward, max_forward) = if intent.input >= 0.05 {
            (
                self.spec.forward_min_move.at(intent.sprint),
                self.spec.forward_max_move.at(intent.sprint),
            )
        } else {
            self.spec.forward_limits_idle
        };
        let max_forward = max_forward * intent.align_scale;
        let forward_amount = rel.dot(forward).clamp(min_forward, max_forward);
        // Preferred lateral offset = the body-frame foot spread projected
        // onto the travel-perpendicular axis; the span relaxes as facing
        // and travel disalign (strafe needs body-fore-aft freedom).
        let alignment = intent.right.dot(right).abs();
        let lateral_target = side * 0.04 * alignment
            + (intent.right * (side * self.spec.step_width)).dot(right) * (1.0 - alignment) * 0.5;
        let lateral_span = 0.08
            + intent.untwist_weight * 0.16
            + intent.recover_weight * 0.06
            + (1.0 - alignment) * self.spec.step_width * 0.9;
        let lateral_amount = rel
            .dot(right)
            .clamp(lateral_target - lateral_span, lateral_target + lateral_span);
        let mut candidate = hip_flat + forward * forward_amount + right * lateral_amount;
        let to_target = candidate - hip_flat;
        let max_planar = if intent.input >= 0.05 {
            self.spec.max_planar_move.at(intent.sprint)
        } else {
            self.spec.max_planar_idle
        };
        if to_target.length() > max_planar {
            candidate = hip_flat + to_target.normalize_or(forward) * max_planar;
        }
        candidate
    }

    fn score_target(
        &self,
        terrain: &impl GroundQuery,
        idx: usize,
        candidate: Vec3,
        nominal: Vec3,
        intent: Intent,
    ) -> f32 {
        let forward = intent.forward;
        let travel_right = intent.travel_right;
        let target_flat = Vec3::new(candidate.x, 0.0, candidate.z);
        let nominal_flat = Vec3::new(nominal.x, 0.0, nominal.z);
        let distance_from_nominal = (target_flat - nominal_flat).length_squared();
        let capture = self.capture_point();
        let mut support = target_flat;
        let mut support_n = 1.0;
        let mut spacing_cost = 0.0;
        for j in 0..2 {
            if j == idx || !self.legs[j].is_stance() {
                continue;
            }
            let other = Vec3::new(self.legs[j].anchor.x, 0.0, self.legs[j].anchor.z);
            support += other;
            support_n += 1.0;
            // Anti-cross/clip guard is a BODY-frame quantity.
            let lateral_gap = (target_flat - other).dot(intent.right).abs();
            spacing_cost += (self.spec.min_pair_gap - lateral_gap).max(0.0).powi(2) * 7.0;
        }
        support /= support_n;
        let capture_error = capture - support;
        let balance_cost = capture_error.dot(forward).powi(2) * 0.35
            + capture_error.dot(travel_right).powi(2) * 1.25;
        let cart_pole_cost = self.cart_pole_cost(terrain, support, forward, travel_right, intent);
        let recovery_cost =
            self.recovery_balance_after_step_cost(idx, candidate) * 36.0 * intent.recover_weight;
        let foot = Vec3::new(self.legs[idx].foot.x, 0.0, self.legs[idx].foot.z);
        let step = target_flat - foot;
        let desired_step = self.spec.desired_step.at(intent.sprint) * intent.align_scale;
        let step_len = step.length();
        let short_step_cost =
            (desired_step - step_len).max(0.0).powi(2) * 0.35 * intent.input.max(0.25);
        let velocity_cost = (intent.desired_vel.length()
            - Vec3::new(self.vel.x, 0.0, self.vel.z).length())
        .max(0.0)
        .powi(2)
            * 0.05;
        let effort_cost = self.step_effort(idx, candidate);
        let posture_cost = self.chain_posture_cost_for(idx, candidate);
        let hip = self.hip_world(idx);
        let reach = hip.distance(candidate + Vec3::Y * 0.10);
        let reach_cost = (reach / (self.spec.max_reach * 0.96) - 1.0)
            .max(0.0)
            .powi(2)
            * 80.0;
        let impact_cost = (candidate.y - self.legs[idx].foot.y).abs().powi(2) * 3.0;
        distance_from_nominal * 2.0
            + balance_cost
            + cart_pole_cost
            + recovery_cost
            + spacing_cost
            + short_step_cost
            + velocity_cost
            + effort_cost * 0.60
            + posture_cost * self.spec.chain_posture_weight
            + reach_cost
            + impact_cost
    }

    /// Linear-inverted-pendulum feedforward: given the acceleration the
    /// command + terrain grade demand, where should the support be so the
    /// pendulum produces it? Scores candidate plants against that point.
    /// Closed-form and cheap — this is what makes braking from a sprint
    /// plant ahead and hill climbs plant behind.
    fn cart_pole_cost(
        &self,
        terrain: &impl GroundQuery,
        support: Vec3,
        forward: Vec3,
        right: Vec3,
        intent: Intent,
    ) -> f32 {
        let n = terrain.normal_at(support.x, support.z);
        let grade_accel = terrain_grade_accel(n);
        let horiz = Vec3::new(self.vel.x, 0.0, self.vel.z);
        let fast = self.spec.fast_twitch;
        let cap = if fast && intent.sprint {
            15.5
        } else if intent.sprint {
            9.6
        } else {
            6.5
        };
        let command_accel = (intent.desired_vel - horiz).clamp_length_max(cap);
        let required_accel =
            (command_accel + grade_accel).clamp_length_max(if fast && intent.sprint {
                16.0
            } else if intent.sprint {
                10.0
            } else {
                7.0
            });
        let omega_sq = 9.81 / self.spec.body_height();
        let support_offset =
            (required_accel / omega_sq).clamp_length_max(if fast && intent.sprint {
                3.05
            } else if intent.sprint {
                1.34
            } else {
                0.96
            });
        let predicted_com = self.com_projection() + horiz * 0.10;
        let desired_support = self.capture_point().lerp(predicted_com, 0.28) - support_offset;
        let err = support - desired_support;
        let grade_n = (grade_accel.length() / 4.4).clamp(0.0, 1.0);
        let speed_n = (horiz.length() / self.spec.speed.at(intent.sprint).max(0.1)).clamp(0.0, 1.0);
        let weight = 0.38 + grade_n * 0.82 + intent.recover_weight * 0.55 + speed_n * 0.18;
        (err.dot(forward).powi(2) * 0.50 + err.dot(right).powi(2) * 1.15) * weight
    }

    fn choose_recovery_leg(&self, recovery_dir: Vec3) -> Option<usize> {
        let mut has_ready_leg = false;
        for i in 0..2 {
            has_ready_leg |= self.legs[i].is_stance() && self.legs[i].cooldown <= 0.0;
        }
        let mut best = None;
        for i in 0..2 {
            if !self.legs[i].is_stance() {
                continue;
            }
            if has_ready_leg && self.legs[i].cooldown > 0.0 {
                continue;
            }
            let foot = Vec3::new(self.legs[i].anchor.x, 0.0, self.legs[i].anchor.z);
            let cooldown_penalty = if self.legs[i].cooldown > 0.0 {
                1.50
            } else {
                0.0
            };
            let alternation_penalty = if self.next_leg.min(1) == i { 0.0 } else { 0.75 };
            let score = foot.dot(recovery_dir) + cooldown_penalty + alternation_penalty;
            if best.map_or(true, |(_, s)| score < s) {
                best = Some((i, score));
            }
        }
        best.map(|(i, _)| i)
    }

    fn recovery_balance_after_step_cost(&self, idx: usize, candidate: Vec3) -> f32 {
        let mut support = Vec3::new(candidate.x, 0.0, candidate.z);
        let mut n = 1.0;
        for j in 0..2 {
            if j == idx || !self.legs[j].is_stance() {
                continue;
            }
            support += Vec3::new(self.legs[j].anchor.x, 0.0, self.legs[j].anchor.z);
            n += 1.0;
        }
        support /= n;
        let com = self.com_projection();
        let capture = self.capture_point();
        let local_com = Quat::from_rotation_y(-self.yaw) * (com - support);
        let local_capture = Quat::from_rotation_y(-self.yaw) * (capture - support);
        local_com.x.powi(2) * 2.0
            + local_com.z.powi(2) * 1.3
            + local_capture.x.powi(2) * 1.3
            + local_capture.z.powi(2) * 0.7
    }

    fn step_effort(&self, idx: usize, target: Vec3) -> f32 {
        let spec = &self.spec;
        // IK poles are a BODY-frame notion: knees bend with the facing,
        // not with the travel direction.
        let yaw_rot = Quat::from_rotation_y(self.yaw);
        let forward = yaw_rot * Vec3::Z;
        let right = yaw_rot * Vec3::X;
        let hip = self.hip_world(idx);
        let side = self.legs[idx].side;
        let current_target = self.legs[idx].foot + Vec3::Y * 0.10;
        let target = target + Vec3::Y * 0.10;
        let cur_compound = desired_compound_len(hip, current_target, spec);
        let next_compound = desired_compound_len(hip, target, spec);
        let (cur_front, cur_hock, cur_lower, cur_end) = solve_chain_with_compound(
            hip,
            current_target,
            forward,
            right,
            side,
            cur_compound,
            spec,
        );
        let (next_front, next_hock, next_lower, next_end) =
            solve_chain_with_compound(hip, target, forward, right, side, next_compound, spec);
        let mut effort = cur_front.distance_squared(next_front) / (spec.hip_link * spec.hip_link)
            + cur_hock.distance_squared(next_hock) / (spec.reverse_link * spec.reverse_link)
            + cur_end.distance_squared(next_end) / (spec.shin_link * spec.shin_link);
        if spec.mid_link > 0.01 {
            effort += cur_lower.distance_squared(next_lower) / (spec.mid_link * spec.mid_link);
        }
        effort
    }

    fn chain_posture_cost_for(&self, idx: usize, target: Vec3) -> f32 {
        let spec = &self.spec;
        if spec.chain_posture_weight <= 0.0 {
            return 0.0;
        }
        let yaw_rot = Quat::from_rotation_y(self.yaw);
        let forward = yaw_rot * Vec3::Z;
        let right = yaw_rot * Vec3::X;
        let hip = self.hip_world(idx);
        let target = target + Vec3::Y * 0.10;
        let compound = desired_compound_len(hip, target, spec);
        let (front_low, rear_hock, lower_hock, end) = solve_chain_with_compound(
            hip,
            target,
            forward,
            right,
            self.legs[idx].side,
            compound,
            spec,
        );
        if spec.mid_link > 0.01 {
            chain_posture_cost(
                &[hip, front_low, rear_hock, lower_hock, end],
                hip.distance(target) / spec.max_reach,
                spec.preferred_extension,
            )
        } else {
            chain_posture_cost(
                &[hip, front_low, rear_hock, end],
                hip.distance(target) / spec.max_reach,
                spec.preferred_extension,
            )
        }
    }

    fn forward_stretch(&self, forward: Vec3) -> f32 {
        let mut max_stretch = 0.0;
        for i in 0..2 {
            if !self.legs[i].is_stance() {
                continue;
            }
            let hip = self.hip_world(i);
            let delta = Vec3::new(
                hip.x - self.legs[i].anchor.x,
                0.0,
                hip.z - self.legs[i].anchor.z,
            );
            max_stretch = f32::max(max_stretch, delta.dot(forward));
        }
        max_stretch
    }

    fn start_swing(&mut self, idx: usize, target: Vec3) {
        let step_len = {
            let leg = &mut self.legs[idx];
            let step_len = Vec3::new(target.x - leg.foot.x, 0.0, target.z - leg.foot.z).length();
            leg.state = LegState::Swing;
            leg.swing_t = 0.0;
            leg.swing_start = leg.foot;
            leg.swing_target = target;
            step_len
        };
        self.validation.record_step(idx, step_len);
        self.next_leg = 1 - idx.min(1);
    }

    // ------------------------------------------------------------------
    // Body control
    // ------------------------------------------------------------------

    /// "Magic" facing: a spring-damper on body yaw toward the commanded
    /// facing. Feet never plan rotation; they follow via untwist replants.
    fn update_yaw(&mut self, cmd: WalkerCommand, dt: f32) {
        let err = angle_delta(cmd.face_yaw, self.yaw);
        self.yaw_vel += err * self.spec.yaw_stiffness * dt;
        self.yaw_vel *= (1.0 - self.spec.yaw_damping * dt).max(0.0);
        self.yaw += self.yaw_vel * dt;
    }

    fn update_ride_actuator(
        &mut self,
        terrain: &impl GroundQuery,
        raw_desired_vel: Vec3,
        desired_vel: Vec3,
        desired_speed: f32,
        dt: f32,
    ) {
        let speed = Vec3::new(self.vel.x, 0.0, self.vel.z).length();
        let speed_n = (speed / desired_speed.max(0.1)).clamp(0.0, 1.0);
        let swing_load = self.swing_wave();
        let control_risk = self
            .control_risk(raw_desired_vel)
            .max(self.control_risk(desired_vel));
        let terrain_challenge = self.terrain_challenge(terrain, desired_vel);
        let target = (-0.10 * speed_n + swing_load * (0.05 + speed_n * 0.07)
            - control_risk * 0.24
            - terrain_challenge * 0.14)
            .clamp(-0.42, 0.14);
        let k = (dt * (2.2 + speed_n * 1.8 + control_risk * 4.0 + terrain_challenge * 2.0))
            .clamp(0.0, 1.0);
        self.ride_offset += (target - self.ride_offset) * k;
    }

    fn update_attitude(&mut self, terrain: &impl GroundQuery, desired_vel: Vec3, dt: f32) {
        let (target_pitch, target_roll) = self.attitude_target(terrain, desired_vel);
        let sprint_like = desired_vel.length() > self.spec.sprint_like_speed;
        let (pitch_motor, roll_motor) = if sprint_like {
            (self.spec.pitch_motor_fast, self.spec.roll_motor_fast)
        } else {
            (self.spec.pitch_motor, self.spec.roll_motor)
        };
        drive_axis_motor(
            &mut self.pitch,
            &mut self.pitch_vel,
            target_pitch,
            dt,
            pitch_motor,
        );
        drive_axis_motor(
            &mut self.roll,
            &mut self.roll_vel,
            target_roll,
            dt,
            roll_motor,
        );
    }

    /// Direct attitude target: terrain conformance + balance lean + the
    /// cart-pole acceleration lean. (The original also ran a 7x7 candidate
    /// search with predicted motor rollouts on top of this base; the rate
    /// limits in the axis motors cover jerk well enough without it.)
    fn attitude_target(&self, terrain: &impl GroundQuery, desired_vel: Vec3) -> (f32, f32) {
        let fast = self.spec.fast_twitch;
        let n = terrain.normal_at(self.pos.x, self.pos.z);
        let terrain_pitch = n.z.atan2(n.y) * 0.22;
        let terrain_roll = -n.x.atan2(n.y) * 0.22;
        let terrain_challenge = self.terrain_challenge(terrain, desired_vel);
        let capture_error = self.capture_point() - self.support_center();
        let local_capture = Quat::from_rotation_y(-self.yaw) * capture_error;
        let balance_pitch = (-local_capture.z * if fast { 0.0075 } else { 0.010 }).clamp(
            -if fast { 0.058 } else { 0.044 },
            if fast { 0.058 } else { 0.044 },
        );
        let balance_roll = (local_capture.x * if fast { 0.011 } else { 0.015 }).clamp(
            -if fast { 0.052 } else { 0.042 },
            if fast { 0.052 } else { 0.042 },
        );
        let horiz = Vec3::new(self.vel.x, 0.0, self.vel.z);
        let sprint_like = desired_vel.length() > self.spec.sprint_like_speed;
        let command_accel = (desired_vel - horiz).clamp_length_max(if fast && sprint_like {
            15.5
        } else if sprint_like {
            9.4
        } else {
            8.0
        });
        let required_accel =
            (command_accel + terrain_grade_accel(n)).clamp_length_max(if fast && sprint_like {
                16.0
            } else if sprint_like {
                10.0
            } else {
                8.8
            });
        let local_accel = Quat::from_rotation_y(-self.yaw) * required_accel;
        let cart_pitch = (-(local_accel.z / 9.81) * if fast { 0.24 } else { 0.19 }).clamp(
            -if fast { 0.052 } else { 0.036 },
            if fast { 0.052 } else { 0.036 },
        );
        let cart_roll = ((local_accel.x / 9.81) * if fast { 0.19 } else { 0.15 }).clamp(
            -if fast { 0.042 } else { 0.030 },
            if fast { 0.042 } else { 0.030 },
        );
        let pitch_limit = if fast { 0.104 } else { 0.070 }
            + terrain_challenge * 0.016
            + if sprint_like { 0.006 } else { 0.0 };
        let roll_limit = if fast { 0.082 } else { 0.058 }
            + terrain_challenge * 0.012
            + if sprint_like { 0.005 } else { 0.0 };
        (
            (terrain_pitch + balance_pitch + cart_pitch).clamp(-pitch_limit, pitch_limit),
            (terrain_roll + balance_roll + cart_roll).clamp(-roll_limit, roll_limit),
        )
    }

    // ------------------------------------------------------------------
    // Stance dynamics
    // ------------------------------------------------------------------

    fn stance_traction_accel(&self, desired_vel: Vec3, horiz: Vec3, cmd: WalkerCommand) -> Vec3 {
        let stance_count = self.stance_count();
        if stance_count == 0 {
            return Vec3::ZERO;
        }
        let input = cmd.move_dir.length().min(1.0);
        let yaw_rot = Quat::from_rotation_y(self.yaw);
        let forward = yaw_rot * Vec3::Z;
        let right = yaw_rot * Vec3::X;
        let local_vel = Quat::from_rotation_y(-self.yaw) * horiz;
        let local_goal = if input < 0.05 {
            Vec3::ZERO
        } else {
            Quat::from_rotation_y(-self.yaw) * desired_vel
        };
        let params = self.spec.traction(cmd.sprint);
        let gain = self.spec.drive_gain.at(cmd.sprint);
        let forward_gain = if input < 0.05 { 5.2 } else { gain };
        let lateral_gain = if input < 0.05 { 6.4 } else { gain * 1.7 };
        let requested = forward * ((local_goal.z - local_vel.z) * forward_gain)
            + right * ((local_goal.x - local_vel.x) * lateral_gain);
        let requested_len = requested.length();
        if requested_len <= 1.0e-5 {
            return Vec3::ZERO;
        }
        let braking = requested.dot(horiz) < -0.05;

        let mut forward_capacity = 0.0;
        let mut lateral_capacity = 0.0;
        for i in 0..2 {
            if !self.legs[i].is_stance() {
                continue;
            }
            let (normal_load, extension) = self.stance_contact_load(i, cmd, desired_vel);
            let reach_factor =
                (1.0 - ((extension - 0.82) / 0.20).clamp(0.0, 1.0) * 0.45).clamp(0.45, 1.0);
            let normal_boost = if braking {
                params.brake_normal_boost
            } else {
                1.0
            };
            let force_boost = if braking {
                params.brake_force_boost
            } else {
                1.0
            };
            let friction_limit = normal_load * normal_boost * params.friction * reach_factor;
            let actuator_limit = params.max_force * force_boost * reach_factor;
            let limit = friction_limit.min(actuator_limit).max(0.0);
            forward_capacity += limit * params.forward_share;
            lateral_capacity += limit * params.lateral_share;
        }
        if forward_capacity <= 1.0e-5 && lateral_capacity <= 1.0e-5 {
            return Vec3::ZERO;
        }

        let requested_forward = requested.dot(forward);
        let requested_lateral = requested.dot(right);
        forward * requested_forward.clamp(-forward_capacity, forward_capacity)
            + right * requested_lateral.clamp(-lateral_capacity, lateral_capacity)
    }

    fn stance_contact_load(&self, idx: usize, cmd: WalkerCommand, desired_vel: Vec3) -> (f32, f32) {
        let leg = &self.legs[idx];
        let hip = self.hip_world(idx);
        let delta = hip - leg.anchor;
        let len = delta.length();
        if len <= 1.0e-4 {
            return (0.0, 1.0);
        }
        let params = self.spec.slip(cmd.sprint);
        let speed = Vec3::new(self.vel.x, 0.0, self.vel.z).length();
        let speed_n = (speed / self.spec.speed.sprint).clamp(0.0, 1.0);
        let command_n = (desired_vel.length() / self.spec.speed.sprint).clamp(0.0, 1.0);
        let rest_len = (self.target_stance_len()
            + params.rest_bias
            + params.speed_rest_bias * speed_n
            + params.command_rest_bias * command_n)
            .min(self.spec.max_reach * 0.965);
        let dir = delta / len;
        let radial_vel = self.vel.dot(dir);
        let compression = (rest_len - len).clamp(0.0, params.max_compression);
        let preload = params.preload * (compression / params.preload_window).clamp(0.0, 1.0);
        let spring_force = (preload + params.stiffness * compression - params.damping * radial_vel)
            .clamp(0.0, params.max_force);
        let stance_share = 1.0 / self.stance_count().max(1) as f32;
        let projected_support = (spring_force * dir.y.max(0.0)).max(0.0);
        let normal_load = projected_support + 9.81 * stance_share * 0.75;
        (normal_load, (len / self.spec.max_reach).clamp(0.0, 1.25))
    }

    fn slip_stance_accel(&self, cmd: WalkerCommand, desired_vel: Vec3) -> Vec3 {
        let stance_count = self.stance_count();
        if stance_count == 0 {
            return Vec3::ZERO;
        }
        let params = self.spec.slip(cmd.sprint);
        let speed = Vec3::new(self.vel.x, 0.0, self.vel.z).length();
        let speed_n = (speed / self.spec.speed.sprint).clamp(0.0, 1.0);
        let command_n = (desired_vel.length() / self.spec.speed.sprint).clamp(0.0, 1.0);
        let mut accel = Vec3::ZERO;
        let share = 1.0 / stance_count as f32;
        for i in 0..2 {
            let leg = &self.legs[i];
            if !leg.is_stance() {
                continue;
            }
            let hip = self.hip_world(i);
            let delta = hip - leg.anchor;
            let len = delta.length();
            if len <= 1.0e-4 {
                continue;
            }
            let dir = delta / len;
            let radial_vel = self.vel.dot(dir);
            let rest_len = (self.target_stance_len()
                + params.rest_bias
                + params.speed_rest_bias * speed_n
                + params.command_rest_bias * command_n)
                .min(self.spec.max_reach * 0.965);
            let compression = (rest_len - len).clamp(0.0, params.max_compression);
            let preload = params.preload * (compression / params.preload_window).clamp(0.0, 1.0);
            let force = (preload + params.stiffness * compression - params.damping * radial_vel)
                .clamp(0.0, params.max_force);
            accel += dir * (force * share);
        }
        accel
    }

    /// Blends the command toward a recovery goal when control risk is
    /// high — the "stumbling, catch yourself first" behavior.
    fn stabilized_desired_vel(&self, desired_vel: Vec3, cmd: WalkerCommand) -> Vec3 {
        let risk = self.control_risk(desired_vel);
        if risk <= 0.02 {
            return desired_vel;
        }
        let balance = self.balance_vector();
        let horiz = Vec3::new(self.vel.x, 0.0, self.vel.z);
        let mut recovery_dir = balance.normalize_or(Vec3::ZERO);
        if recovery_dir.length_squared() < 0.01 {
            recovery_dir = horiz.normalize_or(Vec3::ZERO);
        }
        if recovery_dir.length_squared() < 0.01 {
            return desired_vel;
        }
        let desired_len = desired_vel.length();
        let desired_dir = desired_vel.normalize_or(Vec3::ZERO);
        let actual_dir = horiz.normalize_or(Vec3::ZERO);
        let wrong_way = if desired_len > 0.20 && horiz.length() > 0.20 {
            (-actual_dir.dot(desired_dir)).max(0.0)
        } else {
            0.0
        };
        let opposed = if desired_len > 0.20 {
            (-desired_dir.dot(recovery_dir)).max(0.0)
        } else {
            0.45
        };
        let speed_cap = self.spec.speed.at(cmd.sprint).max(5.2);
        let hard_reverse = wrong_way > 0.45 && opposed > 0.45;
        let recovery_speed = if hard_reverse {
            0.8 + risk * 1.0
        } else {
            2.8 + risk * 2.8
        }
        .min(speed_cap);
        let recovery_goal = recovery_dir * recovery_speed;
        let blend = if hard_reverse {
            (risk * (0.72 + wrong_way * 0.20)).clamp(0.0, 0.88)
        } else {
            (risk * (0.38 + opposed * 0.48)).clamp(0.0, 0.82)
        };
        desired_vel
            .lerp(recovery_goal, blend)
            .clamp_length_max(speed_cap)
    }

    fn control_risk(&self, desired_vel: Vec3) -> f32 {
        let horiz = Vec3::new(self.vel.x, 0.0, self.vel.z);
        let desired_len = desired_vel.length();
        let desired_dir = desired_vel.normalize_or(Vec3::ZERO);
        let actual_dir = horiz.normalize_or(Vec3::ZERO);
        let balance = self.balance_vector();
        let balance_dir = balance.normalize_or(Vec3::ZERO);
        let wrong_way = if desired_len > 0.20 && horiz.length() > 0.20 {
            (-actual_dir.dot(desired_dir)).max(0.0)
        } else {
            0.0
        };
        let command_opposes_balance = if desired_len > 0.20 && balance.length() > 0.25 {
            (-desired_dir.dot(balance_dir)).max(0.0)
        } else {
            0.0
        };
        let speed_ref = self.spec.speed.sprint.max(0.1);
        let idle_drift = if desired_len <= 0.10 {
            ((horiz.length() / speed_ref - 0.18) / 0.45).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let omega = (9.81 / self.spec.body_height()).sqrt();
        let capture_residual = ((horiz - desired_vel) / omega).length();
        let residual_risk = ((capture_residual - 1.05) / 2.15).clamp(0.0, 1.0)
            * wrong_way
                .max(command_opposes_balance * 0.85)
                .max(idle_drift);
        let balance_margin = self.com_balance_cost(self.pitch, self.roll).sqrt();
        let expected_margin = if desired_len > self.spec.sprint_like_speed {
            self.spec.risk_margin.2
        } else if desired_len > 0.10 {
            self.spec.risk_margin.1
        } else {
            self.spec.risk_margin.0
        };
        let margin_risk = ((balance_margin - expected_margin) / 0.65).clamp(0.0, 1.0);
        let attitude_pressure = (self.pitch.abs() / 0.12).max(self.roll.abs() / 0.10);
        let attitude_risk = ((attitude_pressure - 1.0) / 0.55).clamp(0.0, 1.0);
        residual_risk.max(margin_risk).max(attitude_risk)
    }

    fn terrain_challenge(&self, terrain: &impl GroundQuery, desired_vel: Vec3) -> f32 {
        let horiz = Vec3::new(self.vel.x, 0.0, self.vel.z);
        let yaw_forward = Quat::from_rotation_y(self.yaw) * Vec3::Z;
        let dir = desired_vel
            .normalize_or(horiz.normalize_or(yaw_forward))
            .normalize_or(yaw_forward);
        let base_h = terrain.height_at(self.pos.x, self.pos.z);
        let mut challenge: f32 = 0.0;
        for dist in [0.9, 1.8] {
            let p = self.pos + dir * dist;
            let h = terrain.height_at(p.x, p.z);
            let n = terrain.normal_at(p.x, p.z);
            let height_cost = ((h - base_h).abs() - 0.18) / 0.55;
            let slope_cost = ((1.0 - n.y) - 0.035) / 0.18;
            challenge = challenge.max(height_cost.clamp(0.0, 1.0));
            challenge = challenge.max(slope_cost.clamp(0.0, 1.0));
        }
        challenge
    }

    // ------------------------------------------------------------------
    // Constraint stack (XPBD-style position projections)
    // ------------------------------------------------------------------

    fn target_stance_len(&self) -> f32 {
        (self.spec.stance_len + self.ride_offset)
            .clamp(self.spec.stance_len * 0.88, self.spec.max_reach * 0.93)
    }

    fn solve_support_height(&mut self, terrain: &impl GroundQuery, dt: f32) {
        let rot = self.body_rotation();
        let target_len = self.target_stance_len();
        let mut sum = 0.0;
        let mut n = 0.0;
        for leg in &self.legs {
            if !leg.is_stance() {
                continue;
            }
            let hip_offset = rot * leg.hip_local;
            let hip_flat = Vec3::new(self.pos.x + hip_offset.x, 0.0, self.pos.z + hip_offset.z);
            let foot_flat = Vec3::new(leg.anchor.x, 0.0, leg.anchor.z);
            let planar = hip_flat.distance(foot_flat);
            let vertical = (target_len * target_len - planar * planar)
                .max(0.45 * 0.45)
                .sqrt();
            sum += leg.anchor.y + vertical - hip_offset.y;
            n += 1.0;
        }
        let fallback =
            terrain.height_at(self.pos.x, self.pos.z) + self.spec.body_height() + self.ride_offset;
        let raw_target = if n > 0.0 { sum / n } else { fallback };
        if !self.support_y_ready {
            self.support_y = self.pos.y;
            self.support_y_vel = 0.0;
            self.support_y_ready = true;
        }
        let speed = Vec3::new(self.vel.x, 0.0, self.vel.z).length();
        let speed_n = (speed / self.spec.speed.sprint).clamp(0.0, 1.0);
        let fast = self.spec.fast_twitch;
        let max_speed = if fast {
            1.35 + speed_n * 1.25
        } else {
            0.88 + speed_n * 0.80
        };
        let max_accel = if fast {
            6.4 + speed_n * 8.2
        } else {
            4.0 + speed_n * 5.0
        };
        drive_scalar_motor(
            &mut self.support_y,
            &mut self.support_y_vel,
            raw_target,
            dt,
            max_speed,
            max_accel,
            6.8,
        );
        let c = self.pos.y - self.support_y;
        let compliance = 1.2e-4 + self.swing_wave() * 8.0e-5;
        self.project_scalar(c, Vec3::Y, compliance, dt);
    }

    fn solve_lateral_balance(&mut self, dt: f32) {
        let right = Quat::from_rotation_y(self.yaw) * Vec3::X;
        let speed = Vec3::new(self.vel.x, 0.0, self.vel.z).length();
        let speed_n = (speed / self.spec.speed.sprint).clamp(0.0, 1.0);
        let mut stance_count = 0;
        let mut stance_leg = 0;
        for i in 0..2 {
            if self.legs[i].is_stance() {
                stance_count += 1;
                stance_leg = i;
            }
        }
        match stance_count {
            1 => {
                let leg = &self.legs[stance_leg];
                let target = -leg.side * (self.spec.step_width + 0.08 + speed_n * 0.04);
                let c = (self.pos - leg.anchor).dot(right) - target;
                self.project_scalar_limited(c, right, 2.0e-2, dt, 0.00028 + speed_n * 0.00018);
            }
            2 => {
                let support = self.support_center();
                let c = (self.pos - support).dot(right);
                self.project_scalar_limited(c, right, 3.0e-2, dt, 0.00018 + speed_n * 0.00010);
            }
            _ => {}
        }
    }

    fn solve_stance_lateral(&mut self, terrain: &impl GroundQuery) {
        let right = Quat::from_rotation_y(self.yaw) * Vec3::X;
        let max_lateral = self.spec.max_lateral;
        for i in 0..2 {
            if !self.legs[i].is_stance() {
                continue;
            }
            let hip = self.hip_world(i);
            let delta = Vec3::new(
                hip.x - self.legs[i].anchor.x,
                0.0,
                hip.z - self.legs[i].anchor.z,
            );
            let lateral = delta.dot(right);
            let excess = lateral.abs() - max_lateral;
            if excess <= 0.0 {
                continue;
            }
            let slip = excess.min(0.045);
            self.legs[i].anchor += right * lateral.signum() * slip;
            self.legs[i].anchor.y = terrain.height_at(self.legs[i].anchor.x, self.legs[i].anchor.z);
            self.legs[i].foot = self.legs[i].anchor;
        }
    }

    fn solve_leg_reach(&mut self, terrain: &impl GroundQuery) {
        for i in 0..2 {
            let max_reach = self.spec.max_reach;
            let hip = self.hip_world(i);
            let d = hip - self.legs[i].foot;
            let len = d.length();
            if len <= max_reach || len < 1.0e-5 {
                self.legs[i].compression = (max_reach - len).max(0.0);
                continue;
            }
            let c = len - max_reach;
            self.validation.reach_violations += 1;
            self.validation.total_reach_violations += 1;
            if self.legs[i].is_stance() {
                let planar = Vec3::new(d.x, 0.0, d.z);
                if planar.length_squared() > 1.0e-5 {
                    let slip = (c * 0.55).clamp(0.0, 0.12);
                    let dir = planar.normalize();
                    self.legs[i].anchor += dir * slip;
                    self.legs[i].anchor.y =
                        terrain.height_at(self.legs[i].anchor.x, self.legs[i].anchor.z);
                    self.legs[i].foot = self.legs[i].anchor;
                }
            }
            self.legs[i].compression = 0.0;
        }
    }

    fn keep_above_ground(&mut self, terrain: &impl GroundQuery) {
        let floor = terrain.height_at(self.pos.x, self.pos.z) + 1.15;
        if self.pos.y < floor {
            self.pos.y = floor;
            self.vel.y = self.vel.y.max(0.0);
        }
    }

    fn project_scalar(&mut self, c: f32, grad: Vec3, compliance: f32, dt: f32) {
        self.project_scalar_limited(c, grad, compliance, dt, f32::INFINITY);
    }

    fn project_scalar_limited(
        &mut self,
        c: f32,
        grad: Vec3,
        compliance: f32,
        dt: f32,
        max_correction: f32,
    ) {
        let w = 1.0;
        let alpha = compliance / (dt * dt);
        let denom = w * grad.length_squared() + alpha;
        if denom <= 1.0e-8 {
            return;
        }
        let dlambda = -c / denom;
        let mut correction = grad * (w * dlambda);
        let len = correction.length();
        if len > max_correction {
            correction *= max_correction / len;
        }
        self.pos += correction;
    }

    // ------------------------------------------------------------------
    // Actuator / audio state
    // ------------------------------------------------------------------

    fn update_leg_actuators(&mut self, dt: f32) {
        let pos = self.pos;
        let rot = self.body_rotation();
        let mut force_sum = 0.0;
        let mut power_sum = 0.0;
        let mut motion_sum = 0.0;
        let mut samples = 0.0;
        for i in 0..2 {
            let hip = pos + rot * self.legs[i].hip_local;
            let foot = self.legs[i].foot + Vec3::Y * 0.10;
            let spec = self.spec;
            let desired = desired_compound_len(hip, foot, &spec);
            let min_needed = needed_compound_len(hip, foot, &spec);
            let fast = spec.fast_twitch;
            let stance = self.legs[i].is_stance();
            let max_speed = if fast {
                if stance {
                    3.2
                } else {
                    6.8
                }
            } else if stance {
                2.1
            } else {
                4.2
            };
            let max_accel = if fast {
                if stance {
                    13.0
                } else {
                    24.0
                }
            } else if stance {
                8.5
            } else {
                15.0
            };
            let leg = &mut self.legs[i];
            if !leg.compound_ready {
                leg.compound_len = desired;
                leg.compound_vel = 0.0;
                leg.compound_ready = true;
                continue;
            }
            let target = desired.max(min_needed);
            let desired_vel = ((target - leg.compound_len) * 9.0).clamp(-max_speed, max_speed);
            let dv = (desired_vel - leg.compound_vel).clamp(-max_accel * dt, max_accel * dt);
            let accel = if dt > 0.0 { dv / dt } else { 0.0 };
            let extension_error = (target - leg.compound_len).abs();
            let force = extension_error * 4.5 + accel.abs() * 0.085 + desired_vel.abs() * 0.12;
            force_sum += force;
            power_sum += force * (leg.compound_vel.abs() + desired_vel.abs() * 0.35);
            motion_sum += (leg.compound_vel + desired_vel * 0.45) * force.max(0.05);
            samples += 1.0;
            leg.compound_vel += dv;
            leg.compound_len = (leg.compound_len + leg.compound_vel * dt)
                .max(min_needed)
                .clamp(compound_min_len(&spec), compound_max_len(&spec));
        }
        let leg_force = if samples > 0.0 {
            force_sum / samples
        } else {
            0.0
        };
        let leg_power = if samples > 0.0 {
            power_sum / samples
        } else {
            0.0
        };
        let leg_motion = if force_sum > 0.001 {
            motion_sum / force_sum
        } else {
            0.0
        };
        let body_force = self.support_y_vel.abs() * 0.36
            + self.pitch_vel.abs() * 0.14
            + self.roll_vel.abs() * 0.14
            + self.yaw_vel.abs() * 0.06;
        self.actuator_force = (leg_force * 0.23 + body_force).clamp(0.0, 1.0);
        self.actuator_motion = (leg_motion * 0.30
            + self.support_y_vel * 0.26
            + self.pitch_vel * 0.05
            + self.roll_vel * 0.05)
            .clamp(-1.0, 1.0);
        self.power_demand = self
            .power_demand
            .max((leg_power * 0.055 + body_force * 0.55).clamp(0.0, 1.0));
    }

    fn update_power_audio_state(&mut self, desired_vel: Vec3) {
        let speed = Vec3::new(self.vel.x, 0.0, self.vel.z).length();
        let speed_n = (speed / self.spec.speed.sprint).clamp(0.0, 1.0);
        let drive_n = (desired_vel.length() / self.spec.speed.sprint).clamp(0.0, 1.0);
        let angular =
            (self.yaw_vel.abs() * 0.08 + self.pitch_vel.abs() * 0.08 + self.roll_vel.abs() * 0.08)
                .clamp(0.0, 0.35);
        let target = (self.power_demand * 0.58
            + self.actuator_force * 0.28
            + speed_n * 0.24
            + drive_n * 0.16
            + angular)
            .clamp(0.0, 1.0);
        self.power_demand = target;
    }

    // ------------------------------------------------------------------
    // Balance quantities
    // ------------------------------------------------------------------

    fn com_balance_cost(&self, pitch: f32, roll: f32) -> f32 {
        let support = self.support_center();
        let rot = Quat::from_rotation_y(self.yaw)
            * Quat::from_rotation_z(roll)
            * Quat::from_rotation_x(pitch);
        let com = self.pos + rot * self.spec.com_offset;
        let err = Vec3::new(com.x - support.x, 0.0, com.z - support.z);
        let local = Quat::from_rotation_y(-self.yaw) * err;
        let double = self.stance_count() > 1;
        let forward_margin = if double {
            self.spec.balance_margin_forward.0
        } else {
            self.spec.balance_margin_forward.1
        };
        let lateral_margin = if double {
            self.spec.balance_margin_lateral.0
        } else {
            self.spec.balance_margin_lateral.1
        };
        let forward_excess = (local.z.abs() - forward_margin).max(0.0);
        let lateral_excess = (local.x.abs() - lateral_margin).max(0.0);
        forward_excess.powi(2) * 1.3 + lateral_excess.powi(2) * 2.0
    }

    fn com_projection(&self) -> Vec3 {
        let com = self.pos + self.body_rotation() * self.spec.com_offset;
        Vec3::new(com.x, 0.0, com.z)
    }

    fn balance_vector(&self) -> Vec3 {
        let support = self.support_center();
        let com_error = self.com_projection() - support;
        let capture_error = self.capture_point() - support;
        Vec3::new(
            com_error.x * 0.35 + capture_error.x * 0.65,
            0.0,
            com_error.z * 0.35 + capture_error.z * 0.65,
        )
    }

    fn capture_point(&self) -> Vec3 {
        let omega = (9.81 / self.spec.body_height()).sqrt();
        Vec3::new(self.pos.x, 0.0, self.pos.z) + Vec3::new(self.vel.x, 0.0, self.vel.z) / omega
    }

    fn support_center(&self) -> Vec3 {
        let mut sum = Vec3::ZERO;
        let mut n = 0.0;
        for leg in &self.legs {
            if leg.is_stance() {
                sum += Vec3::new(leg.anchor.x, 0.0, leg.anchor.z);
                n += 1.0;
            }
        }
        if n > 0.0 {
            sum / n
        } else {
            Vec3::new(self.pos.x, 0.0, self.pos.z)
        }
    }

    fn stance_count(&self) -> usize {
        self.legs.iter().filter(|l| l.is_stance()).count()
    }

    fn swing_count(&self) -> usize {
        self.legs.iter().filter(|l| !l.is_stance()).count()
    }

    fn swing_wave(&self) -> f32 {
        let mut wave: f32 = 0.0;
        for leg in &self.legs {
            if !leg.is_stance() {
                wave = wave.max((PI * leg.swing_t).sin().max(0.0));
            }
        }
        wave
    }

    fn validate_gait(&mut self, dt: f32) {
        let capture_error = (self.capture_point() - self.support_center()).length();
        let speed = Vec3::new(self.vel.x, 0.0, self.vel.z).length();
        let balance_error = self.com_balance_cost(self.pitch, self.roll).sqrt();
        let vertical_jerk = ((self.vel.y - self.prev_y_vel) / dt).abs();
        let angular_jerk = (((self.pitch_vel - self.prev_pitch_vel) / dt).powi(2)
            + ((self.roll_vel - self.prev_roll_vel) / dt).powi(2))
        .sqrt();
        let cabin_jerk = vertical_jerk + angular_jerk * 0.35;
        let mut leg_use: f32 = 0.0;
        for i in 0..2 {
            leg_use =
                leg_use.max(self.hip_world(i).distance(self.legs[i].foot) / self.spec.max_reach);
        }
        self.prev_y_vel = self.vel.y;
        self.prev_pitch_vel = self.pitch_vel;
        self.prev_roll_vel = self.roll_vel;
        self.validation.update_window(
            dt,
            self.pitch,
            self.roll,
            capture_error,
            balance_error,
            speed,
            self.vel.y,
            cabin_jerk,
            leg_use,
            self.swing_count(),
        );
    }

    fn hip_world(&self, idx: usize) -> Vec3 {
        self.pos + self.body_rotation() * self.legs[idx].hip_local
    }

    fn body_rotation(&self) -> Quat {
        Quat::from_rotation_y(self.yaw)
            * Quat::from_rotation_z(self.roll)
            * Quat::from_rotation_x(self.pitch)
    }

    // ------------------------------------------------------------------
    // Debug/preview rendering
    // ------------------------------------------------------------------

    /// Emit a generic renderer-neutral body as oriented boxes/segments.
    /// Instanceable as-is (brushed-model / RTS crowds), or ignore this and
    /// skin a real rig from `leg_chain` + `signals`.
    pub fn part_poses(&self, out: &mut Vec<PartPose>) {
        let start = out.len();
        let hull_tint = Vec4::new(0.34, 0.43, 0.38, 1.0);
        let dark = Vec4::new(0.18, 0.21, 0.22, 1.0);
        let accent = Vec4::new(0.62, 0.59, 0.47, 1.0);
        let foot_tint = Vec4::new(0.15, 0.16, 0.15, 1.0);
        let rot = self.body_rotation();
        let spec = &self.spec;

        push_part(
            out,
            PartRole::Hull,
            MeshKey::BOX,
            spec.hull_size,
            rot,
            self.pos + rot * spec.hull_offset,
            hull_tint,
        );
        push_part(
            out,
            PartRole::Pelvis,
            MeshKey::BOX,
            spec.pelvis_size,
            rot,
            self.pos + rot * spec.pelvis_offset,
            dark,
        );
        let left_hip = self.hip_world(0);
        let right_hip = self.hip_world(1);
        push_segment(out, PartRole::Pelvis, left_hip, right_hip, 0.22, dark);

        let yaw_rot = Quat::from_rotation_y(self.yaw);
        let forward = yaw_rot * Vec3::Z;
        let right = yaw_rot * Vec3::X;
        for i in 0..2 {
            let hip = self.hip_world(i);
            let foot = self.legs[i].foot + Vec3::Y * 0.10;
            let compound_len = if self.legs[i].compound_ready {
                self.legs[i].compound_len
            } else {
                desired_compound_len(hip, foot, spec)
            };
            let (knee, hock, mid, end) = solve_chain_with_compound(
                hip,
                foot,
                forward,
                right,
                self.legs[i].side,
                compound_len,
                spec,
            );
            push_part(
                out,
                PartRole::Hip,
                MeshKey::JOINT,
                Vec3::splat(0.30),
                rot,
                hip,
                accent,
            );
            push_oriented_box(
                out,
                PartRole::UpperLink,
                hip,
                knee,
                Vec3::new(0.42, 1.0, 0.50),
                right,
                dark,
            );
            push_oriented_box(
                out,
                PartRole::MiddleLink,
                knee,
                hock,
                Vec3::new(0.28, 1.0, 0.38),
                right,
                accent,
            );
            if spec.mid_link > 0.01 {
                push_oriented_box(
                    out,
                    PartRole::MiddleLink,
                    hock,
                    mid,
                    Vec3::new(0.24, 1.0, 0.34),
                    right,
                    hull_tint,
                );
                push_oriented_box(
                    out,
                    PartRole::LowerLink,
                    mid,
                    end,
                    Vec3::new(0.34, 1.0, 0.42),
                    right,
                    dark,
                );
            } else {
                push_oriented_box(
                    out,
                    PartRole::LowerLink,
                    hock,
                    end,
                    Vec3::new(0.36, 1.0, 0.44),
                    right,
                    dark,
                );
            }
            push_part(
                out,
                PartRole::MiddleLink,
                MeshKey::JOINT,
                Vec3::splat(0.20),
                rot,
                knee,
                accent,
            );
            push_part(
                out,
                PartRole::MiddleLink,
                MeshKey::JOINT,
                Vec3::splat(0.18),
                rot,
                hock,
                accent,
            );
            push_part(
                out,
                PartRole::Foot,
                MeshKey::JOINT,
                Vec3::splat(0.16),
                Quat::IDENTITY,
                end,
                foot_tint,
            );
            push_part(
                out,
                PartRole::Foot,
                MeshKey::BOX,
                spec.foot_size,
                Quat::from_rotation_y(self.yaw),
                self.legs[i].foot + Vec3::Y * 0.06,
                foot_tint,
            );
        }

        if self.scale != 1.0 {
            for pose in &mut out[start..] {
                pose.transform.translation =
                    (Vec3::from_array(pose.transform.translation) * self.scale).to_array();
                pose.transform.scale =
                    (Vec3::from_array(pose.transform.scale) * self.scale).to_array();
            }
        }
    }
}

// ----------------------------------------------------------------------
// IK / chain solvers
// ----------------------------------------------------------------------

fn solve_chain_with_compound(
    hip: Vec3,
    foot: Vec3,
    forward: Vec3,
    right: Vec3,
    side: f32,
    compound_len: f32,
    spec: &WalkerSpec,
) -> (Vec3, Vec3, Vec3, Vec3) {
    let compound_len = compound_len
        .max(needed_compound_len(hip, foot, spec))
        .clamp(compound_min_len(spec), compound_max_len(spec));
    let (front_pole, rear_pole, lower_pole) = match spec.bend {
        KneeBend::Backward => (
            -forward * 1.55 + Vec3::Y * 0.42 + right * side * 0.08,
            -forward * 1.65 + Vec3::Y * 0.62 + right * side * 0.06,
            -forward * 1.10 + Vec3::Y * 0.44 - right * side * 0.08,
        ),
        KneeBend::Forward => (
            forward * 1.55 + Vec3::Y * 0.28 + right * side * 0.08,
            forward * 1.42 + Vec3::Y * 0.54 + right * side * 0.06,
            forward * 1.05 + Vec3::Y * 0.38 - right * side * 0.08,
        ),
        KneeBend::Normal => (
            forward * 1.25 - Vec3::Y * 0.18 + right * side * 0.10,
            -forward * 1.45 + Vec3::Y * 0.82 + right * side * 0.08,
            forward * 0.35 + Vec3::Y * 0.58 - right * side * 0.12,
        ),
    };
    let (front_low, _) = solve_two_bone(hip, foot, spec.hip_link, compound_len, front_pole);
    if spec.mid_link > 0.01 {
        let distal_len = spec.mid_link + spec.shin_link - 0.02;
        let (rear_hock, _) =
            solve_two_bone(front_low, foot, spec.reverse_link, distal_len, rear_pole);
        let (lower_hock, end) =
            solve_two_bone(rear_hock, foot, spec.mid_link, spec.shin_link, lower_pole);
        (front_low, rear_hock, lower_hock, end)
    } else {
        let (rear_hock, end) = solve_two_bone(
            front_low,
            foot,
            spec.reverse_link,
            spec.shin_link,
            rear_pole,
        );
        (front_low, rear_hock, end, end)
    }
}

fn desired_compound_len(hip: Vec3, foot: Vec3, spec: &WalkerSpec) -> f32 {
    let preferred = (spec.reverse_link + spec.mid_link + spec.shin_link) * 0.84;
    preferred
        .max(needed_compound_len(hip, foot, spec))
        .clamp(compound_min_len(spec), compound_max_len(spec))
}

fn needed_compound_len(hip: Vec3, foot: Vec3, spec: &WalkerSpec) -> f32 {
    (hip.distance(foot) - spec.hip_link + 0.03)
        .max(compound_min_len(spec))
        .min(compound_max_len(spec))
}

fn compound_min_len(spec: &WalkerSpec) -> f32 {
    let total = spec.reverse_link + spec.mid_link + spec.shin_link;
    let longest = spec.reverse_link.max(spec.mid_link).max(spec.shin_link);
    (longest - (total - longest)).max(0.0) + 0.03
}

fn compound_max_len(spec: &WalkerSpec) -> f32 {
    spec.reverse_link + spec.mid_link + spec.shin_link - 0.03
}

fn solve_two_bone(root: Vec3, target: Vec3, l1: f32, l2: f32, pole: Vec3) -> (Vec3, Vec3) {
    let to_target = target - root;
    let mut dist = to_target.length();
    let eps = 1.0e-4;
    let clamped = dist.clamp((l1 - l2).abs() + eps, l1 + l2 - eps);
    let dir = if dist > eps {
        to_target / dist
    } else {
        Vec3::Y
    };
    dist = clamped;
    let end = root + dir * dist;
    let a = ((l1 * l1 - l2 * l2) / (2.0 * dist) + dist * 0.5).clamp(-l1, l1);
    let h = (l1 * l1 - a * a).max(0.0).sqrt();
    let mut bend = pole - dir * pole.dot(dir);
    if bend.length_squared() < 1.0e-6 {
        let arbitrary = if dir.x.abs() < 0.9 { Vec3::X } else { Vec3::Z };
        bend = arbitrary - dir * arbitrary.dot(dir);
    }
    bend = bend.normalize_or(Vec3::X);
    (root + dir * a + bend * h, end)
}

fn chain_posture_cost(points: &[Vec3], extension: f32, preferred_extension: f32) -> f32 {
    let mut straight_cost: f32 = 0.0;
    if points.len() >= 3 {
        for i in 1..points.len() - 1 {
            let a = (points[i] - points[i - 1]).normalize_or(Vec3::Y);
            let b = (points[i + 1] - points[i]).normalize_or(Vec3::Y);
            straight_cost = straight_cost.max((a.dot(b).abs() - 0.92).max(0.0).powi(2) * 18.0);
        }
    }
    let extension_cost = (extension - preferred_extension).abs().powi(2) * 2.6;
    straight_cost + extension_cost
}

// ----------------------------------------------------------------------
// Small helpers
// ----------------------------------------------------------------------

fn swing_travel_t(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    1.0 - (1.0 - t) * (1.0 - t)
}

fn terrain_grade_accel(n: Vec3) -> Vec3 {
    let inv_y = 1.0 / n.y.max(0.25);
    Vec3::new(-n.x * inv_y, 0.0, -n.z * inv_y).clamp_length_max(0.48) * 9.81
}

fn drive_scalar_motor(
    value: &mut f32,
    velocity: &mut f32,
    target: f32,
    dt: f32,
    max_speed: f32,
    max_accel: f32,
    stiffness: f32,
) {
    let desired_velocity = ((target - *value) * stiffness).clamp(-max_speed, max_speed);
    let dv = (desired_velocity - *velocity).clamp(-max_accel * dt, max_accel * dt);
    *velocity += dv;
    *value += *velocity * dt;
}

fn drive_axis_motor(value: &mut f32, velocity: &mut f32, target: f32, dt: f32, motor: AxisMotor) {
    drive_scalar_motor(
        value,
        velocity,
        target,
        dt,
        motor.max_speed,
        motor.max_accel,
        motor.stiffness,
    );
}

pub(crate) fn angle_delta(to: f32, from: f32) -> f32 {
    (to - from).sin().atan2((to - from).cos())
}

fn push_part(
    out: &mut Vec<PartPose>,
    role: PartRole,
    mesh: MeshKey,
    scale: Vec3,
    rot: Quat,
    translation: Vec3,
    tint: Vec4,
) {
    out.push(PartPose {
        role,
        transform: Transform {
            translation: translation.to_array(),
            rotation_xyzw: [rot.x, rot.y, rot.z, rot.w],
            scale: scale.to_array(),
        },
        mesh,
        material: part_material(role),
        tint: tint.to_array(),
    });
}

fn part_material(role: PartRole) -> u16 {
    match role {
        PartRole::Hull => 1,
        PartRole::Pelvis => 3,
        PartRole::Hip => 4,
        PartRole::UpperLink => 5,
        PartRole::MiddleLink => 6,
        PartRole::LowerLink => 7,
        PartRole::Foot => 8,
    }
}

fn push_segment(
    out: &mut Vec<PartPose>,
    role: PartRole,
    a: Vec3,
    b: Vec3,
    radius: f32,
    tint: Vec4,
) {
    let d = b - a;
    let len = d.length().max(1.0e-4);
    let dir = d / len;
    let rot = Quat::from_rotation_arc(Vec3::Y, dir);
    push_part(
        out,
        role,
        MeshKey::LIMB,
        Vec3::new(radius, len, radius),
        rot,
        a,
        tint,
    );
}

fn push_oriented_box(
    out: &mut Vec<PartPose>,
    role: PartRole,
    a: Vec3,
    b: Vec3,
    scale: Vec3,
    pole: Vec3,
    tint: Vec4,
) {
    let d = b - a;
    let len = d.length().max(1.0e-4);
    let y = d / len;
    let mut x = pole - y * pole.dot(y);
    if x.length_squared() < 1.0e-6 {
        let fallback = if y.y.abs() < 0.85 { Vec3::Y } else { Vec3::Z };
        x = fallback - y * fallback.dot(y);
    }
    let x = x.normalize_or(Vec3::X);
    let z = x.cross(y).normalize_or(Vec3::Z);
    let x = y.cross(z).normalize_or(x);
    let rot = Quat::from_mat3(&Mat3::from_cols(x, y, z));
    push_part(
        out,
        role,
        MeshKey::BOX,
        Vec3::new(scale.x, len * scale.y, scale.z),
        rot,
        a + d * 0.5,
        tint,
    );
}

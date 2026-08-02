use glam::Vec3;

/// Which archetype box/segment a `PartPose` represents. Coarse on purpose:
/// hosts map roles to their own meshes/materials, or instance the raw
/// boxes directly (ideal for brushed/RTS-crowd rendering).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PartRole {
    Hull,
    Pelvis,
    Hip,
    UpperLink,
    MiddleLink,
    LowerLink,
    Foot,
}

/// Mesh archetype hint: a unit box, a unit-height limb cylinder, or a
/// joint blob. Hosts may ignore this and key off `PartRole` alone.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MeshKey(pub u16);

impl MeshKey {
    pub const BOX: Self = Self(0);
    pub const LIMB: Self = Self(1);
    pub const JOINT: Self = Self(2);
}

#[derive(Clone, Copy, Debug)]
pub struct Transform {
    pub translation: [f32; 3],
    pub rotation_xyzw: [f32; 4],
    pub scale: [f32; 3],
}

/// One renderable part, world space, renderer-neutral. `Walker::part_poses`
/// emits a full debug/preview body from these; production hosts will more
/// likely skin a rig from `LegChain` + body transform instead.
#[derive(Clone, Copy, Debug)]
pub struct PartPose {
    pub role: PartRole,
    pub transform: Transform,
    pub mesh: MeshKey,
    pub material: u16,
    pub tint: [f32; 4],
}

/// World-space joint positions for one leg, hip to toe. `mid` is only
/// present for four-link chains (`WalkerSpec::mid_link > 0`).
#[derive(Clone, Copy, Debug)]
pub struct LegChain {
    pub hip: Vec3,
    pub knee: Vec3,
    pub hock: Vec3,
    pub mid: Option<Vec3>,
    pub foot: Vec3,
}

/// Per-leg signal for pose layers.
#[derive(Clone, Copy, Debug)]
pub struct LegSignal {
    /// True while the foot is planted (stance).
    pub contact: bool,
    /// 0..1 through the current swing; 0 when in stance.
    pub swing_t: f32,
    /// Current world foot position.
    pub foot: Vec3,
    /// Where the current/last swing is headed, world space.
    pub target: Vec3,
    /// -1 left, +1 right.
    pub side: f32,
}

/// Everything an Overgrowth-style pose layer needs from the locomotion
/// core, world space, one call. Drive arm counter-swing off leg phases,
/// lean/anticipation off `balance` and acceleration, head-look and cloth
/// off `pitch`/`roll`, upper-body twist off `support_yaw` vs `yaw`,
/// landing crouches off footfall events.
#[derive(Clone, Copy, Debug)]
pub struct RigSignals {
    pub pos: Vec3,
    pub vel: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
    /// Yaw implied by the support line (feet), if well-defined. The
    /// difference from `yaw` is how twisted the body is over its feet —
    /// map it to pelvis/spine counter-rotation.
    pub support_yaw: Option<f32>,
    /// Capture point minus support center (world, planar). Grows when
    /// momentum is carrying the walker off its feet.
    pub capture_error: Vec3,
    /// Blended COM/capture balance vector (world, planar); the planner's
    /// own "which way am I falling" signal.
    pub balance: Vec3,
    /// 0..1: how close the controller thinks it is to losing control.
    pub control_risk: f32,
    /// 0..1 sine hump over the active swing; useful for body bob layers.
    pub swing_wave: f32,
    pub legs: [LegSignal; 2],
}

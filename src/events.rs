use glam::Vec3;

use crate::handles::WalkerHandle;

/// A foot touching down. `pos` is world space; `strength` grows with step
/// length and body speed — feed it to audio volume, dust bursts, camera
/// shake, RTS-crowd rumble.
#[derive(Clone, Copy, Debug)]
pub struct FootfallEvent {
    pub pos: Vec3,
    pub strength: f32,
}

/// Host-facing event stream drained from `WalkerWorld::drain_events`.
#[derive(Clone, Copy, Debug)]
pub enum WalkerEvent {
    Footfall {
        walker: WalkerHandle,
        pos: [f32; 3],
        strength: f32,
    },
    /// Aggregate actuator state, normalized 0..1 (load) and -1..1
    /// (motion). Drives servo/hydraulic audio loops.
    ActuatorLoad {
        walker: WalkerHandle,
        load: f32,
        motion: f32,
    },
    /// Normalized 0..1 powerplant demand. Drives engine audio.
    PowerDemand { walker: WalkerHandle, value: f32 },
}

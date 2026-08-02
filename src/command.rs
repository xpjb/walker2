use glam::Vec3;

/// Per-tick locomotion intent, in world space. This is the M&B-style
/// control model: a movement vector and a facing, fully decoupled.
///
/// - `move_dir`: world-space desired travel direction; magnitude below
///   0.05 reads as "no input". Direction is honored continuously (strafe
///   and backpedal are just vectors), speed comes from the spec.
/// - `face_yaw`: desired facing in radians. The body tracks it with a
///   spring ("magic" rotation); feet never plan rotation — they re-plant
///   at rotated home positions when twist exceeds the comfort gate.
/// - `sprint`: gait selector.
///
/// There is deliberately no `turn` field: integrate turn input into
/// `face_yaw` host-side.
#[derive(Clone, Copy, Debug)]
pub struct WalkerCommand {
    pub move_dir: Vec3,
    pub face_yaw: f32,
    pub sprint: bool,
}

impl WalkerCommand {
    pub const IDLE: Self = Self {
        move_dir: Vec3::ZERO,
        face_yaw: 0.0,
        sprint: false,
    };

    pub fn face(face_yaw: f32) -> Self {
        Self {
            face_yaw,
            ..Self::IDLE
        }
    }

    pub fn walk(move_dir: Vec3, face_yaw: f32) -> Self {
        Self {
            move_dir,
            face_yaw,
            sprint: false,
        }
    }

    pub fn sprint(move_dir: Vec3, face_yaw: f32) -> Self {
        Self {
            move_dir,
            face_yaw,
            sprint: true,
        }
    }
}

use glam::Vec3;
use walker2::{Fidelity, WalkerCommand};

pub const DT: f32 = 1.0 / 60.0;

pub const FLAT_PHASE_SECONDS: f32 = 8.0;
pub const FLAT_DURATION: f32 = FLAT_PHASE_SECONDS * 2.0;

pub const TURN_IDLE_SECONDS: f32 = 1.0;
pub const TURN_PHASE_SECONDS: f32 = 6.0;
pub const TURN_TARGET_A: f32 = 150.0_f32.to_radians();
pub const TURN_TARGET_B: f32 = 30.0_f32.to_radians();
pub const TURN_DURATION: f32 = TURN_IDLE_SECONDS + TURN_PHASE_SECONDS * 2.0;

pub const MOVE_PHASE_SECONDS: f32 = 6.0;
pub const MOVE_DURATION: f32 = MOVE_PHASE_SECONDS * 3.0;

pub const START_SECONDS: f32 = 1.0;
pub const BRAKE_AT: f32 = 6.0;
pub const START_STOP_DURATION: f32 = 10.0;

pub const ROLLING_SECONDS: f32 = 12.0;
pub const SLOPE_SECONDS: f32 = 10.0;
pub const ROLLING_AMPLITUDE: f32 = 0.6;
pub const ROLLING_WAVELENGTH: f32 = 9.0;
pub const SLOPE_GRADE: f32 = 0.14;

pub const IMPULSE_1_AT: f32 = 2.0;
pub const IMPULSE_2_AT: f32 = 5.0;
pub const IMPULSE_SPRINT_AT: f32 = 8.0;
pub const IMPULSE_3_AT: f32 = 10.5;
pub const IMPULSE_DURATION: f32 = 14.0;
pub const IMPULSE_1: Vec3 = Vec3::new(3.5, 0.0, 0.0);
pub const IMPULSE_2: Vec3 = Vec3::new(-2.6, 0.0, -2.6);
pub const IMPULSE_3: Vec3 = Vec3::new(3.0, 0.0, 0.0);

pub const SCALE_WARMUP: f32 = 3.0;
pub const SCALE_MEASURE: f32 = 8.0;
pub const SCALE_DURATION: f32 = SCALE_WARMUP + SCALE_MEASURE;
pub const SCALES: [f32; 3] = [0.28, 1.0, 4.0];

pub const FIDELITY_DURATION: f32 = 13.0;
pub const FIDELITIES: [(&str, Fidelity); 3] = [
    ("Full", Fidelity::Full),
    ("Reduced", Fidelity::Reduced),
    ("Kinematic", Fidelity::Kinematic),
];

pub const CROWD_SECONDS: f32 = 10.0;
pub const CROWD_TIERS: [(&str, Fidelity, usize); 3] = [
    ("Full", Fidelity::Full, 30),
    ("Reduced", Fidelity::Reduced, 60),
    ("Kinematic", Fidelity::Kinematic, 90),
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScenarioId {
    FlatWalk,
    TurnInPlace,
    StrafeBackpedal,
    StartStop,
    Terrain,
    ImpulseRecovery,
    ScaleFroude,
    FidelityTiers,
    Crowd,
}

impl ScenarioId {
    pub const ALL: [Self; 9] = [
        Self::FlatWalk,
        Self::TurnInPlace,
        Self::StrafeBackpedal,
        Self::StartStop,
        Self::Terrain,
        Self::ImpulseRecovery,
        Self::ScaleFroude,
        Self::FidelityTiers,
        Self::Crowd,
    ];

    pub fn number(self) -> usize {
        Self::ALL.iter().position(|id| *id == self).unwrap_or(0) + 1
    }

    pub fn from_number(number: usize) -> Option<Self> {
        Self::ALL.get(number.checked_sub(1)?).copied()
    }

    pub fn title(self) -> &'static str {
        match self {
            Self::FlatWalk => "Flat walk and sprint",
            Self::TurnInPlace => "Turn in place and untwist",
            Self::StrafeBackpedal => "Strafe, backpedal, diagonal",
            Self::StartStop => "Sprint launch and hard brake",
            Self::Terrain => "Rolling terrain then 14% grade",
            Self::ImpulseRecovery => "Impulse recovery",
            Self::ScaleFroude => "Froude scale comparison",
            Self::FidelityTiers => "Fidelity comparison and switching",
            Self::Crowd => "RTS crowd",
        }
    }

    pub fn criterion(self) -> &'static str {
        match self {
            Self::FlatWalk => "Clean alternating steps; sprint transition stays planted",
            Self::TurnInPlace => "Body turns first; feet untwist one at a time without crossing",
            Self::StrafeBackpedal => "Torso keeps facing +Z while travel direction changes",
            Self::StartStop => "Launch plants trail COM; braking plants move ahead; no oscillation",
            Self::Terrain => "Feet meet the profile while body height and tilt change smoothly",
            Self::ImpulseRecovery => "Each shove causes recovery footwork, then a stable settle",
            Self::ScaleFroude => "Small walker scurries; giant has visibly slower cadence",
            Self::FidelityTiers => "Tiers follow the same path; switching actor does not pop",
            Self::Crowd => "All walkers remain upright and footfalls continue across LODs",
        }
    }

    pub fn duration(self) -> f32 {
        match self {
            Self::FlatWalk => FLAT_DURATION,
            Self::TurnInPlace => TURN_DURATION,
            Self::StrafeBackpedal => MOVE_DURATION,
            Self::StartStop => START_STOP_DURATION,
            Self::Terrain => ROLLING_SECONDS + SLOPE_SECONDS,
            Self::ImpulseRecovery => IMPULSE_DURATION,
            Self::ScaleFroude => SCALE_DURATION,
            Self::FidelityTiers => FIDELITY_DURATION,
            Self::Crowd => CROWD_SECONDS,
        }
    }
}

pub fn flat_walk_command(t: f32) -> WalkerCommand {
    if t < FLAT_PHASE_SECONDS {
        WalkerCommand::walk(Vec3::Z, 0.0)
    } else {
        WalkerCommand::sprint(Vec3::Z, 0.0)
    }
}

pub fn turn_command(t: f32) -> WalkerCommand {
    if t < TURN_IDLE_SECONDS {
        WalkerCommand::IDLE
    } else if t < TURN_IDLE_SECONDS + TURN_PHASE_SECONDS {
        WalkerCommand::face(TURN_TARGET_A)
    } else {
        WalkerCommand::face(TURN_TARGET_B)
    }
}

pub fn movement_command(t: f32) -> WalkerCommand {
    if t < MOVE_PHASE_SECONDS {
        WalkerCommand::walk(Vec3::X, 0.0)
    } else if t < MOVE_PHASE_SECONDS * 2.0 {
        WalkerCommand::walk(-Vec3::Z, 0.0)
    } else {
        WalkerCommand::walk(Vec3::new(-1.0, 0.0, 1.0).normalize(), 0.0)
    }
}

pub fn start_stop_command(t: f32) -> WalkerCommand {
    if t < START_SECONDS || t >= BRAKE_AT {
        WalkerCommand::IDLE
    } else {
        WalkerCommand::sprint(Vec3::Z, 0.0)
    }
}

pub fn impulse_command(t: f32) -> WalkerCommand {
    if t < IMPULSE_SPRINT_AT {
        WalkerCommand::IDLE
    } else {
        WalkerCommand::sprint(Vec3::Z, 0.0)
    }
}

pub fn fidelity_command(t: f32) -> WalkerCommand {
    if t < 4.0 {
        WalkerCommand::walk(Vec3::Z, 0.0)
    } else if t < 8.0 {
        WalkerCommand::sprint(Vec3::Z, 0.0)
    } else if t < 10.0 {
        WalkerCommand::face(2.0)
    } else {
        WalkerCommand::walk(Vec3::X, 2.0)
    }
}

pub fn crowd_command(index: usize, t: f32) -> WalkerCommand {
    let phase = index as f32 * 0.61 + t * 0.25;
    WalkerCommand {
        move_dir: Vec3::new(phase.cos(), 0.0, phase.sin()),
        face_yaw: phase,
        sprint: index % 4 == 0,
    }
}

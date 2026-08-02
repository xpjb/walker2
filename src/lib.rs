//! # walker2
//!
//! Procedural biped locomotion, extracted and library-ified from the
//! petrogradrevival mech walker. Physics-first: no keyframes anywhere —
//! capture-point / Raibert step planning, an XPBD-style stance constraint
//! stack, spring-leg contact dynamics, and analytic IK chains produce the
//! walk. Feed it a `GroundQuery` and a `WalkerCommand` per tick; read back
//! world-space poses, rig signals, and footfall events.
//!
//! Design decisions (vs. the original mech code):
//!
//! - **Facing is "magic".** Body yaw is a spring toward `face_yaw`; feet
//!   never plan rotation. When idle twist exceeds a comfort gate the feet
//!   re-plant at rotated home positions (`untwist`). Movement direction is
//!   a world-space vector fully decoupled from facing — strafe and
//!   backpedal are just vectors (the M&B control model).
//! - **The LIP family is kept** (capture point, Raibert targeting,
//!   cart-pole feedforward/scoring, acceleration lean) — it is what makes
//!   starts, stops, and braking read as weight.
//! - **Per-step action sampling and rollout previews are gone** (mech
//!   hydraulic flavor, an order of magnitude of planning cost).
//! - **Everything is spec-driven** (`WalkerSpec`), with three fidelity
//!   tiers (`Fidelity`) sharing one state layout for popping-free LOD.
//! - **Size is a spawn-time `scale`**, Froude-scaled: one tuning walks at
//!   every size with physically consistent cadence (world speed scales
//!   with sqrt(scale)). Specs stay in canonical units — do not author
//!   "small" specs, spawn small scales.
//!
//! Layering intent: this crate is the lower-body/COM ground truth. Build
//! an Overgrowth-style pose layer on top using `RigSignals` (arm swing
//! from leg phase, lean from acceleration, pelvis twist from
//! `support_yaw`, landing crouches from footfalls).

pub mod command;
pub mod events;
pub mod ground;
pub mod handles;
pub mod rig;
pub mod spec;
pub mod testkit;
pub mod validation;
pub mod walker;
pub mod world;

pub use command::WalkerCommand;
pub use events::{FootfallEvent, WalkerEvent};
pub use ground::{GroundQuery, ScaledGround};
pub use handles::WalkerHandle;
pub use rig::{LegChain, LegSignal, MeshKey, PartPose, PartRole, RigSignals, Transform};
pub use spec::{AxisMotor, KneeBend, Pair, SlipParams, TractionParams, WalkerSpec};
pub use validation::GaitValidation;
pub use walker::{Fidelity, Walker};
pub use world::{StepCtx, WalkerSpawnDesc, WalkerWorld};

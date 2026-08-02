//! 03 — Facing-decoupled movement (the M&B control model).
//!
//! Facing stays locked at +Z while the movement vector points sideways,
//! backward, and diagonally. This is the control model the library is
//! built around: `move_dir` and `face_yaw` are independent, and strafe /
//! backpedal fall out of the omnidirectional planner rather than being
//! authored gaits. Sidesteps are deliberately shorter than forward
//! strides (align-scaled), so expect strafe to be slower than walk speed.
//!
//! Run: `cargo run --example 03_strafe_backpedal`

use glam::Vec3;
use walker2::testkit::{write_trace_svg, Checks, FlatGround, Runner};
use walker2::{Walker, WalkerCommand, WalkerSpec};

fn main() {
    std::process::exit(run());
}

fn run() -> i32 {
    println!("== 03 strafe/backpedal: face +Z always; move +X, then -Z, then diagonal ==");
    let mut checks = Checks::new();
    let ground = FlatGround;
    let spec = WalkerSpec::biped();
    let mut walker = Walker::new(spec, &ground);
    let mut runner = Runner::new(1.0 / 60.0);

    let mut max_yaw: f32 = 0.0;

    // Phase 1: strafe right.
    let p0 = walker.position();
    runner.run(&mut walker, &ground, 6.0, |_, w| {
        WalkerCommand::walk(Vec3::X, 0.0).tap_yaw(&mut max_yaw, w)
    });
    let p1 = walker.position();
    checks.check_ge("strafe +X distance (6s)", p1.x - p0.x, 8.0);
    checks.check_le("strafe lateral drift |z|", (p1.z - p0.z).abs(), 2.5);

    // Phase 2: backpedal.
    runner.run(&mut walker, &ground, 6.0, |_, w| {
        WalkerCommand::walk(-Vec3::Z, 0.0).tap_yaw(&mut max_yaw, w)
    });
    let p2 = walker.position();
    checks.check_ge("backpedal -Z distance (6s)", p1.z - p2.z, 10.0);

    // Phase 3: diagonal (forward-left), still facing +Z.
    let diag = Vec3::new(-1.0, 0.0, 1.0).normalize();
    runner.run(&mut walker, &ground, 6.0, |_, w| {
        WalkerCommand::walk(diag, 0.0).tap_yaw(&mut max_yaw, w)
    });
    let p3 = walker.position();
    let diag_dist = (p3 - p2).dot(diag);
    checks.check_ge("diagonal distance (6s)", diag_dist, 9.0);

    checks.check_le("facing held throughout (max |yaw| rad)", max_yaw, 0.10);
    let v = walker.validation();
    checks.check_le("same-foot double steps", v.total_same_foot as f32, 2.0);
    checks.check_le("double-swing frames", v.total_double_swing_frames as f32, 0.0);
    println!(
        "  strafe {:.1}m, backpedal {:.1}m, diagonal {:.1}m, steps={}",
        p1.x - p0.x,
        p1.z - p2.z,
        diag_dist,
        v.total_steps
    );
    runner.print_reports();
    let _ = write_trace_svg(
        "target/traces/03_strafe_backpedal.svg",
        &runner,
        "03 strafe -> backpedal -> diagonal (facing +Z the whole time)",
    );
    checks.finish("03_strafe_backpedal")
}

trait TapYaw {
    fn tap_yaw(self, max_yaw: &mut f32, walker: &Walker) -> Self;
}

impl TapYaw for WalkerCommand {
    fn tap_yaw(self, max_yaw: &mut f32, walker: &Walker) -> Self {
        *max_yaw = max_yaw.max(walker.yaw().abs());
        self
    }
}

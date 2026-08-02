//! 05 — Terrain: rolling hills and a constant grade.
//!
//! Feet follow terrain height during swings, stance anchors re-sample the
//! ground, ride height is a rate-limited motor (bumps become crouch/climb
//! instead of snaps), and the cart-pole feedforward leans plants uphill.
//! Checks: forward progress on both terrains, bounded tilt, and that
//! reach violations (leg overstretch events) stay rare.
//!
//! Run: `cargo run --example 05_terrain`

mod shared;

use glam::Vec3;
use shared::scenarios::{
    DT, ROLLING_AMPLITUDE, ROLLING_SECONDS, ROLLING_WAVELENGTH, SLOPE_GRADE, SLOPE_SECONDS,
};
use walker2::testkit::{write_trace_svg, Checks, RollingGround, Runner, SlopeGround};
use walker2::{GroundQuery, Walker, WalkerCommand, WalkerSpec};

fn main() {
    std::process::exit(run());
}

fn run() -> i32 {
    println!("== 05 terrain: rolling hills walk, then 14% grade sprint ==");
    let mut checks = Checks::new();
    let spec = WalkerSpec::biped();

    // Rolling hills.
    let rolling = RollingGround {
        amplitude: ROLLING_AMPLITUDE,
        wavelength: ROLLING_WAVELENGTH,
    };
    let mut walker = Walker::new(spec, &rolling);
    let mut runner = Runner::new(DT);
    runner.run(&mut walker, &rolling, ROLLING_SECONDS, |_, _| {
        WalkerCommand::walk(Vec3::Z, 0.0)
    });
    let dist = walker.position().z;
    checks.check_ge(
        "rolling: distance (12s walk)",
        dist,
        spec.speed.walk * 12.0 * 0.6,
    );
    checks.check_le("rolling: max tilt (rad)", runner.max_tilt(), 0.30);
    let v = walker.validation();
    checks.check_le(
        "rolling: same-foot double steps",
        v.total_same_foot as f32,
        2.0,
    );
    checks.check_le(
        "rolling: reach violations",
        v.total_reach_violations as f32,
        400.0,
    );
    println!(
        "  rolling: {dist:.1}m, steps={}, reach_violations={}",
        v.total_steps, v.total_reach_violations
    );
    runner.print_reports();
    let _ = write_trace_svg(
        "target/traces/05_terrain_rolling.svg",
        &runner,
        "05 rolling hills walk",
    );

    // Constant climb.
    let slope = SlopeGround { grade: SLOPE_GRADE };
    let mut climber = Walker::new(spec, &slope);
    let mut climb_runner = Runner::new(DT);
    climb_runner.run(&mut climber, &slope, SLOPE_SECONDS, |_, _| {
        WalkerCommand::sprint(Vec3::Z, 0.0)
    });
    let climb_dist = climber.position().z;
    let climb_height = climber.position().y - slope.height_at(0.0, 0.0);
    checks.check_ge(
        "slope: distance (10s sprint, 14% grade)",
        climb_dist,
        spec.speed.sprint * 10.0 * 0.5,
    );
    checks.check_le("slope: max tilt (rad)", climb_runner.max_tilt(), 0.30);
    let vc = climber.validation();
    checks.check_le(
        "slope: same-foot double steps",
        vc.total_same_foot as f32,
        2.0,
    );
    println!(
        "  slope: {climb_dist:.1}m forward, ~{:.1}m climbed, steps={}",
        climb_height, vc.total_steps
    );
    climb_runner.print_reports();

    checks.finish("05_terrain")
}

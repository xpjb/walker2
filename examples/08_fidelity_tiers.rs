//! 08 — Fidelity tiers: Full vs Reduced vs Kinematic.
//!
//! The same 13-second movement script (walk, sprint, turn-in-place,
//! strafe) runs at each tier. Reduced must track Full's outcome closely
//! (it shares the physics and step triggers, just no candidate search and
//! fewer substeps); Kinematic must be in the right ballpark (it has no
//! balance dynamics at all). Also validates popping-free tier switching
//! mid-run, and prints per-tier wall time so the cost ladder is visible.
//!
//! Run: `cargo run --example 08_fidelity_tiers` (use --release for
//! meaningful timings)

use std::time::Instant;

use glam::Vec3;
use walker2::testkit::{write_trace_svg, Checks, FlatGround, Runner};
use walker2::{Fidelity, Walker, WalkerCommand, WalkerSpec};

fn script(t: f32) -> WalkerCommand {
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

fn main() {
    std::process::exit(run());
}

fn run() -> i32 {
    println!("== 08 fidelity tiers: same 13s script at Full / Reduced / Kinematic ==");
    let mut checks = Checks::new();
    let ground = FlatGround;
    let spec = WalkerSpec::biped();
    let seconds = 13.0;

    let mut outcomes = Vec::new();
    for (name, fidelity) in [
        ("Full", Fidelity::Full),
        ("Reduced", Fidelity::Reduced),
        ("Kinematic", Fidelity::Kinematic),
    ] {
        let mut walker = Walker::new(spec, &ground);
        walker.set_fidelity(fidelity);
        let mut runner = Runner::new(1.0 / 60.0);
        let start = Instant::now();
        runner.run(&mut walker, &ground, seconds, |t, _| script(t));
        let elapsed = start.elapsed();
        let ticks = (seconds * 60.0) as u32;
        let pos = walker.position();
        let v = walker.validation();
        println!(
            "  {name:9}: end=({:6.1},{:6.1}) steps={:3} same_foot={} max_tilt={:.3} | {:6.1} us/tick",
            pos.x,
            pos.z,
            v.total_steps,
            v.total_same_foot,
            runner.max_tilt(),
            elapsed.as_micros() as f64 / ticks as f64,
        );
        let _ = write_trace_svg(
            &format!("target/traces/08_fidelity_{}.svg", name.to_lowercase()),
            &runner,
            &format!("08 fidelity tier: {name}"),
        );
        outcomes.push((name, pos, v.total_steps));
        checks.check(
            &format!("{name}: finite & upright"),
            pos.is_finite() && pos.y > spec.body_height() * 0.4,
            format!("pos {pos:?}"),
        );
    }

    let full_pos = outcomes[0].1;
    let reduced_pos = outcomes[1].1;
    let kinematic_pos = outcomes[2].1;
    let travel = full_pos.distance(Vec3::ZERO).max(1.0);
    checks.check_le(
        "Reduced endpoint within 20% of Full",
        reduced_pos.distance(full_pos) / travel,
        0.20,
    );
    checks.check_le(
        "Kinematic endpoint within 35% of Full",
        kinematic_pos.distance(full_pos) / travel,
        0.35,
    );

    // Popping-free switching: walk on Full, then hop tiers mid-stride.
    let mut walker = Walker::new(spec, &ground);
    let dt = 1.0 / 60.0;
    for i in 0..(4 * 60) {
        walker.step(&ground, WalkerCommand::walk(Vec3::Z, 0.0), dt);
        let _ = i;
    }
    for fidelity in [Fidelity::Reduced, Fidelity::Kinematic, Fidelity::Full] {
        let before = walker.position();
        walker.set_fidelity(fidelity);
        walker.step(&ground, WalkerCommand::walk(Vec3::Z, 0.0), dt);
        let jump = walker.position().distance(before);
        checks.check_le(&format!("switch to {fidelity:?}: one-tick jump (m)"), jump, 0.40);
        for _ in 0..60 {
            walker.step(&ground, WalkerCommand::walk(Vec3::Z, 0.0), dt);
        }
    }

    checks.finish("08_fidelity_tiers")
}

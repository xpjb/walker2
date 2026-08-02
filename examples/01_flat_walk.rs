//! 01 — Flat-ground walk & sprint, all presets.
//!
//! The baseline sanity example: every preset must reach its commanded
//! walk and sprint speeds on flat ground with clean alternation (no
//! same-foot double steps, never two feet in the air) and bounded body
//! tilt. Writes a top-down trace SVG for the biped run.
//!
//! Run: `cargo run --example 01_flat_walk`

mod shared;

use glam::Vec3;
use shared::scenarios::{DT, FLAT_PHASE_SECONDS};
use walker2::testkit::{write_trace_svg, Checks, FlatGround, Runner};
use walker2::{Walker, WalkerCommand, WalkerSpec};

fn main() {
    std::process::exit(run());
}

fn run() -> i32 {
    println!("== 01 flat walk: walk 8s then sprint 8s, per preset ==");
    let mut checks = Checks::new();
    let ground = FlatGround;

    for (name, spec) in [
        ("biped", WalkerSpec::biped()),
        ("longstrider", WalkerSpec::longstrider()),
        ("humanoid", WalkerSpec::humanoid()),
    ] {
        println!("\n-- preset: {name} --");
        let mut walker = Walker::new(spec, &ground);
        let mut runner = Runner::new(DT);
        runner.run(&mut walker, &ground, FLAT_PHASE_SECONDS, |_, _| {
            WalkerCommand::walk(Vec3::Z, 0.0)
        });
        let walk_speed = runner.avg_speed_last(4.0);
        runner.run(&mut walker, &ground, FLAT_PHASE_SECONDS, |_, _| {
            WalkerCommand::sprint(Vec3::Z, 0.0)
        });
        let sprint_speed = runner.avg_speed_last(4.0);

        checks.check_near(
            &format!("{name}: walk speed"),
            walk_speed,
            spec.speed.walk,
            spec.speed.walk * 0.25,
        );
        checks.check_near(
            &format!("{name}: sprint speed"),
            sprint_speed,
            spec.speed.sprint,
            spec.speed.sprint * 0.25,
        );
        let v = walker.validation();
        checks.check_le(
            &format!("{name}: same-foot double steps"),
            v.total_same_foot as f32,
            1.0,
        );
        checks.check_le(
            &format!("{name}: double-swing frames"),
            v.total_double_swing_frames as f32,
            0.0,
        );
        checks.check_le(&format!("{name}: max tilt (rad)"), runner.max_tilt(), 0.25);
        println!(
            "  steps={} plants={} distance={:.1}",
            v.total_steps,
            runner.plants.len(),
            walker.position().z
        );
        runner.print_reports();

        if name == "biped" {
            let _ = write_trace_svg(
                "target/traces/01_flat_walk.svg",
                &runner,
                "01 flat walk+sprint (biped)",
            );
        }
    }

    checks.finish("01_flat_walk")
}

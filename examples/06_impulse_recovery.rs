//! 06 — Impulse recovery: the melee payoff.
//!
//! `apply_impulse` is the hook a combat layer calls on shoves, parries,
//! and explosions. The balance loop (capture point + recovery stepping +
//! stabilized command blending) must turn each hit into visible recovery
//! footwork and settle — no canned stagger animations involved. Three
//! hits: lateral at idle, diagonal at idle, lateral mid-sprint.
//!
//! Run: `cargo run --example 06_impulse_recovery`

mod shared;

use glam::Vec3;
use shared::scenarios::{
    DT, IMPULSE_1, IMPULSE_1_AT, IMPULSE_2, IMPULSE_2_AT, IMPULSE_3, IMPULSE_3_AT,
    IMPULSE_DURATION, IMPULSE_SPRINT_AT,
};
use walker2::testkit::{write_trace_svg, Checks, FlatGround, Runner};
use walker2::{Walker, WalkerCommand, WalkerSpec};

fn main() {
    std::process::exit(run());
}

fn run() -> i32 {
    println!("== 06 impulse recovery: three hits, must recover from each ==");
    let mut checks = Checks::new();
    let ground = FlatGround;
    let spec = WalkerSpec::biped();
    let mut walker = Walker::new(spec, &ground);
    let mut runner = Runner::new(DT);

    // Hit 1: lateral shove at idle.
    runner.run(&mut walker, &ground, IMPULSE_1_AT, |_, _| {
        WalkerCommand::IDLE
    });
    let steps_0 = walker.validation().total_steps;
    walker.apply_impulse(IMPULSE_1);
    runner.run(&mut walker, &ground, IMPULSE_2_AT - IMPULSE_1_AT, |_, _| {
        WalkerCommand::IDLE
    });
    let steps_1 = walker.validation().total_steps;
    let speed_1 = runner.avg_speed_last(0.5);
    checks.check_ge("hit1: recovery steps", (steps_1 - steps_0) as f32, 1.0);
    checks.check_le("hit1: settled speed", speed_1, 0.6);

    // Hit 2: diagonal backward shove at idle.
    walker.apply_impulse(IMPULSE_2);
    runner.run(
        &mut walker,
        &ground,
        IMPULSE_SPRINT_AT - IMPULSE_2_AT,
        |_, _| WalkerCommand::IDLE,
    );
    let steps_2 = walker.validation().total_steps;
    let speed_2 = runner.avg_speed_last(0.5);
    checks.check_ge("hit2: recovery steps", (steps_2 - steps_1) as f32, 1.0);
    checks.check_le("hit2: settled speed", speed_2, 0.6);

    // Hit 3: lateral hit while sprinting — must keep running, not fall.
    runner.run(
        &mut walker,
        &ground,
        IMPULSE_3_AT - IMPULSE_SPRINT_AT,
        |_, _| WalkerCommand::sprint(Vec3::Z, 0.0),
    );
    walker.apply_impulse(IMPULSE_3);
    runner.run(
        &mut walker,
        &ground,
        IMPULSE_DURATION - IMPULSE_3_AT,
        |_, _| WalkerCommand::sprint(Vec3::Z, 0.0),
    );
    let sprint_speed = runner.avg_speed_last(1.0);
    checks.check_ge(
        "hit3: still sprinting after mid-run hit",
        sprint_speed,
        spec.speed.sprint * 0.6,
    );

    checks.check_le("max tilt across all hits (rad)", runner.max_tilt(), 0.30);
    let pos = walker.position();
    checks.check(
        "still standing / finite",
        pos.is_finite() && pos.y > spec.body_height() * 0.5,
        format!("pos {pos:?}"),
    );
    println!("  total steps={}", walker.validation().total_steps);
    runner.print_reports();
    let _ = write_trace_svg(
        "target/traces/06_impulse_recovery.svg",
        &runner,
        "06 impulse recovery (2 idle hits, 1 mid-sprint hit)",
    );
    checks.finish("06_impulse_recovery")
}

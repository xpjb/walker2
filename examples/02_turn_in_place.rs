//! 02 — Magic yaw + untwist re-planting.
//!
//! Facing is applied by a spring on body yaw ("magic"); the feet never
//! plan rotation. This example validates the replacement mechanism: when
//! idle twist exceeds the comfort gate, feet re-plant at rotated home
//! positions. Two turns (150° then -120°) while standing still must:
//! track yaw, trigger untwist steps, leave the support line near the new
//! facing, and never cross the legs.
//!
//! Run: `cargo run --example 02_turn_in_place`

use walker2::testkit::{write_trace_svg, Checks, FlatGround, Runner};
use walker2::{Walker, WalkerCommand, WalkerSpec};

fn angle_delta(to: f32, from: f32) -> f32 {
    (to - from).sin().atan2((to - from).cos())
}

fn main() {
    std::process::exit(run());
}

fn run() -> i32 {
    println!("== 02 turn in place: idle 1s, face 150deg, then face 30deg ==");
    let mut checks = Checks::new();
    let ground = FlatGround;
    let spec = WalkerSpec::biped();
    let mut walker = Walker::new(spec, &ground);
    let mut runner = Runner::new(1.0 / 60.0);

    runner.run(&mut walker, &ground, 1.0, |_, _| WalkerCommand::IDLE);
    let steps_at_start = walker.validation().total_steps;

    let target_a = 150f32.to_radians();
    runner.run(&mut walker, &ground, 6.0, |_, _| WalkerCommand::face(target_a));
    let steps_after_a = walker.validation().total_steps;
    report_turn(&mut checks, &walker, "turn A (150deg)", target_a, steps_after_a - steps_at_start);

    let target_b = 30f32.to_radians();
    runner.run(&mut walker, &ground, 6.0, |_, _| WalkerCommand::face(target_b));
    let steps_after_b = walker.validation().total_steps;
    report_turn(&mut checks, &walker, "turn B (30deg)", target_b, steps_after_b - steps_after_a);

    checks.check_le(
        "drift while turning (m)",
        walker.position().distance(glam::Vec3::new(0.0, walker.position().y, 0.0)),
        1.5,
    );
    runner.print_reports();
    let _ = write_trace_svg("target/traces/02_turn_in_place.svg", &runner, "02 turn in place (two turns)");
    checks.finish("02_turn_in_place")
}

fn report_turn(checks: &mut Checks, walker: &Walker, name: &str, target: f32, steps: u32) {
    let sig = walker.signals();
    checks.check_le(
        &format!("{name}: yaw tracks face_yaw (rad err)"),
        angle_delta(target, walker.yaw()).abs(),
        0.05,
    );
    checks.check_ge(&format!("{name}: untwist steps taken"), steps as f32, 1.0);
    let twist = sig
        .support_yaw
        .map(|sy| angle_delta(walker.yaw(), sy).abs())
        .unwrap_or(99.0);
    checks.check_le(&format!("{name}: residual support twist (rad)"), twist, 0.50);
    let right_axis = glam::Vec3::new(walker.yaw().cos(), 0.0, -walker.yaw().sin());
    let lateral = (sig.legs[1].foot - sig.legs[0].foot).dot(right_axis);
    checks.check_ge(&format!("{name}: feet not crossed (lateral m)"), lateral, 0.2);
    println!("  {name}: steps={steps} twist={twist:.2} lateral={lateral:.2}");
}

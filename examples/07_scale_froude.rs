//! 07 — Froude scaling: one tuning, every size.
//!
//! Size is exclusively the spawn-time `scale`. The sim runs in canonical
//! units with canonical time `dt/sqrt(scale)`, so world speed scales with
//! sqrt(scale) and cadence with 1/sqrt(scale): a 4x giant moves 2x faster
//! in absolute terms but strides at half the rate — genuinely ponderous —
//! while a 0.28x "human-sized" walker scurries. This is the mechanism
//! that lets the same spec drive RTS-scale crowds and hero giants.
//!
//! Run: `cargo run --example 07_scale_froude`

use glam::Vec3;
use walker2::testkit::{Checks, FlatGround};
use walker2::{Walker, WalkerCommand, WalkerSpec};

fn main() {
    std::process::exit(run());
}

fn run() -> i32 {
    println!("== 07 Froude scaling: sprint at scale 0.28 / 1.0 / 4.0 ==");
    let mut checks = Checks::new();
    let ground = FlatGround;
    let spec = WalkerSpec::biped();
    let dt = 1.0 / 60.0;
    let warmup = 3.0f32;
    let measure = 8.0f32;

    let mut rows: Vec<(f32, f32, f32)> = Vec::new();
    for scale in [0.28f32, 1.0, 4.0] {
        let mut walker = Walker::new_at(spec, &ground, 0.0, 0.0, 0.0, scale);
        let mut t = 0.0f32;
        let mut speed_sum = 0.0;
        let mut speed_n = 0u32;
        let mut steps_at_warmup = 0u32;
        while t < warmup + measure {
            walker.step(&ground, WalkerCommand::sprint(Vec3::Z, 0.0), dt);
            t += dt;
            if (t - warmup).abs() < dt * 0.5 {
                steps_at_warmup = walker.validation().total_steps;
            }
            if t >= warmup {
                let v = walker.velocity();
                speed_sum += Vec3::new(v.x, 0.0, v.z).length();
                speed_n += 1;
            }
        }
        let speed = speed_sum / speed_n as f32;
        let cadence = (walker.validation().total_steps - steps_at_warmup) as f32 / measure;
        println!(
            "  scale {scale:4.2}: speed {speed:6.2} m/s, cadence {cadence:5.2} steps/s, height ~{:.1}m",
            spec.body_height() * scale
        );
        rows.push((scale, speed, cadence));
    }

    let (s_small, v_small, c_small) = rows[0];
    let (_, v_base, c_base) = rows[1];
    let (s_big, v_big, c_big) = rows[2];
    checks.check_near(
        "speed ratio big/base ~ sqrt(4)",
        v_big / v_base,
        s_big.sqrt(),
        s_big.sqrt() * 0.15,
    );
    checks.check_near(
        "speed ratio small/base ~ sqrt(0.28)",
        v_small / v_base,
        s_small.sqrt(),
        s_small.sqrt() * 0.20,
    );
    checks.check_near(
        "cadence ratio big/base ~ 1/sqrt(4)",
        c_big / c_base,
        1.0 / s_big.sqrt(),
        0.15,
    );
    checks.check_near(
        "cadence ratio small/base ~ 1/sqrt(0.28)",
        c_small / c_base,
        1.0 / s_small.sqrt(),
        0.45,
    );
    checks.finish("07_scale_froude")
}

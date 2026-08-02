//! 09 — Crowd benchmark: RTS-scale smoke test.
//!
//! 180 walkers (30 Full / 60 Reduced / 90 Kinematic — a plausible LOD
//! distribution) wander rolling terrain for 10 simulated seconds. Each
//! tier runs in its own world so the cost ladder is measured separately.
//! Checks are sanity (everyone finite, upright, footfalls flowing);
//! numbers are the point — run with --release for real timings.
//!
//! Run: `cargo run --release --example 09_crowd_bench`

use std::time::Instant;

use glam::Vec3;
use walker2::testkit::{Checks, RollingGround};
use walker2::{Fidelity, StepCtx, WalkerCommand, WalkerEvent, WalkerSpawnDesc, WalkerSpec, WalkerWorld};

fn main() {
    std::process::exit(run());
}

fn run() -> i32 {
    println!("== 09 crowd bench: 30 Full / 60 Reduced / 90 Kinematic, 10s on rolling ground ==");
    let mut checks = Checks::new();
    let ground = RollingGround { amplitude: 0.5, wavelength: 11.0 };
    let spec = WalkerSpec::biped();
    let dt = 1.0 / 60.0;
    let ticks = 600;

    let mut total_footfalls = 0usize;
    for (name, fidelity, count) in [
        ("Full", Fidelity::Full, 30usize),
        ("Reduced", Fidelity::Reduced, 60),
        ("Kinematic", Fidelity::Kinematic, 90),
    ] {
        let mut world = WalkerWorld::default();
        let mut handles = Vec::new();
        for i in 0..count {
            let col = (i % 10) as f32;
            let row = (i / 10) as f32;
            let desc = WalkerSpawnDesc::new(spec, col * 8.0, row * 8.0, 0.0)
                .with_fidelity(fidelity);
            handles.push(world.spawn(&ground, desc));
        }

        let mut events = Vec::new();
        let mut footfalls = 0usize;
        let start = Instant::now();
        for tick in 0..ticks {
            let t = tick as f32 * dt;
            // Deterministic wandering: each walker orbits a drifting
            // waypoint; direction varies by index and time.
            for (i, &h) in handles.iter().enumerate() {
                let phase = i as f32 * 0.61 + t * 0.25;
                let dir = Vec3::new(phase.cos(), 0.0, phase.sin());
                let sprint = i % 4 == 0;
                world.set_command(
                    h,
                    WalkerCommand { move_dir: dir, face_yaw: phase, sprint },
                );
            }
            world.step(StepCtx { ground: &ground, dt });
            world.drain_events(&mut events);
            footfalls += events
                .iter()
                .filter(|e| matches!(e, WalkerEvent::Footfall { .. }))
                .count();
            events.clear();
        }
        let elapsed = start.elapsed();
        let us_per_walker_tick = elapsed.as_micros() as f64 / (ticks * count) as f64;
        println!(
            "  {name:9} x{count:3}: {:7.1} ms total, {us_per_walker_tick:6.2} us/walker/tick, {footfalls} footfalls",
            elapsed.as_secs_f64() * 1000.0
        );
        total_footfalls += footfalls;

        let mut all_ok = true;
        for &h in &handles {
            let w = world.walker(h).unwrap();
            let pos = w.position();
            let ground_h = {
                use walker2::GroundQuery;
                ground.height_at(pos.x, pos.z)
            };
            if !pos.is_finite() || pos.y < ground_h - 1.0 || pos.y > ground_h + 20.0 {
                all_ok = false;
            }
        }
        checks.check(&format!("{name}: all walkers finite & plausible height"), all_ok, format!("{count} walkers"));
    }

    checks.check_ge("footfall events flowing", total_footfalls as f32, 2000.0);
    checks.finish("09_crowd_bench")
}

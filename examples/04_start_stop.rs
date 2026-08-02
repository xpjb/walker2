//! 04 — Sprint launch and hard brake: the cart-pole test.
//!
//! What makes running look load-bearing is where the feet plant relative
//! to the COM under acceleration: behind it while launching, ahead of it
//! while braking (linear-inverted-pendulum feedforward). This example
//! measures exactly that, plus time-to-speed and stopping distance.
//!
//! Run: `cargo run --example 04_start_stop`

use glam::Vec3;
use walker2::testkit::{write_trace_svg, Checks, FlatGround, Runner};
use walker2::{Walker, WalkerCommand, WalkerSpec};

fn main() {
    std::process::exit(run());
}

fn run() -> i32 {
    println!("== 04 start/stop: idle 1s -> sprint +Z 5s -> hard stop ==");
    let mut checks = Checks::new();
    let ground = FlatGround;
    let spec = WalkerSpec::biped();
    let mut walker = Walker::new(spec, &ground);
    let mut runner = Runner::new(1.0 / 60.0);

    let brake_at = 6.0;
    runner.run(&mut walker, &ground, 10.0, |t, _| {
        if t < 1.0 {
            WalkerCommand::IDLE
        } else if t < brake_at {
            WalkerCommand::sprint(Vec3::Z, 0.0)
        } else {
            WalkerCommand::IDLE
        }
    });

    // Time to 90% sprint speed after the command flips on at t=1.
    let target = spec.speed.sprint * 0.9;
    let t90 = runner
        .samples
        .iter()
        .find(|s| s.t > 1.0 && Vec3::new(s.vel.x, 0.0, s.vel.z).length() >= target)
        .map(|s| s.t - 1.0);
    match t90 {
        Some(t) => checks.check_le("time to 90% sprint speed (s)", t, 3.0),
        None => checks.check("time to 90% sprint speed", false, "never reached".into()),
    }

    // Stop time and overshoot after the brake.
    let stop_t = runner
        .samples
        .iter()
        .find(|s| s.t > brake_at && Vec3::new(s.vel.x, 0.0, s.vel.z).length() < 0.5)
        .map(|s| s.t - brake_at);
    match stop_t {
        Some(t) => checks.check_le("stop time from sprint (s)", t, 2.2),
        None => checks.check("stop time from sprint", false, "never stopped".into()),
    }
    let brake_pos = runner
        .samples
        .iter()
        .find(|s| s.t >= brake_at)
        .map(|s| s.pos.z)
        .unwrap_or(0.0);
    let final_pos = runner.samples.last().unwrap().pos.z;
    checks.check_le("brake overshoot (m)", final_pos - brake_pos, spec.speed.sprint * 0.9);

    // Cart-pole signature: brake-window plants land AHEAD of the COM
    // (positive plant.z - com.z), launch-window plants land at/behind it.
    let plant_offset = |from: f32, to: f32| -> (f32, usize) {
        let mut sum = 0.0;
        let mut n = 0;
        for p in runner.plants_between(from, to) {
            let com_z = runner
                .samples
                .iter()
                .min_by(|a, b| (a.t - p.t).abs().total_cmp(&(b.t - p.t).abs()))
                .map(|s| s.pos.z)
                .unwrap_or(0.0);
            sum += p.pos.z - com_z;
            n += 1;
        }
        (if n > 0 { sum / n as f32 } else { 0.0 }, n)
    };
    let (launch_off, launch_n) = plant_offset(1.0, 2.2);
    let (cruise_off, cruise_n) = plant_offset(3.5, brake_at);
    let (brake_off, brake_n) = plant_offset(brake_at, brake_at + 1.4);
    println!(
        "  plant offset vs COM at landing: launch {launch_off:+.2}m (n={launch_n}), cruise {cruise_off:+.2}m (n={cruise_n}), brake {brake_off:+.2}m (n={brake_n})"
    );
    // At cruise the sprinting body overtakes the foot mid-swing, so plants
    // land slightly BEHIND the COM. Braking must shift them forward (the
    // LIP/cart-pole signature: support ahead of COM to decelerate).
    checks.check(
        "braking plants shift ahead of cruise plants",
        brake_off > cruise_off + 0.15,
        format!("brake {brake_off:+.2} vs cruise {cruise_off:+.2}"),
    );
    checks.check_ge("braking plants not behind COM (m)", brake_off, -0.05);

    let v = walker.validation();
    checks.check_le("same-foot double steps", v.total_same_foot as f32, 1.0);
    checks.check_le("max tilt (rad)", runner.max_tilt(), 0.25);
    runner.print_reports();
    let _ = write_trace_svg("target/traces/04_start_stop.svg", &runner, "04 sprint launch + hard brake");
    checks.finish("04_start_stop")
}

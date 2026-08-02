//! Gait invariants ported/adapted from the petrogradrevival test suite,
//! plus new invariants for the library-specific behavior (untwist,
//! fidelity tiers, Froude scaling, determinism).

use glam::Vec3;
use walker2::testkit::FlatGround;
use walker2::{Fidelity, GroundQuery, Walker, WalkerCommand, WalkerSpec};

const DT: f32 = 1.0 / 60.0;

fn angle_delta(to: f32, from: f32) -> f32 {
    (to - from).sin().atan2((to - from).cos())
}

fn run(
    walker: &mut Walker,
    ground: &impl GroundQuery,
    seconds: f32,
    mut cmd: impl FnMut(f32) -> WalkerCommand,
) {
    let steps = (seconds / DT).round() as usize;
    for i in 0..steps {
        walker.step(ground, cmd(i as f32 * DT), DT);
    }
}

fn planar_speed(walker: &Walker) -> f32 {
    let v = walker.velocity();
    Vec3::new(v.x, 0.0, v.z).length()
}

#[test]
fn leg_chain_preserves_link_lengths() {
    for spec in [WalkerSpec::biped(), WalkerSpec::longstrider(), WalkerSpec::humanoid()] {
        let ground = FlatGround;
        let mut walker = Walker::new(spec, &ground);
        // Walk a bit so the pose is generic, then measure.
        run(&mut walker, &ground, 2.0, |_| WalkerCommand::walk(Vec3::Z, 0.0));
        for i in 0..2 {
            let chain = walker.leg_chain(i);
            let d1 = chain.hip.distance(chain.knee);
            let d2 = chain.knee.distance(chain.hock);
            let d3 = chain.hock.distance(chain.foot);
            assert!((d1 - spec.hip_link).abs() < 0.02, "hip link {d1} vs {}", spec.hip_link);
            assert!(
                (d2 - spec.reverse_link).abs() < 0.02,
                "reverse link {d2} vs {}",
                spec.reverse_link
            );
            assert!((d3 - spec.shin_link).abs() < 0.02, "shin link {d3} vs {}", spec.shin_link);
        }
    }
}

#[test]
fn all_presets_move_forward_under_sprint() {
    for (name, spec) in [
        ("biped", WalkerSpec::biped()),
        ("longstrider", WalkerSpec::longstrider()),
        ("humanoid", WalkerSpec::humanoid()),
    ] {
        for fidelity in [Fidelity::Full, Fidelity::Reduced] {
            let ground = FlatGround;
            let mut walker = Walker::new(spec, &ground);
            walker.set_fidelity(fidelity);
            run(&mut walker, &ground, 8.0, |_| WalkerCommand::sprint(Vec3::Z, 0.0));
            let z = walker.position().z;
            let expected = spec.speed.sprint * 8.0;
            assert!(
                z > expected * 0.55,
                "{name} {fidelity:?}: advanced {z:.1}, expected > {:.1}",
                expected * 0.55
            );
            assert!(
                walker.validation().total_same_foot <= 1,
                "{name} {fidelity:?}: same-foot steps {}",
                walker.validation().total_same_foot
            );
            assert_eq!(
                walker.validation().total_double_swing_frames,
                0,
                "{name} {fidelity:?}: double swing frames"
            );
        }
    }
}

#[test]
fn walk_speed_tracks_spec() {
    let spec = WalkerSpec::biped();
    let ground = FlatGround;
    let mut walker = Walker::new(spec, &ground);
    run(&mut walker, &ground, 10.0, |_| WalkerCommand::walk(Vec3::Z, 0.0));
    let speed = planar_speed(&walker);
    assert!(
        (speed - spec.speed.walk).abs() < spec.speed.walk * 0.25,
        "walk speed {speed:.2} vs spec {:.2}",
        spec.speed.walk
    );
}

#[test]
fn idle_magic_turn_untwists_feet() {
    let spec = WalkerSpec::biped();
    let ground = FlatGround;
    let mut walker = Walker::new(spec, &ground);
    // Settle, then command a 150-degree facing change with zero movement.
    run(&mut walker, &ground, 1.0, |_| WalkerCommand::IDLE);
    let steps_before = walker.validation().total_steps;
    let target = 150f32.to_radians();
    run(&mut walker, &ground, 6.0, |_| WalkerCommand::face(target));

    assert!(
        angle_delta(target, walker.yaw()).abs() < 0.05,
        "yaw should track face_yaw: yaw {:.2} target {target:.2}",
        walker.yaw()
    );
    let steps_after = walker.validation().total_steps;
    assert!(steps_after > steps_before, "untwist should re-plant feet");
    let sig = walker.signals();
    let support_yaw = sig.support_yaw.expect("support line defined");
    let twist = angle_delta(walker.yaw(), support_yaw).abs();
    assert!(
        twist < spec.untwist_gate + 0.20,
        "support line should follow the body: residual twist {twist:.2}"
    );
    // Feet must not be crossed: in the body frame, the left anchor sits
    // left of the right anchor.
    let right_axis = Vec3::new(walker.yaw().cos(), 0.0, -walker.yaw().sin());
    let lateral = (sig.legs[1].foot - sig.legs[0].foot).dot(right_axis);
    assert!(lateral > 0.2, "feet crossed after untwist: lateral {lateral:.2}");
}

#[test]
fn facing_decoupled_strafe_and_backpedal() {
    let spec = WalkerSpec::biped();
    let ground = FlatGround;
    let mut walker = Walker::new(spec, &ground);
    // Face +Z the whole time; move +X (strafe) then -Z (backpedal).
    run(&mut walker, &ground, 6.0, |_| WalkerCommand::walk(Vec3::X, 0.0));
    let after_strafe = walker.position();
    assert!(after_strafe.x > 10.0, "strafe distance {:.1}", after_strafe.x);
    assert!(walker.yaw().abs() < 0.10, "facing held during strafe: {:.2}", walker.yaw());

    run(&mut walker, &ground, 6.0, |_| WalkerCommand::walk(-Vec3::Z, 0.0));
    let after_back = walker.position();
    assert!(
        after_back.z < after_strafe.z - 8.0,
        "backpedal distance {:.1}",
        after_strafe.z - after_back.z
    );
    assert!(walker.yaw().abs() < 0.10, "facing held during backpedal: {:.2}", walker.yaw());
    assert!(walker.validation().total_same_foot <= 2);
}

#[test]
fn sprint_brake_stops_quickly() {
    let spec = WalkerSpec::biped();
    let ground = FlatGround;
    let mut walker = Walker::new(spec, &ground);
    run(&mut walker, &ground, 5.0, |_| WalkerCommand::sprint(Vec3::Z, 0.0));
    assert!(planar_speed(&walker) > spec.speed.sprint * 0.8);
    let brake_point = walker.position();
    run(&mut walker, &ground, 2.5, |_| WalkerCommand::IDLE);
    assert!(planar_speed(&walker) < 0.5, "still moving at {:.2}", planar_speed(&walker));
    let overshoot = walker.position().z - brake_point.z;
    assert!(
        overshoot < spec.speed.sprint * 0.9,
        "brake overshoot {overshoot:.1} too long"
    );
}

#[test]
fn lateral_impulse_recovers_without_falling() {
    let spec = WalkerSpec::biped();
    let ground = FlatGround;
    let mut walker = Walker::new(spec, &ground);
    run(&mut walker, &ground, 1.5, |_| WalkerCommand::IDLE);
    walker.apply_impulse(Vec3::X * 3.5);
    let steps_before = walker.validation().total_steps;
    run(&mut walker, &ground, 3.0, |_| WalkerCommand::IDLE);
    assert!(planar_speed(&walker) < 0.6, "recovered speed {:.2}", planar_speed(&walker));
    assert!(
        walker.validation().total_steps > steps_before,
        "impulse should force recovery steps"
    );
    let (pitch, roll) = walker.cabin_tilt();
    assert!(pitch.abs() < 0.20 && roll.abs() < 0.20, "tilt bounded after recovery");
    let pos = walker.position();
    assert!(pos.y > spec.body_height() * 0.5, "still standing");
}

#[test]
fn froude_scaling_speeds_follow_sqrt_scale() {
    let spec = WalkerSpec::biped();
    let ground = FlatGround;
    let mut speeds = Vec::new();
    for scale in [1.0f32, 4.0] {
        let mut walker = Walker::new_at(spec, &ground, 0.0, 0.0, 0.0, scale);
        run(&mut walker, &ground, 10.0, |_| WalkerCommand::sprint(Vec3::Z, 0.0));
        speeds.push(planar_speed(&walker));
    }
    let ratio = speeds[1] / speeds[0].max(0.01);
    assert!(
        (ratio - 2.0).abs() < 0.35,
        "scale-4 walker should be ~2x faster (sqrt scale): ratio {ratio:.2} ({:.2} vs {:.2})",
        speeds[1],
        speeds[0]
    );
}

#[test]
fn kinematic_tier_walks_and_alternates() {
    let spec = WalkerSpec::biped();
    let ground = FlatGround;
    let mut walker = Walker::new(spec, &ground);
    walker.set_fidelity(Fidelity::Kinematic);
    run(&mut walker, &ground, 8.0, |_| WalkerCommand::walk(Vec3::Z, 0.0));
    let speed = planar_speed(&walker);
    assert!(
        (speed - spec.speed.walk).abs() < spec.speed.walk * 0.25,
        "kinematic speed {speed:.2} vs {:.2}",
        spec.speed.walk
    );
    assert!(walker.validation().total_steps > 10);
    assert!(walker.validation().total_same_foot <= 1);
    assert!(walker.position().is_finite());
}

#[test]
fn fidelity_switch_is_continuous() {
    let spec = WalkerSpec::biped();
    let ground = FlatGround;
    let mut walker = Walker::new(spec, &ground);
    run(&mut walker, &ground, 4.0, |_| WalkerCommand::walk(Vec3::Z, 0.0));
    for fidelity in [Fidelity::Reduced, Fidelity::Kinematic, Fidelity::Full] {
        let before = walker.position();
        walker.set_fidelity(fidelity);
        walker.step(&ground, WalkerCommand::walk(Vec3::Z, 0.0), DT);
        let jump = walker.position().distance(before);
        assert!(
            jump < 0.40,
            "tier switch to {fidelity:?} popped: moved {jump:.3} in one tick"
        );
        run(&mut walker, &ground, 1.0, |_| WalkerCommand::walk(Vec3::Z, 0.0));
    }
}

#[test]
fn stepping_is_deterministic() {
    let spec = WalkerSpec::biped();
    let ground = FlatGround;
    let script = |t: f32| {
        if t < 2.0 {
            WalkerCommand::sprint(Vec3::Z, 0.3)
        } else if t < 4.0 {
            WalkerCommand::walk(Vec3::X, 1.2)
        } else {
            WalkerCommand::face(2.0)
        }
    };
    let mut a = Walker::new(spec, &ground);
    let mut b = Walker::new(spec, &ground);
    run(&mut a, &ground, 6.0, script);
    run(&mut b, &ground, 6.0, script);
    assert_eq!(a.position().to_array(), b.position().to_array());
    assert_eq!(a.yaw().to_bits(), b.yaw().to_bits());
}

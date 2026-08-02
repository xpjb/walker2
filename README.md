# walker2

Procedural biped locomotion library, extracted and library-ified from the
petrogradrevival mech walker. Physics-first — there are no keyframes
anywhere: capture-point / Raibert step planning, an XPBD-style stance
constraint stack, spring-leg contact dynamics, and analytic IK chains
produce the walk. Intended as the **lower-body / COM ground truth** for an
Overgrowth-style pose layer on top.

```rust
use glam::Vec3;
use walker2::{Walker, WalkerCommand, WalkerSpec, GroundQuery};

struct Flat;
impl GroundQuery for Flat {
    fn height_at(&self, _x: f32, _z: f32) -> f32 { 0.0 }
}

let ground = Flat;
let mut walker = Walker::new_at(WalkerSpec::biped(), &ground, 0.0, 0.0, 0.0, 1.0);
loop {
    // World-space move vector + facing, fully decoupled (M&B model).
    walker.step(&ground, WalkerCommand::walk(Vec3::Z, 0.4), 1.0 / 60.0);
    let signals = walker.signals();   // rig signals for a pose layer
    let chain = walker.leg_chain(0);  // IK joints for a skeleton
    for hit in walker.take_footfalls() { /* audio / dust / shake */ }
    # break;
}
```

## Design decisions (vs. the original mech code)

- **Facing is "magic".** Body yaw is a spring toward `face_yaw`; feet never
  plan rotation. When idle twist exceeds a comfort gate, feet re-plant at
  rotated home positions ("untwist"). `move_dir` is a world-space vector
  fully decoupled from facing — strafe, backpedal, and diagonals fall out
  of the omnidirectional planner (with sidesteps align-scaled shorter than
  strides).
- **The LIP family is kept**: capture point, Raibert targeting, cart-pole
  feedforward/scoring, acceleration lean. This is what makes starts,
  stops, and braking read as weight (see example 04: braking plants land
  ahead of the COM, cruise plants behind).
- **Cut**: per-step action sampling + slip-preview rollouts (mech
  hydraulic flavor, ~10x planning cost), the 7x7 attitude candidate
  search, all multileg machinery, and game-specific features.
- **Spec-driven**: `WalkerSpec` replaces the kind enum + ~60 match tables.
  Presets: `biped()` (the proven baseline), `longstrider()`,
  `humanoid()` (experimental forward-knee silhouette).
- **Melee hook**: `apply_impulse()` — shoves/parries become emergent
  recovery footwork from the balance loop, no stagger animations.

## Size = `scale`, not spec values

Specs are in **canonical units**. Actual size comes exclusively from the
spawn-time `scale`, which Froude-scales the whole sim: world speed scales
with `sqrt(scale)`, cadence with `1/sqrt(scale)` — giants are genuinely
ponderous, small walkers scurry, from one tuning (example 07 verifies the
ratios). Don't author "small" specs; spawn small scales.

## Fidelity tiers

One state layout, three cost levels, switchable at runtime without
popping (example 08):

| Tier | What | Cost* | Endpoint vs Full |
|---|---|---|---|
| `Full` | candidate grid + full scoring, 14 substeps | ~17 us/walker/tick | — |
| `Reduced` | nominal target + cart-pole feedforward, 4 substeps | ~7 us | within ~1% |
| `Kinematic` | phase-driven gait, no balance dynamics | ~1 us | within ~10% |

*release build, one core of a desktop CPU; see example 09 (crowd bench).

## Validation examples

Progressively structured; each prints PASS/FAIL checks (nonzero exit on
failure) and most write a top-down trace SVG to `target/traces/`:

| # | Example | Validates |
|---|---|---|
| 01 | `01_flat_walk` | walk/sprint speeds, clean alternation, all presets |
| 02 | `02_turn_in_place` | magic yaw + untwist re-planting, no crossed legs |
| 03 | `03_strafe_backpedal` | facing-decoupled movement (strafe/backpedal/diagonal) |
| 04 | `04_start_stop` | launch/brake, cart-pole plant signature, stop distance |
| 05 | `05_terrain` | rolling hills + 14% grade, ride-height motor, tilt bounds |
| 06 | `06_impulse_recovery` | melee shoves -> emergent recovery footwork |
| 07 | `07_scale_froude` | sqrt(scale) speed / 1/sqrt(scale) cadence ratios |
| 08 | `08_fidelity_tiers` | tier equivalence, popping-free switching, cost ladder |
| 09 | `09_crowd_bench` | 180-walker RTS-scale smoke + per-tier timing (use `--release`) |

```bash
cargo test                                  # gait invariants
cargo run --example 01_flat_walk            # start of the ladder
cargo run --release --example 09_crowd_bench
```

### Visual gallery

The `visual` example runs the same numbered scenario inputs in an
interactive `pixels` gallery. It renders `Walker::part_poses()` with
screen-space SDF capsules, joints, and rounded boxes, plus COM, balance,
capture, facing, contact, target, footfall, terrain, and impulse overlays.

```bash
cargo run --features visual-example --example visual
cargo run --features visual-example --example visual -- --scenario 04
cargo run --features visual-example --example visual -- --capture-all target/visual
```

Use `1`-`9` to select a scenario, Space to pause, `.` to step one fixed
tick, `R` to reset, `V` to change view, Tab to toggle overlays, `+`/`-`
to change speed, and `[`/`]` or the mouse wheel to zoom. Capture mode
writes three deterministic PNG review frames per scenario; they are
human-review artifacts, not pixel-equality tests.


## Outputs

- `Walker::signals() -> RigSignals` — pose-layer inputs: per-leg
  contact/phase/targets, capture error, balance vector, control risk,
  support-line yaw (map to pelvis twist), swing wave.
- `Walker::leg_chain(i) -> LegChain` — world-space IK joints for skinning.
- `Walker::part_poses()` — renderer-neutral oriented boxes; instanceable
  as-is for brushed-model / RTS crowds.
- Events: footfalls (with strength), actuator load/motion, power demand —
  audio and FX hooks.

## Known gaps / roadmap

- No flight phase: sprint is a fast dynamic walk (one foot always down).
  A true run needs a short ballistic phase + landing — contained change in
  the support-height solver.
- Strafe steady-state is ~35% of walk speed (deliberately short
  sidesteps); tune `align_scale` mapping or cap strafe input host-side.
- Quadruped (mounts) is intentionally NOT the old multileg crawl; it
  should arrive as a separate paired-phase gait model.
- The Overgrowth-style pose layer (arm swing, lean shaping, head look,
  landing crouches) lives above this crate by design — `RigSignals` is its
  contract.

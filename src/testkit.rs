//! Headless validation helpers shared by the examples and tests: canned
//! terrains, a fixed-timestep runner that records telemetry, PASS/FAIL
//! check bookkeeping, and a top-down SVG trace writer for eyeballing gait
//! quality without a renderer.

use std::io::Write as _;
use std::path::Path;

use glam::Vec3;

use crate::{command::WalkerCommand, ground::GroundQuery, walker::Walker};

// ----------------------------------------------------------------------
// Terrains
// ----------------------------------------------------------------------

pub struct FlatGround;

impl GroundQuery for FlatGround {
    fn height_at(&self, _x: f32, _z: f32) -> f32 {
        0.0
    }
}

/// Gentle rolling hills: `amp * (sin(x/wl) + cos(z/wl'))`.
pub struct RollingGround {
    pub amplitude: f32,
    pub wavelength: f32,
}

impl GroundQuery for RollingGround {
    fn height_at(&self, x: f32, z: f32) -> f32 {
        let w = self.wavelength.max(0.01);
        self.amplitude * ((x / w).sin() + (z / (w * 1.37)).cos())
    }
}

/// Constant grade along +Z (rise per unit run).
pub struct SlopeGround {
    pub grade: f32,
}

impl GroundQuery for SlopeGround {
    fn height_at(&self, _x: f32, z: f32) -> f32 {
        z * self.grade
    }
}

// ----------------------------------------------------------------------
// Telemetry runner
// ----------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub t: f32,
    pub pos: Vec3,
    pub vel: Vec3,
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
    pub support_yaw: Option<f32>,
    pub left_contact: bool,
    pub right_contact: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Plant {
    pub t: f32,
    pub pos: Vec3,
    pub strength: f32,
}

/// Fixed-timestep telemetry recorder. Drives a `Walker` with a
/// command-per-time closure and keeps a full sample/footfall/report log.
pub struct Runner {
    pub dt: f32,
    pub time: f32,
    pub samples: Vec<Sample>,
    pub plants: Vec<Plant>,
    pub reports: Vec<String>,
}

impl Runner {
    pub fn new(dt: f32) -> Self {
        Self {
            dt,
            time: 0.0,
            samples: Vec::new(),
            plants: Vec::new(),
            reports: Vec::new(),
        }
    }

    pub fn run<G: GroundQuery>(
        &mut self,
        walker: &mut Walker,
        ground: &G,
        seconds: f32,
        mut command: impl FnMut(f32, &Walker) -> WalkerCommand,
    ) {
        let steps = (seconds / self.dt).round() as usize;
        for _ in 0..steps {
            let cmd = command(self.time, walker);
            walker.step(ground, cmd, self.dt);
            self.time += self.dt;
            for f in walker.take_footfalls() {
                self.plants.push(Plant {
                    t: self.time,
                    pos: f.pos,
                    strength: f.strength,
                });
            }
            if let Some(report) = walker.take_gait_report() {
                self.reports.push(format!("[t={:6.2}] {report}", self.time));
            }
            let sig = walker.signals();
            self.samples.push(Sample {
                t: self.time,
                pos: sig.pos,
                vel: sig.vel,
                yaw: sig.yaw,
                pitch: sig.pitch,
                roll: sig.roll,
                support_yaw: sig.support_yaw,
                left_contact: sig.legs[0].contact,
                right_contact: sig.legs[1].contact,
            });
        }
    }

    /// Average planar speed over the trailing `seconds`.
    pub fn avg_speed_last(&self, seconds: f32) -> f32 {
        let from = self.time - seconds;
        let mut sum = 0.0;
        let mut n = 0;
        for s in &self.samples {
            if s.t >= from {
                sum += Vec3::new(s.vel.x, 0.0, s.vel.z).length();
                n += 1;
            }
        }
        if n > 0 {
            sum / n as f32
        } else {
            0.0
        }
    }

    pub fn max_tilt(&self) -> f32 {
        self.samples
            .iter()
            .map(|s| s.pitch.abs().max(s.roll.abs()))
            .fold(0.0, f32::max)
    }

    /// Footfalls whose timestamps fall inside [from, to).
    pub fn plants_between(&self, from: f32, to: f32) -> impl Iterator<Item = &Plant> {
        self.plants.iter().filter(move |p| p.t >= from && p.t < to)
    }

    pub fn print_reports(&self) {
        for r in &self.reports {
            println!("  {r}");
        }
    }
}

// ----------------------------------------------------------------------
// PASS/FAIL bookkeeping
// ----------------------------------------------------------------------

/// Collects named checks; `finish()` prints a summary and returns the
/// process exit code (0 = all passed), so examples double as validation
/// scripts.
#[derive(Default)]
pub struct Checks {
    results: Vec<(String, bool, String)>,
}

impl Checks {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn check(&mut self, name: &str, pass: bool, detail: String) {
        let mark = if pass { "PASS" } else { "FAIL" };
        println!("  [{mark}] {name}: {detail}");
        self.results.push((name.to_string(), pass, detail));
    }

    pub fn check_near(&mut self, name: &str, value: f32, expected: f32, tolerance: f32) {
        self.check(
            name,
            (value - expected).abs() <= tolerance,
            format!("value {value:.3}, expected {expected:.3} ± {tolerance:.3}"),
        );
    }

    pub fn check_le(&mut self, name: &str, value: f32, limit: f32) {
        self.check(
            name,
            value <= limit,
            format!("value {value:.3}, limit {limit:.3}"),
        );
    }

    pub fn check_ge(&mut self, name: &str, value: f32, floor: f32) {
        self.check(
            name,
            value >= floor,
            format!("value {value:.3}, floor {floor:.3}"),
        );
    }

    pub fn all_passed(&self) -> bool {
        self.results.iter().all(|(_, p, _)| *p)
    }

    /// Print a summary line and return a process exit code.
    pub fn finish(self, title: &str) -> i32 {
        let passed = self.results.iter().filter(|(_, p, _)| *p).count();
        let total = self.results.len();
        let ok = passed == total;
        println!(
            "\n{title}: {passed}/{total} checks passed{}",
            if ok { "" } else { "  <-- FAILURES" }
        );
        if ok {
            0
        } else {
            1
        }
    }
}

// ----------------------------------------------------------------------
// SVG trace writer
// ----------------------------------------------------------------------

/// Write a top-down SVG of the run: COM path (blue), left/right foot
/// plants (orange/teal circles sized by strength), start marker. Open in
/// any browser to eyeball stride pattern, straightness, and untwist
/// behavior without a renderer.
pub fn write_trace_svg(
    path: impl AsRef<Path>,
    runner: &Runner,
    title: &str,
) -> std::io::Result<()> {
    let path = path.as_ref();
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }

    let mut min = Vec3::splat(f32::MAX);
    let mut max = Vec3::splat(f32::MIN);
    let mut grow = |p: Vec3| {
        min = min.min(p);
        max = max.max(p);
    };
    for s in &runner.samples {
        grow(s.pos);
    }
    for p in &runner.plants {
        grow(p.pos);
    }
    if runner.samples.is_empty() {
        min = Vec3::ZERO;
        max = Vec3::ONE;
    }
    let pad = 1.5f32;
    let (x0, z0) = (min.x - pad, min.z - pad);
    let (x1, z1) = (max.x + pad, max.z + pad);
    let w = (x1 - x0).max(1.0);
    let h = (z1 - z0).max(1.0);
    let view_w = 900.0f32;
    let view_h = (view_w * h / w).clamp(240.0, 1600.0);
    let sx = view_w / w;
    let sz = view_h / h;
    let map = |p: Vec3| ((p.x - x0) * sx, view_h - (p.z - z0) * sz);

    let mut svg = String::new();
    svg.push_str(&format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 {view_w:.0} {view_h:.0}\" style=\"background:#101418\">\n"
    ));
    svg.push_str(&format!(
        "<text x=\"12\" y=\"22\" fill=\"#c8d2dc\" font-family=\"monospace\" font-size=\"15\">{title}</text>\n"
    ));

    // COM path.
    if runner.samples.len() > 1 {
        svg.push_str("<polyline fill=\"none\" stroke=\"#4f8fd0\" stroke-width=\"1.6\" points=\"");
        for s in &runner.samples {
            let (x, y) = map(s.pos);
            svg.push_str(&format!("{x:.1},{y:.1} "));
        }
        svg.push_str("\"/>\n");
    }

    // Foot plants: alternate colors by which foot is closer... we don't
    // record the leg index on plants, so color by alternation order.
    for (i, p) in runner.plants.iter().enumerate() {
        let (x, y) = map(p.pos);
        let r = 2.0 + p.strength * 3.0;
        let color = if i % 2 == 0 { "#e0913d" } else { "#3dc9b0" };
        svg.push_str(&format!(
            "<circle cx=\"{x:.1}\" cy=\"{y:.1}\" r=\"{r:.1}\" fill=\"none\" stroke=\"{color}\" stroke-width=\"1.4\"/>\n"
        ));
    }

    if let Some(first) = runner.samples.first() {
        let (x, y) = map(first.pos);
        svg.push_str(&format!(
            "<circle cx=\"{x:.1}\" cy=\"{y:.1}\" r=\"5\" fill=\"#d0d0d0\"/>\n"
        ));
    }

    // Speed strip along the bottom.
    if runner.samples.len() > 1 {
        let max_speed = runner
            .samples
            .iter()
            .map(|s| Vec3::new(s.vel.x, 0.0, s.vel.z).length())
            .fold(0.0f32, f32::max)
            .max(0.1);
        let strip_h = 46.0;
        let y_base = view_h - 8.0;
        svg.push_str(&format!(
            "<text x=\"12\" y=\"{:.1}\" fill=\"#8a97a3\" font-family=\"monospace\" font-size=\"11\">speed (max {max_speed:.2})</text>\n",
            y_base - strip_h - 4.0
        ));
        svg.push_str("<polyline fill=\"none\" stroke=\"#7ec97e\" stroke-width=\"1.2\" points=\"");
        let n = runner.samples.len();
        for (i, s) in runner.samples.iter().enumerate() {
            let x = 12.0 + (view_w - 24.0) * i as f32 / (n - 1) as f32;
            let speed = Vec3::new(s.vel.x, 0.0, s.vel.z).length();
            let y = y_base - strip_h * (speed / max_speed);
            svg.push_str(&format!("{x:.1},{y:.1} "));
        }
        svg.push_str("\"/>\n");
    }

    svg.push_str("</svg>\n");
    let mut file = std::fs::File::create(path)?;
    file.write_all(svg.as_bytes())?;
    println!("  trace written: {}", path.display());
    Ok(())
}

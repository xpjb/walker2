#[path = "actor_lab/render.rs"]
mod render;

use std::error::Error;
use std::fs::File;
use std::io::BufWriter;
use std::path::Path;

use brush_actor::{ActorModel, Bone, PoseDriver, PosePalette};
use chad::winit::event::{ElementState, WindowEvent};
use chad::winit::keyboard::{Key, NamedKey};
use chad::{wgpu, ChadApp, Config, Ctx, Headless, Timestep};
use glam::Vec3;
use render::{Camera, GpuRenderer};
use walker2::{GroundQuery, KneeBend, Pair, Walker, WalkerCommand, WalkerSpec};

const SIZE: (u32, u32) = (1280, 720);
const DT: f32 = 1.0 / 60.0;
const LOOP_SECONDS: f32 = 10.0;

struct Flat;
impl GroundQuery for Flat {
    fn height_at(&self, _x: f32, _z: f32) -> f32 {
        0.0
    }
}

struct LabScene {
    marine: Walker,
    ogre: Walker,
    marine_model: ActorModel,
    ogre_model: ActorModel,
    marine_previous_pose: PosePalette,
    marine_pose: PosePalette,
    ogre_previous_pose: PosePalette,
    ogre_pose: PosePalette,
    marine_pose_driver: PoseDriver,
    ogre_pose_driver: PoseDriver,
    time: f32,
}

impl LabScene {
    fn new() -> Self {
        let ground = Flat;
        let marine_model = ActorModel::marine();
        let ogre_model = ActorModel::ogre();
        let mut marine_spec = WalkerSpec::humanoid();
        marine_spec.bend = KneeBend::Normal;
        let marine = Walker::new_at(marine_spec, &ground, -1.25, 0.0, 0.0, 0.28);
        let mut ogre_spec = WalkerSpec::humanoid();
        ogre_spec.bend = KneeBend::Normal;
        ogre_spec.hip_width *= 1.24;
        ogre_spec.step_width *= 1.18;
        ogre_spec.com_offset.z += 0.16;
        // Keep scale-driven cadence, but cap the demo's canonical top speed so
        // the ogre lumbers instead of outrunning the marine in absolute units.
        let ogre_pace = 0.90;
        ogre_spec.speed = Pair::new(
            ogre_spec.speed.walk * ogre_pace,
            ogre_spec.speed.sprint * ogre_pace,
        );
        let ogre = Walker::new_at(ogre_spec, &ground, 1.45, 0.0, 0.0, 0.50);
        let mut marine_pose_driver = PoseDriver::new();
        let mut ogre_pose_driver = PoseDriver::new();
        let marine_pose = marine_pose_driver.update(&marine, &marine_model.morphology, DT);
        let ogre_pose = ogre_pose_driver.update(&ogre, &ogre_model.morphology, DT);
        Self {
            marine,
            ogre,
            marine_model,
            ogre_model,
            marine_previous_pose: marine_pose.clone(),
            marine_pose,
            ogre_previous_pose: ogre_pose.clone(),
            ogre_pose,
            marine_pose_driver,
            ogre_pose_driver,
            time: 0.0,
        }
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn step(&mut self) {
        if self.time >= LOOP_SECONDS {
            self.reset();
        }
        let ground = Flat;
        let command = scripted_command(self.time);
        self.marine_previous_pose = self.marine_pose.clone();
        self.ogre_previous_pose = self.ogre_pose.clone();
        self.marine.step(&ground, command, DT);
        self.ogre.step(&ground, command, DT);
        self.marine.take_footfalls();
        self.ogre.take_footfalls();
        self.time += DT;
        self.marine_pose =
            self.marine_pose_driver
                .update(&self.marine, &self.marine_model.morphology, DT);
        self.ogre_pose = self
            .ogre_pose_driver
            .update(&self.ogre, &self.ogre_model.morphology, DT);
    }

    fn camera(&self, alternate: bool) -> Camera {
        let marine = self.marine.position();
        let ogre = self.ogre.position();
        let target = (marine + ogre) * 0.5 + Vec3::Y * 0.70;
        let separation = marine.distance(ogre);
        let distance = 6.2 + separation * 0.20;
        let offset = if alternate {
            Vec3::new(-distance, distance * 0.38, -distance * 0.22)
        } else {
            Vec3::new(distance, distance * 0.38, distance * 0.22)
        };
        Camera {
            eye: target + offset,
            target,
        }
    }
}

fn scripted_command(time: f32) -> WalkerCommand {
    if time < 3.0 {
        WalkerCommand::walk(Vec3::Z, 0.0)
    } else if time < 6.0 {
        WalkerCommand::sprint(Vec3::Z, 0.0)
    } else if time < 8.0 {
        WalkerCommand::walk(Vec3::X, 0.0)
    } else {
        WalkerCommand::face(1.35)
    }
}

struct ActorLab {
    scene: LabScene,
    renderer: GpuRenderer,
    paused: bool,
    alternate_camera: bool,
}

impl ChadApp for ActorLab {
    fn init(ctx: &mut Ctx) -> Result<Self, String> {
        Ok(Self {
            scene: LabScene::new(),
            renderer: GpuRenderer::new(&ctx.device, ctx.surface_format, ctx.size()),
            paused: false,
            alternate_camera: false,
        })
    }

    fn event(&mut self, ctx: &mut Ctx, event: &WindowEvent) {
        match event {
            WindowEvent::CloseRequested => ctx.exit(),
            WindowEvent::KeyboardInput { event, .. }
                if event.state == ElementState::Pressed && !event.repeat =>
            {
                match &event.logical_key {
                    Key::Named(NamedKey::Escape) => ctx.exit(),
                    Key::Named(NamedKey::Space) => self.paused = !self.paused,
                    Key::Character(character) if character.eq_ignore_ascii_case("r") => {
                        self.scene.reset();
                    }
                    Key::Character(character) if character.eq_ignore_ascii_case("v") => {
                        self.alternate_camera = !self.alternate_camera;
                    }
                    _ => {}
                }
            }
            _ => {}
        }
    }

    fn update(&mut self, _ctx: &mut Ctx) {
        if !self.paused {
            self.scene.step();
        }
    }

    fn frame(&mut self, ctx: &mut Ctx, view: &wgpu::TextureView) {
        let marine_pose = self
            .scene
            .marine_previous_pose
            .interpolate(&self.scene.marine_pose, ctx.alpha());
        let ogre_pose = self
            .scene
            .ogre_previous_pose
            .interpolate(&self.scene.ogre_pose, ctx.alpha());
        let actors = [
            (&self.scene.marine_model, &marine_pose),
            (&self.scene.ogre_model, &ogre_pose),
        ];
        self.renderer.render(
            &ctx.device,
            &ctx.queue,
            view,
            ctx.size(),
            &actors,
            self.scene.camera(self.alternate_camera),
        );
    }
}

fn main() {
    if let Err(error) = entry() {
        eprintln!("actor lab failed: {error}");
        std::process::exit(1);
    }
}

fn entry() -> Result<(), Box<dyn Error>> {
    let mut arguments = std::env::args().skip(1);
    if let Some(argument) = arguments.next() {
        if argument == "--capture" {
            let path = arguments.next().ok_or("--capture requires a PNG path")?;
            let seconds = arguments
                .next()
                .map(|value| value.parse::<f32>())
                .transpose()?
                .unwrap_or(5.5);
            return capture(Path::new(&path), seconds);
        }
        if argument == "--help" || argument == "-h" {
            println!(
                "brush actor lab\n\n  --capture PATH [SECONDS]  render a headless PNG\n\nSPACE pause  R reset  V camera"
            );
            return Ok(());
        }
        return Err(format!("unknown argument: {argument}").into());
    }

    let config = Config {
        title: "brush_actor — ranger marine + bloated ogre".into(),
        size: SIZE,
        timestep: Timestep::Fixed {
            hz: 60,
            max_updates_per_frame: 8,
        },
        ..Default::default()
    };
    chad::run::<ActorLab>(config).map_err(Into::into)
}

fn capture(path: &Path, seconds: f32) -> Result<(), Box<dyn Error>> {
    let gpu = Headless::new()?;
    let target = gpu.target(SIZE);
    let mut scene = LabScene::new();
    for _ in 0..(seconds.clamp(0.0, LOOP_SECONDS) / DT).round() as usize {
        scene.step();
    }
    let elapsed = scene.time.max(DT);
    println!(
        "marine: {:.2} m/s, {:.2} steps/s | ogre: {:.2} m/s, {:.2} steps/s",
        scene.marine.position().z / elapsed,
        scene.marine.validation().total_steps as f32 / elapsed,
        scene.ogre.position().z / elapsed,
        scene.ogre.validation().total_steps as f32 / elapsed,
    );
    println!(
        "foot pitch: marine [{:+.1}, {:+.1}] deg | ogre [{:+.1}, {:+.1}] deg",
        foot_pitch(&scene.marine_pose, Bone::LeftFoot),
        foot_pitch(&scene.marine_pose, Bone::RightFoot),
        foot_pitch(&scene.ogre_pose, Bone::LeftFoot),
        foot_pitch(&scene.ogre_pose, Bone::RightFoot),
    );
    let mut renderer = GpuRenderer::new(&gpu.device, gpu.format, SIZE);
    let actors = [
        (&scene.marine_model, &scene.marine_pose),
        (&scene.ogre_model, &scene.ogre_pose),
    ];
    renderer.render(
        &gpu.device,
        &gpu.queue,
        &target.view,
        SIZE,
        &actors,
        scene.camera(false),
    );
    let rgba = gpu.read_rgba8(&target)?;
    write_png(path, &rgba, SIZE)?;
    println!("capture written: {}", path.display());
    Ok(())
}

fn foot_pitch(pose: &PosePalette, bone: Bone) -> f32 {
    let forward = pose.transforms[bone as usize]
        .transform_vector3(Vec3::Z)
        .normalize_or_zero();
    (-forward.y).clamp(-1.0, 1.0).asin().to_degrees()
}

fn write_png(path: &Path, rgba: &[u8], size: (u32, u32)) -> Result<(), Box<dyn Error>> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let file = File::create(path)?;
    let mut encoder = png::Encoder::new(BufWriter::new(file), size.0, size.1);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.write_header()?.write_image_data(rgba)?;
    Ok(())
}

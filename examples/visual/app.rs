use std::error::Error;
use std::fs::File;
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

use pixels::{Pixels, SurfaceTexture};
use winit::application::ApplicationHandler;
use winit::dpi::LogicalSize;
use winit::event::{ElementState, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop};
use winit::keyboard::{Key, NamedKey};
use winit::window::{Window, WindowAttributes, WindowId};

use crate::camera::ViewMode;
use crate::renderer::Renderer;
use crate::scene::Scene;
use crate::shared::scenarios::{ScenarioId, DT};

const WIDTH: u32 = 960;
const HEIGHT: u32 = 640;

pub fn run() -> Result<(), Box<dyn Error>> {
    let options = Options::parse()?;
    if let Some(directory) = options.capture_all {
        capture_all(&directory)?;
        return Ok(());
    }

    let event_loop = EventLoop::new()?;
    event_loop.set_control_flow(ControlFlow::Poll);
    let mut app = GalleryApp::new(options.scenario);
    event_loop.run_app(&mut app)?;
    Ok(())
}

struct Options {
    scenario: ScenarioId,
    capture_all: Option<PathBuf>,
}

impl Options {
    fn parse() -> Result<Self, Box<dyn Error>> {
        let mut scenario = ScenarioId::FlatWalk;
        let mut capture_all = None;
        let mut arguments = std::env::args().skip(1);
        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--scenario" => {
                    let value = arguments
                        .next()
                        .ok_or("--scenario requires a number from 1 to 9")?;
                    let number: usize = value.parse()?;
                    scenario =
                        ScenarioId::from_number(number).ok_or("scenario must be from 1 to 9")?;
                }
                "--capture-all" => {
                    let value = arguments
                        .next()
                        .ok_or("--capture-all requires an output directory")?;
                    capture_all = Some(PathBuf::from(value));
                }
                "--help" | "-h" => {
                    println!(
                        "walker2 visual gallery\n\n  --scenario N        open scenario 1..9\n  --capture-all DIR  write three deterministic PNG frames per scenario"
                    );
                    std::process::exit(0);
                }
                _ => return Err(format!("unknown argument: {argument}").into()),
            }
        }
        Ok(Self {
            scenario,
            capture_all,
        })
    }
}

struct GalleryApp {
    window: Option<Arc<Window>>,
    pixels: Option<Pixels<'static>>,
    scene: Scene,
    renderer: Renderer,
    view: ViewMode,
    zoom: f32,
    speed: f32,
    paused: bool,
    debug: bool,
    step_once: bool,
    accumulator: f32,
    last_frame: Instant,
}

impl GalleryApp {
    fn new(id: ScenarioId) -> Self {
        let scene = Scene::new(id);
        let view = scene.default_view();
        let zoom = scene.default_zoom();
        Self {
            window: None,
            pixels: None,
            scene,
            renderer: Renderer::new(),
            view,
            zoom,
            speed: 1.0,
            paused: false,
            debug: true,
            step_once: false,
            accumulator: 0.0,
            last_frame: Instant::now(),
        }
    }

    fn choose_scenario(&mut self, id: ScenarioId) {
        self.scene.set_scenario(id);
        self.view = self.scene.default_view();
        self.zoom = self.scene.default_zoom();
        self.accumulator = 0.0;
        self.last_frame = Instant::now();
        if let Some(window) = &self.window {
            window.set_title(&format!(
                "walker2 visual — {:02} {}",
                id.number(),
                id.title()
            ));
        }
    }

    fn update(&mut self) {
        let now = Instant::now();
        let elapsed = now.duration_since(self.last_frame).as_secs_f32().min(0.25);
        self.last_frame = now;
        if !self.paused {
            self.accumulator += elapsed * self.speed;
        }
        if self.step_once {
            self.accumulator += DT;
            self.step_once = false;
        }
        while self.accumulator >= DT {
            if self.scene.finished() {
                self.scene.reset();
            }
            self.scene.step();
            self.accumulator -= DT;
        }
    }

    fn handle_key(&mut self, event_loop: &ActiveEventLoop, key: &Key) {
        match key {
            Key::Named(NamedKey::Escape) => event_loop.exit(),
            Key::Named(NamedKey::Space) => {
                self.paused = !self.paused;
                self.last_frame = Instant::now();
            }
            Key::Named(NamedKey::Tab) => self.debug = !self.debug,
            Key::Named(NamedKey::ArrowLeft) => {
                let number = if self.scene.id.number() == 1 {
                    9
                } else {
                    self.scene.id.number() - 1
                };
                self.choose_scenario(ScenarioId::from_number(number).unwrap());
            }
            Key::Named(NamedKey::ArrowRight) => {
                let number = if self.scene.id.number() == 9 {
                    1
                } else {
                    self.scene.id.number() + 1
                };
                self.choose_scenario(ScenarioId::from_number(number).unwrap());
            }
            Key::Character(character) => {
                let value = character.to_lowercase();
                if let Ok(number) = value.parse::<usize>() {
                    if let Some(id) = ScenarioId::from_number(number) {
                        self.choose_scenario(id);
                    }
                    return;
                }
                match value.as_str() {
                    "r" => self.scene.reset(),
                    "v" => self.view = self.view.next(),
                    "." => {
                        self.paused = true;
                        self.step_once = true;
                    }
                    "+" | "=" => self.speed = (self.speed * 2.0).min(8.0),
                    "-" | "_" => self.speed = (self.speed * 0.5).max(0.125),
                    "[" => self.zoom = (self.zoom / 1.15).max(1.0),
                    "]" => self.zoom = (self.zoom * 1.15).min(100.0),
                    "p" => {
                        if let Err(error) = self.capture_current() {
                            eprintln!("capture failed: {error}");
                        }
                    }
                    _ => {}
                }
            }
            _ => {}
        }
    }

    fn capture_current(&mut self) -> Result<(), Box<dyn Error>> {
        let mut frame = vec![0; (WIDTH * HEIGHT * 4) as usize];
        self.renderer.render(
            &self.scene,
            &mut frame,
            WIDTH,
            HEIGHT,
            self.view,
            self.zoom,
            self.debug,
        );
        let path = PathBuf::from(format!(
            "target/visual/{:02}_{:05.2}.png",
            self.scene.id.number(),
            self.scene.time()
        ));
        write_png(&path, &frame, WIDTH, HEIGHT)?;
        println!("capture written: {}", path.display());
        Ok(())
    }
}

impl ApplicationHandler for GalleryApp {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }
        let attributes = WindowAttributes::default()
            .with_title(format!(
                "walker2 visual — {:02} {}",
                self.scene.id.number(),
                self.scene.id.title()
            ))
            .with_inner_size(LogicalSize::new(WIDTH as f64, HEIGHT as f64))
            .with_min_inner_size(LogicalSize::new(640.0, 420.0));
        let window = Arc::new(
            event_loop
                .create_window(attributes)
                .expect("create visual window"),
        );
        let size = window.inner_size();
        let surface = SurfaceTexture::new(size.width, size.height, Arc::clone(&window));
        self.pixels = Some(Pixels::new(WIDTH, HEIGHT, surface).expect("create pixel surface"));
        self.window = Some(window);
        self.last_frame = Instant::now();
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        window_id: WindowId,
        event: WindowEvent,
    ) {
        if self
            .window
            .as_ref()
            .is_none_or(|window| window.id() != window_id)
        {
            return;
        }
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) if size.width > 0 && size.height > 0 => {
                if let Some(pixels) = &mut self.pixels {
                    if let Err(error) = pixels.resize_surface(size.width, size.height) {
                        eprintln!("surface resize failed: {error}");
                        event_loop.exit();
                    }
                }
            }
            WindowEvent::KeyboardInput { event, .. }
                if event.state == ElementState::Pressed && !event.repeat =>
            {
                self.handle_key(event_loop, &event.logical_key);
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let amount = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y,
                    MouseScrollDelta::PixelDelta(position) => position.y as f32 / 40.0,
                };
                self.zoom = (self.zoom * 1.1_f32.powf(amount)).clamp(1.0, 100.0);
            }
            WindowEvent::RedrawRequested => {
                self.update();
                if let Some(pixels) = &mut self.pixels {
                    self.renderer.render(
                        &self.scene,
                        pixels.frame_mut(),
                        WIDTH,
                        HEIGHT,
                        self.view,
                        self.zoom,
                        self.debug,
                    );
                    if let Err(error) = pixels.render() {
                        eprintln!("pixel presentation failed: {error}");
                        event_loop.exit();
                    }
                }
            }
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}

fn capture_all(directory: &Path) -> Result<(), Box<dyn Error>> {
    std::fs::create_dir_all(directory)?;
    let mut renderer = Renderer::new();
    let mut frame = vec![0; (WIDTH * HEIGHT * 4) as usize];
    for id in ScenarioId::ALL {
        for (capture_index, numerator) in [1_u64, 2, 3].into_iter().enumerate() {
            let mut scene = Scene::new(id);
            let target_tick = scene.duration_ticks() * numerator / 4;
            while scene.tick < target_tick {
                scene.step();
            }
            let view = scene.default_view();
            let zoom = scene.default_zoom();
            renderer.render(&scene, &mut frame, WIDTH, HEIGHT, view, zoom, true);
            let path = directory.join(format!(
                "{:02}_{}_{}.png",
                id.number(),
                slug(id.title()),
                capture_index + 1
            ));
            write_png(&path, &frame, WIDTH, HEIGHT)?;
            println!("capture written: {}", path.display());
        }
    }
    Ok(())
}

fn write_png(path: &Path, frame: &[u8], width: u32, height: u32) -> Result<(), Box<dyn Error>> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let file = File::create(path)?;
    let mut encoder = png::Encoder::new(BufWriter::new(file), width, height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder.write_header()?.write_image_data(frame)?;
    Ok(())
}

fn slug(title: &str) -> String {
    title
        .chars()
        .map(|character| {
            if character.is_ascii_alphanumeric() {
                character.to_ascii_lowercase()
            } else {
                '_'
            }
        })
        .collect::<String>()
        .split('_')
        .filter(|part| !part.is_empty())
        .collect::<Vec<_>>()
        .join("_")
}

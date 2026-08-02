#[path = "../shared/mod.rs"]
mod shared;

mod app;
mod camera;
mod canvas;
mod renderer;
mod scene;

fn main() {
    if let Err(error) = app::run() {
        eprintln!("visual example failed: {error}");
        std::process::exit(1);
    }
}

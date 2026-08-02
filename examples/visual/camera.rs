use glam::{Vec2, Vec3};

use crate::scene::Scene;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViewMode {
    Iso,
    Side,
    Front,
    Top,
}

impl ViewMode {
    pub fn next(self) -> Self {
        match self {
            Self::Iso => Self::Side,
            Self::Side => Self::Front,
            Self::Front => Self::Top,
            Self::Top => Self::Iso,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Iso => "ISO",
            Self::Side => "SIDE",
            Self::Front => "FRONT",
            Self::Top => "TOP",
        }
    }
}

pub struct Camera {
    pub center: Vec3,
    pub pixels_per_unit: f32,
    pub view: ViewMode,
    width: f32,
    height: f32,
}

impl Camera {
    pub fn for_scene(scene: &Scene, width: u32, height: u32, view: ViewMode, zoom: f32) -> Self {
        let mut center = Vec3::ZERO;
        for actor in &scene.actors {
            center += actor.walker.position();
        }
        if !scene.actors.is_empty() {
            center /= scene.actors.len() as f32;
        }
        if matches!(view, ViewMode::Side | ViewMode::Front) {
            center.y *= 0.5;
        }
        Self {
            center,
            pixels_per_unit: zoom,
            view,
            width: width as f32,
            height: height as f32,
        }
    }

    pub fn project(&self, point: Vec3) -> (Vec2, f32) {
        let relative = point - self.center;
        let (plane, depth) = self.project_relative(relative);
        (
            Vec2::new(
                self.width * 0.5 + plane.x * self.pixels_per_unit,
                self.height * 0.54 - plane.y * self.pixels_per_unit,
            ),
            depth,
        )
    }

    pub fn project_vector(&self, vector: Vec3) -> Vec2 {
        let (plane, _) = self.project_relative(vector);
        Vec2::new(plane.x, -plane.y) * self.pixels_per_unit
    }

    fn project_relative(&self, point: Vec3) -> (Vec2, f32) {
        match self.view {
            ViewMode::Side => (Vec2::new(point.z, point.y), point.x),
            ViewMode::Front => (Vec2::new(point.x, point.y), -point.z),
            ViewMode::Top => (Vec2::new(point.x, point.z), point.y),
            ViewMode::Iso => {
                let x = (point.x - point.z) * 0.707_106_77;
                let y = point.y + (point.x + point.z) * 0.30;
                (Vec2::new(x, y), point.x + point.z + point.y * 0.1)
            }
        }
    }
}

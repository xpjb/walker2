use std::cmp::Ordering;

use glam::{Quat, Vec2, Vec3};
use walker2::{GroundQuery, MeshKey, PartPose, PartRole};

use crate::camera::{Camera, ViewMode};
use crate::canvas::{Canvas, Color};
use crate::scene::Scene;

const BACKGROUND: Color = [14, 18, 23, 255];
const GRID: Color = [42, 49, 57, 150];
const GROUND: Color = [102, 119, 100, 255];
const LEFT: Color = [232, 145, 61, 255];
const RIGHT: Color = [61, 201, 176, 255];

#[derive(Clone, Copy)]
enum Primitive {
    Capsule {
        depth: f32,
        a: Vec2,
        b: Vec2,
        radius: f32,
        color: Color,
    },
    RoundedBox {
        depth: f32,
        center: Vec2,
        axis: Vec2,
        half_extents: Vec2,
        radius: f32,
        color: Color,
    },
}

impl Primitive {
    fn depth(self) -> f32 {
        match self {
            Self::Capsule { depth, .. } | Self::RoundedBox { depth, .. } => depth,
        }
    }
}

pub struct Renderer {
    poses: Vec<PartPose>,
    primitives: Vec<Primitive>,
}

impl Renderer {
    pub fn new() -> Self {
        Self {
            poses: Vec::with_capacity(256),
            primitives: Vec::with_capacity(256),
        }
    }

    pub fn render(
        &mut self,
        scene: &Scene,
        frame: &mut [u8],
        width: u32,
        height: u32,
        view: ViewMode,
        zoom: f32,
        debug: bool,
    ) {
        let mut canvas = Canvas::new(frame, width, height);
        canvas.clear(BACKGROUND);
        let camera = Camera::for_scene(scene, width, height, view, zoom);

        self.draw_ground(&mut canvas, scene, &camera);
        self.collect_body_primitives(scene, &camera);
        self.primitives.sort_by(|left, right| {
            left.depth()
                .partial_cmp(&right.depth())
                .unwrap_or(Ordering::Equal)
        });
        for primitive in self.primitives.drain(..) {
            match primitive {
                Primitive::Capsule {
                    a,
                    b,
                    radius,
                    color,
                    ..
                } => {
                    canvas.capsule(a, b, radius, color);
                }
                Primitive::RoundedBox {
                    center,
                    axis,
                    half_extents,
                    radius,
                    color,
                    ..
                } => canvas.rounded_box(center, axis, half_extents, radius, color),
            }
        }

        self.draw_events(&mut canvas, scene, &camera);
        if debug && scene.actors.len() <= 20 {
            self.draw_debug(&mut canvas, scene, &camera);
        }
        self.draw_labels(&mut canvas, scene, &camera);
        self.draw_hud(&mut canvas, scene, view, debug);
    }

    fn collect_body_primitives(&mut self, scene: &Scene, camera: &Camera) {
        self.poses.clear();
        for actor in &scene.actors {
            let start = self.poses.len();
            actor.walker.part_poses(&mut self.poses);
            for pose in &self.poses[start..] {
                self.primitives.push(project_pose(*pose, camera));
            }
        }
    }

    fn draw_ground(&self, canvas: &mut Canvas<'_>, scene: &Scene, camera: &Camera) {
        let Some(actor) = scene.actors.first() else {
            return;
        };
        match camera.view {
            ViewMode::Side => {
                let span = canvas.width as f32 / camera.pixels_per_unit;
                let samples = (canvas.width / 5).max(2);
                let mut previous = None;
                for index in 0..samples {
                    let z =
                        camera.center.z - span * 0.5 + span * index as f32 / (samples - 1) as f32;
                    let point = Vec3::new(
                        actor.walker.position().x,
                        actor.ground.height_at(actor.walker.position().x, z),
                        z,
                    );
                    let projected = camera.project(point).0;
                    if let Some(last) = previous {
                        canvas.line(last, projected, 2.0, GROUND);
                    }
                    previous = Some(projected);
                }
            }
            ViewMode::Front => {
                let span = canvas.width as f32 / camera.pixels_per_unit;
                let samples = (canvas.width / 5).max(2);
                let mut previous = None;
                for index in 0..samples {
                    let x =
                        camera.center.x - span * 0.5 + span * index as f32 / (samples - 1) as f32;
                    let z = actor.walker.position().z;
                    let point = Vec3::new(x, actor.ground.height_at(x, z), z);
                    let projected = camera.project(point).0;
                    if let Some(last) = previous {
                        canvas.line(last, projected, 2.0, GROUND);
                    }
                    previous = Some(projected);
                }
            }
            ViewMode::Top | ViewMode::Iso => {
                let span =
                    (canvas.width.max(canvas.height) as f32 / camera.pixels_per_unit).max(20.0);
                let step = if span > 100.0 {
                    20.0
                } else if span > 45.0 {
                    10.0
                } else {
                    5.0
                };
                let x0 = ((camera.center.x - span) / step).floor() * step;
                let z0 = ((camera.center.z - span) / step).floor() * step;
                let count = (span * 2.0 / step).ceil() as usize;
                for index in 0..=count {
                    let x = x0 + index as f32 * step;
                    let a = camera
                        .project(Vec3::new(x, actor.ground.height_at(x, z0), z0))
                        .0;
                    let z1 = z0 + count as f32 * step;
                    let b = camera
                        .project(Vec3::new(x, actor.ground.height_at(x, z1), z1))
                        .0;
                    canvas.line(a, b, 1.0, GRID);

                    let z = z0 + index as f32 * step;
                    let a = camera
                        .project(Vec3::new(x0, actor.ground.height_at(x0, z), z))
                        .0;
                    let x1 = x0 + count as f32 * step;
                    let b = camera
                        .project(Vec3::new(x1, actor.ground.height_at(x1, z), z))
                        .0;
                    canvas.line(a, b, 1.0, GRID);
                }
            }
        }
    }

    fn draw_debug(&self, canvas: &mut Canvas<'_>, scene: &Scene, camera: &Camera) {
        for actor in &scene.actors {
            let signals = actor.walker.signals();
            let com = camera.project(signals.pos).0;
            let velocity = camera.project(signals.pos + signals.vel * 0.35).0;
            let capture = camera.project(signals.pos + signals.capture_error).0;
            let balance = camera.project(signals.pos + signals.balance).0;
            let facing = camera
                .project(signals.pos + Vec3::new(signals.yaw.sin(), 0.0, signals.yaw.cos()) * 1.5)
                .0;
            canvas.capsule(com, com, 4.0, [79, 143, 208, 255]);
            canvas.line(com, velocity, 2.0, [126, 201, 126, 230]);
            canvas.line(com, capture, 2.0, [235, 91, 91, 235]);
            canvas.line(com, balance, 2.0, [226, 194, 78, 235]);
            canvas.line(com, facing, 1.5, [220, 225, 230, 210]);

            let left = camera.project(signals.legs[0].foot).0;
            let right = camera.project(signals.legs[1].foot).0;
            canvas.line(left, right, 1.2, [164, 172, 180, 180]);
            for (leg, color) in signals.legs.iter().zip([LEFT, RIGHT]) {
                let foot = camera.project(leg.foot).0;
                let target = camera.project(leg.target).0;
                canvas.cross(target, 5.0, [color[0], color[1], color[2], 210]);
                if leg.contact {
                    canvas.capsule(foot, foot, 3.5, color);
                } else {
                    canvas.capsule(foot, foot, 2.0, [color[0], color[1], color[2], 150]);
                }
            }
        }
    }

    fn draw_events(&self, canvas: &mut Canvas<'_>, scene: &Scene, camera: &Camera) {
        for flash in &scene.footfalls {
            let point = camera.project(flash.pos).0;
            let alpha = (flash.remaining / 0.45).clamp(0.0, 1.0);
            let radius = 3.0 + flash.strength * 5.0 + (1.0 - alpha) * 8.0;
            canvas.capsule(point, point, radius, [228, 221, 178, (alpha * 180.0) as u8]);
        }
        for flash in &scene.impulses {
            let alpha = (flash.remaining / 0.9).clamp(0.0, 1.0);
            let origin = camera.project(flash.origin).0;
            let end = camera.project(flash.origin + flash.impulse * 0.8).0;
            canvas.line(origin, end, 4.0, [239, 92, 72, (alpha * 255.0) as u8]);
            canvas.capsule(end, end, 5.0, [239, 92, 72, (alpha * 255.0) as u8]);
        }
    }

    fn draw_labels(&self, canvas: &mut Canvas<'_>, scene: &Scene, camera: &Camera) {
        if scene.actors.len() > 12 {
            return;
        }
        for actor in &scene.actors {
            let point = actor.walker.position()
                + Vec3::Y * actor.walker.spec().body_height() * actor.walker.scale() * 0.8;
            let screen = camera.project(point).0;
            let width = actor.label.len() as f32 * 8.0;
            let label_x = (screen.x - width * 0.5).clamp(4.0, canvas.width as f32 - width - 4.0);
            let label_y = (screen.y - 12.0).clamp(76.0, canvas.height as f32 - 28.0);
            canvas.text(
                label_x as i32,
                label_y as i32,
                &actor.label,
                [205, 213, 220, 230],
                1,
            );
        }
    }

    fn draw_hud(&self, canvas: &mut Canvas<'_>, scene: &Scene, view: ViewMode, debug: bool) {
        canvas.fill_rect(
            Vec2::ZERO,
            Vec2::new(canvas.width as f32, 70.0),
            [7, 10, 13, 225],
        );
        let title = format!("{:02}  {}", scene.id.number(), scene.id.title()).to_uppercase();
        canvas.text(12, 9, &title, [230, 235, 240, 255], 1);
        canvas.text(
            12,
            25,
            scene.id.criterion().to_uppercase().as_str(),
            [165, 180, 192, 255],
            1,
        );
        let status = format!(
            "T {:05.2}/{:05.2}  PHASE: {}  VIEW: {}  DEBUG: {}",
            scene.time(),
            scene.id.duration(),
            scene.phase_label(),
            view.label(),
            if debug { "ON" } else { "OFF" },
        )
        .to_uppercase();
        canvas.text(12, 45, &status, [126, 201, 126, 255], 1);
        let controls =
            "1-9 SCENARIO  SPACE PAUSE  . STEP  R RESET  V VIEW  TAB DEBUG  +/- SPEED  [] ZOOM";
        canvas.text(
            12,
            canvas.height as i32 - 14,
            controls,
            [132, 143, 153, 235],
            1,
        );
    }
}

fn project_pose(pose: PartPose, camera: &Camera) -> Primitive {
    let translation = Vec3::from_array(pose.transform.translation);
    let rotation = Quat::from_array(pose.transform.rotation_xyzw);
    let scale = Vec3::from_array(pose.transform.scale).abs();
    let (center, depth) = camera.project(translation);
    let color = tint_to_color(pose.tint);
    let is_link = matches!(
        pose.role,
        PartRole::UpperLink | PartRole::MiddleLink | PartRole::LowerLink
    );

    if pose.mesh == MeshKey::JOINT {
        let radius = scale.max_element() * camera.pixels_per_unit * 0.5;
        return Primitive::Capsule {
            depth,
            a: center,
            b: center,
            radius,
            color,
        };
    }

    if pose.mesh == MeshKey::LIMB || is_link {
        let direction = rotation * Vec3::Y;
        let (a_world, b_world, radius_world) = if pose.mesh == MeshKey::LIMB {
            (
                translation,
                translation + direction * scale.y,
                scale.x.max(scale.z),
            )
        } else {
            (
                translation - direction * scale.y * 0.5,
                translation + direction * scale.y * 0.5,
                scale.x.min(scale.z) * 0.5,
            )
        };
        return Primitive::Capsule {
            depth,
            a: camera.project(a_world).0,
            b: camera.project(b_world).0,
            radius: radius_world * camera.pixels_per_unit,
            color,
        };
    }

    let mut axes = [
        camera.project_vector(rotation * Vec3::X * scale.x),
        camera.project_vector(rotation * Vec3::Y * scale.y),
        camera.project_vector(rotation * Vec3::Z * scale.z),
    ];
    axes.sort_by(|left, right| {
        right
            .length_squared()
            .partial_cmp(&left.length_squared())
            .unwrap_or(Ordering::Equal)
    });
    let major = axes[0];
    let minor = axes[1];
    let half_extents = Vec2::new(major.length() * 0.5, minor.length() * 0.5);
    Primitive::RoundedBox {
        depth,
        center,
        axis: major.normalize_or(Vec2::X),
        half_extents,
        radius: half_extents.min_element() * 0.22,
        color,
    }
}

fn tint_to_color(tint: [f32; 4]) -> Color {
    [
        (tint[0].clamp(0.0, 1.0) * 255.0) as u8,
        (tint[1].clamp(0.0, 1.0) * 255.0) as u8,
        (tint[2].clamp(0.0, 1.0) * 255.0) as u8,
        (tint[3].clamp(0.0, 1.0) * 255.0) as u8,
    ]
}

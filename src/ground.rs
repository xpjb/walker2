use glam::{vec3, Vec3};

/// Host-provided terrain query. The walker only ever samples heights (and
/// derived normals); it never mutates or raycasts the world. Implement
/// `height_at` and the rest comes free.
pub trait GroundQuery {
    fn height_at(&self, x: f32, z: f32) -> f32;

    fn normal_at(&self, x: f32, z: f32) -> Vec3 {
        let e = 0.5;
        let hx = self.height_at(x + e, z) - self.height_at(x - e, z);
        let hz = self.height_at(x, z + e) - self.height_at(x, z - e);
        vec3(-hx / (e * 2.0), 1.0, -hz / (e * 2.0)).normalize_or(Vec3::Y)
    }

    fn point_at(&self, x: f32, z: f32) -> Vec3 {
        vec3(x, self.height_at(x, z), z)
    }
}

/// Presents the world to the sim in CANONICAL units. A canonical walker
/// stepping on `ScaledGround` behaves exactly like a `scale`-times-bigger
/// walker on the real world: `world_point = scale * canonical_point` on
/// every axis, so `canonical_height(x, z) = world_height(scale*x, scale*z)
/// / scale`. This is the entire size knob: every gait table, IK pole and
/// epsilon in this crate is expressed in canonical units and stays
/// untouched — only positions crossing this boundary (and those returned
/// to callers) get multiplied through by `scale`. Combined with Froude
/// time scaling (`dt / sqrt(scale)` in `Walker::step`) this makes giants
/// genuinely ponderous and small walkers scurry, from one tuning.
pub struct ScaledGround<'a, G: GroundQuery> {
    pub inner: &'a G,
    pub scale: f32,
}

impl<G: GroundQuery> GroundQuery for ScaledGround<'_, G> {
    fn height_at(&self, x: f32, z: f32) -> f32 {
        self.inner.height_at(x * self.scale, z * self.scale) / self.scale
    }
}

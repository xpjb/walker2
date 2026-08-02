use glam::Vec3;

#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Material {
    MarineArmor = 0,
    MarineCloth = 1,
    Skin = 2,
    Leather = 3,
    Metal = 4,
    OgreSkin = 5,
    OgreBelly = 6,
    Ground = 7,
}

#[derive(Clone, Copy, Debug)]
pub struct Plane {
    /// Unit outward normal. The brush interior satisfies `normal.dot(p) <= distance`.
    pub normal: Vec3,
    pub distance: f32,
}

impl Plane {
    pub fn new(normal: Vec3, distance: f32) -> Self {
        let length = normal.length().max(1.0e-6);
        Self {
            normal: normal / length,
            distance: distance / length,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Brush {
    pub planes: Vec<Plane>,
    pub material: Material,
    pub texel_scale: f32,
}

impl Brush {
    pub fn cuboid(center: Vec3, half_extents: Vec3, material: Material) -> Self {
        let half = half_extents.max(Vec3::splat(0.001));
        Self {
            planes: vec![
                Plane::new(Vec3::X, center.x + half.x),
                Plane::new(Vec3::NEG_X, -center.x + half.x),
                Plane::new(Vec3::Y, center.y + half.y),
                Plane::new(Vec3::NEG_Y, -center.y + half.y),
                Plane::new(Vec3::Z, center.z + half.z),
                Plane::new(Vec3::NEG_Z, -center.z + half.z),
            ],
            material,
            texel_scale: 8.0,
        }
    }

    /// A box with corner-cut planes. This keeps the planar, convex brush
    /// construction while avoiding a perfectly modern cuboid silhouette.
    pub fn beveled_box(center: Vec3, half_extents: Vec3, bevel: f32, material: Material) -> Self {
        let mut brush = Self::cuboid(center, half_extents, material);
        let bevel = bevel.max(0.0);
        if bevel > 0.0 {
            for x in [-1.0, 1.0] {
                for y in [-1.0, 1.0] {
                    for z in [-1.0, 1.0] {
                        let normal = Vec3::new(x, y, z).normalize();
                        let support = normal.dot(center) + normal.abs().dot(half_extents)
                            - bevel.min(half_extents.min_element() * 0.8);
                        brush.planes.push(Plane::new(normal, support));
                    }
                }
            }
        }
        brush
    }

    /// Convex planar approximation of an ellipsoid. `slices` controls the
    /// intentionally low-poly silhouette; eight is a useful Quake-like default.
    pub fn ellipsoid(center: Vec3, radii: Vec3, slices: usize, material: Material) -> Self {
        let radii = radii.max(Vec3::splat(0.001));
        let slices = slices.max(4);
        let mut planes = Vec::with_capacity(slices * 5 + 2);
        planes.push(Plane::new(Vec3::Y, center.y + radii.y));
        planes.push(Plane::new(Vec3::NEG_Y, -center.y + radii.y));
        for elevation_degrees in [-60.0_f32, -30.0, 0.0, 30.0, 60.0] {
            let elevation = elevation_degrees.to_radians();
            for slice in 0..slices {
                let yaw = std::f32::consts::TAU * slice as f32 / slices as f32;
                let normal = Vec3::new(
                    elevation.cos() * yaw.cos(),
                    elevation.sin(),
                    elevation.cos() * yaw.sin(),
                );
                let support_radius =
                    Vec3::new(normal.x * radii.x, normal.y * radii.y, normal.z * radii.z).length();
                planes.push(Plane::new(normal, normal.dot(center) + support_radius));
            }
        }
        Self {
            planes,
            material,
            texel_scale: 8.0,
        }
    }

    pub fn with_texel_scale(mut self, texel_scale: f32) -> Self {
        self.texel_scale = texel_scale.max(0.01);
        self
    }
}

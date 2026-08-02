use glam::{Mat3, Mat4, Vec2, Vec3};

use crate::brush::{Brush, Material, Plane};

#[derive(Clone, Copy, Debug)]
pub struct Vertex {
    pub position: Vec3,
    pub normal: Vec3,
    pub uv: Vec2,
    pub material: Material,
}

#[derive(Clone, Debug, Default)]
pub struct Mesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}

impl Mesh {
    pub fn append(&mut self, other: &Self) {
        let base = self.vertices.len() as u32;
        self.vertices.extend_from_slice(&other.vertices);
        self.indices
            .extend(other.indices.iter().map(|index| base + index));
    }

    pub fn append_transformed(&mut self, other: &Self, transform: Mat4) {
        let base = self.vertices.len() as u32;
        let normal_matrix = Mat3::from_mat4(transform).inverse().transpose();
        self.vertices
            .extend(other.vertices.iter().map(|vertex| Vertex {
                position: transform.transform_point3(vertex.position),
                normal: (normal_matrix * vertex.normal).normalize_or(vertex.normal),
                uv: vertex.uv,
                material: vertex.material,
            }));
        self.indices
            .extend(other.indices.iter().map(|index| base + index));
    }
}

pub fn compile_brush(brush: &Brush) -> Mesh {
    let mut mesh = Mesh::default();
    for (face_index, face) in brush.planes.iter().enumerate() {
        let (u, v) = plane_basis(face.normal);
        let center = face.normal * face.distance;
        let extent = 1024.0_f32;
        let mut polygon = vec![
            center - u * extent - v * extent,
            center + u * extent - v * extent,
            center + u * extent + v * extent,
            center - u * extent + v * extent,
        ];
        for (clip_index, clip_plane) in brush.planes.iter().enumerate() {
            if clip_index == face_index {
                continue;
            }
            polygon = clip_polygon(&polygon, *clip_plane);
            if polygon.len() < 3 {
                break;
            }
        }
        deduplicate_polygon(&mut polygon);
        if polygon.len() < 3 {
            continue;
        }

        let base = mesh.vertices.len() as u32;
        for point in &polygon {
            mesh.vertices.push(Vertex {
                position: *point,
                normal: face.normal,
                uv: Vec2::new(point.dot(u), point.dot(v)) * brush.texel_scale,
                material: brush.material,
            });
        }
        for triangle in 1..polygon.len() - 1 {
            mesh.indices.extend_from_slice(&[
                base,
                base + triangle as u32,
                base + triangle as u32 + 1,
            ]);
        }
    }
    mesh
}

fn plane_basis(normal: Vec3) -> (Vec3, Vec3) {
    let reference = if normal.y.abs() < 0.9 {
        Vec3::Y
    } else {
        Vec3::X
    };
    let u = reference.cross(normal).normalize_or(Vec3::X);
    let v = normal.cross(u).normalize_or(Vec3::Z);
    (u, v)
}

fn clip_polygon(polygon: &[Vec3], plane: Plane) -> Vec<Vec3> {
    const EPSILON: f32 = 1.0e-4;
    if polygon.is_empty() {
        return Vec::new();
    }
    let mut output = Vec::with_capacity(polygon.len() + 1);
    let mut previous = polygon[polygon.len() - 1];
    let mut previous_distance = plane.normal.dot(previous) - plane.distance;
    let mut previous_inside = previous_distance <= EPSILON;
    for &current in polygon {
        let current_distance = plane.normal.dot(current) - plane.distance;
        let current_inside = current_distance <= EPSILON;
        if current_inside != previous_inside {
            let denominator = previous_distance - current_distance;
            if denominator.abs() > 1.0e-7 {
                let t = (previous_distance / denominator).clamp(0.0, 1.0);
                output.push(previous.lerp(current, t));
            }
        }
        if current_inside {
            output.push(current);
        }
        previous = current;
        previous_distance = current_distance;
        previous_inside = current_inside;
    }
    output
}

fn deduplicate_polygon(polygon: &mut Vec<Vec3>) {
    const EPSILON_SQ: f32 = 1.0e-8;
    let mut index = 0;
    while polygon.len() >= 2 && index < polygon.len() {
        let next = (index + 1) % polygon.len();
        if polygon[index].distance_squared(polygon[next]) <= EPSILON_SQ {
            polygon.remove(next);
            if next == 0 {
                index = 0;
            }
        } else {
            index += 1;
        }
    }
}

//! Procedural low-poly characters assembled from convex brushes.
//!
//! Brushes compile once into bone-local meshes. A locomotion adapter then
//! supplies rigid bone transforms; geometry is never rebuilt per frame.

pub mod brush;
pub mod mesh;
pub mod model;
pub mod pose;

pub use brush::{Brush, Material, Plane};
pub use mesh::{compile_brush, Mesh, Vertex};
pub use model::{ActorModel, Bone, Morphology, Part};
pub use pose::{pose_walker, PoseDriver, PosePalette};

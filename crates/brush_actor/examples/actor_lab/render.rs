use brush_actor::{ActorModel, Material, Mesh, PosePalette, Vertex};
use bytemuck::{Pod, Zeroable};
use chad::wgpu;
use glam::{Mat4, Vec2, Vec3};

const MAX_VERTICES: usize = 65_536;
const MAX_INDICES: usize = 196_608;
const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth24Plus;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct GpuVertex {
    position: [f32; 3],
    normal: [f32; 3],
    uv: [f32; 2],
    material: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Uniforms {
    view_projection: [[f32; 4]; 4],
    camera_position: [f32; 4],
    light_direction: [f32; 4],
}

pub struct Camera {
    pub eye: Vec3,
    pub target: Vec3,
}

pub struct GpuRenderer {
    pipeline: wgpu::RenderPipeline,
    bind_group: wgpu::BindGroup,
    uniform: wgpu::Buffer,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    depth: wgpu::Texture,
    depth_view: wgpu::TextureView,
    size: (u32, u32),
    mesh: Mesh,
    gpu_vertices: Vec<GpuVertex>,
}

impl GpuRenderer {
    pub fn new(device: &wgpu::Device, format: wgpu::TextureFormat, size: (u32, u32)) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("brush-actor-shader"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });
        let uniform = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("brush-actor-uniform"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let vertex_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("brush-actor-vertices"),
            size: (MAX_VERTICES * std::mem::size_of::<GpuVertex>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let index_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("brush-actor-indices"),
            size: (MAX_INDICES * std::mem::size_of::<u32>()) as u64,
            usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("brush-actor-bind-group-layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("brush-actor-bind-group"),
            layout: &bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform.as_entire_binding(),
            }],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("brush-actor-pipeline-layout"),
            bind_group_layouts: &[Some(&bind_group_layout)],
            immediate_size: 0,
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("brush-actor-pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<GpuVertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![
                        0 => Float32x3,
                        1 => Float32x3,
                        2 => Float32x2,
                        3 => Uint32
                    ],
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                unclipped_depth: false,
                polygon_mode: wgpu::PolygonMode::Fill,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::Less),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });
        let (depth, depth_view) = create_depth(device, size);
        Self {
            pipeline,
            bind_group,
            uniform,
            vertex_buffer,
            index_buffer,
            depth,
            depth_view,
            size,
            mesh: Mesh::default(),
            gpu_vertices: Vec::with_capacity(MAX_VERTICES),
        }
    }

    pub fn render(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        target: &wgpu::TextureView,
        size: (u32, u32),
        actors: &[(&ActorModel, &PosePalette)],
        camera: Camera,
    ) {
        if self.size != size {
            let (depth, depth_view) = create_depth(device, size);
            self.depth = depth;
            self.depth_view = depth_view;
            self.size = size;
        }

        self.mesh.vertices.clear();
        self.mesh.indices.clear();
        append_ground(&mut self.mesh, camera.target);
        for (model, pose) in actors {
            model.append_pose_mesh(&mut self.mesh, &pose.transforms);
        }
        assert!(
            self.mesh.vertices.len() <= MAX_VERTICES,
            "brush actor vertex capacity exceeded"
        );
        assert!(
            self.mesh.indices.len() <= MAX_INDICES,
            "brush actor index capacity exceeded"
        );

        self.gpu_vertices.clear();
        self.gpu_vertices
            .extend(self.mesh.vertices.iter().map(|vertex| GpuVertex {
                position: vertex.position.to_array(),
                normal: vertex.normal.to_array(),
                uv: vertex.uv.to_array(),
                material: vertex.material as u32,
            }));
        queue.write_buffer(
            &self.vertex_buffer,
            0,
            bytemuck::cast_slice(&self.gpu_vertices),
        );
        queue.write_buffer(
            &self.index_buffer,
            0,
            bytemuck::cast_slice(&self.mesh.indices),
        );

        let aspect = size.0.max(1) as f32 / size.1.max(1) as f32;
        let projection = Mat4::perspective_rh(58.0_f32.to_radians(), aspect, 0.08, 160.0);
        let view = Mat4::look_at_rh(camera.eye, camera.target, Vec3::Y);
        let uniforms = Uniforms {
            view_projection: (projection * view).to_cols_array_2d(),
            camera_position: camera.eye.extend(1.0).to_array(),
            light_direction: Vec3::new(-0.48, -0.83, -0.28)
                .normalize()
                .extend(0.0)
                .to_array(),
        };
        queue.write_buffer(&self.uniform, 0, bytemuck::bytes_of(&uniforms));

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("brush-actor-frame"),
        });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("brush-actor-pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: target,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.018,
                            g: 0.022,
                            b: 0.028,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &self.bind_group, &[]);
            pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
            pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..self.mesh.indices.len() as u32, 0, 0..1);
        }
        queue.submit(std::iter::once(encoder.finish()));
    }
}

fn create_depth(device: &wgpu::Device, size: (u32, u32)) -> (wgpu::Texture, wgpu::TextureView) {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("brush-actor-depth"),
        size: wgpu::Extent3d {
            width: size.0.max(1),
            height: size.1.max(1),
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: DEPTH_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    (texture, view)
}

fn append_ground(mesh: &mut Mesh, center: Vec3) {
    let span = 45.0;
    let y = 0.0;
    let x0 = center.x - span;
    let x1 = center.x + span;
    let z0 = center.z - span;
    let z1 = center.z + span;
    let base = mesh.vertices.len() as u32;
    mesh.vertices.extend([
        Vertex {
            position: Vec3::new(x0, y, z0),
            normal: Vec3::Y,
            uv: Vec2::new(x0, z0) * 2.0,
            material: Material::Ground,
        },
        Vertex {
            position: Vec3::new(x0, y, z1),
            normal: Vec3::Y,
            uv: Vec2::new(x0, z1) * 2.0,
            material: Material::Ground,
        },
        Vertex {
            position: Vec3::new(x1, y, z1),
            normal: Vec3::Y,
            uv: Vec2::new(x1, z1) * 2.0,
            material: Material::Ground,
        },
        Vertex {
            position: Vec3::new(x1, y, z0),
            normal: Vec3::Y,
            uv: Vec2::new(x1, z0) * 2.0,
            material: Material::Ground,
        },
    ]);
    mesh.indices
        .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
}

const SHADER: &str = r#"
struct Uniforms {
    view_projection: mat4x4<f32>,
    camera_position: vec4<f32>,
    light_direction: vec4<f32>,
};
@group(0) @binding(0) var<uniform> uniforms: Uniforms;

struct VertexIn {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) material: u32,
};

struct VertexOut {
    @builtin(position) position: vec4<f32>,
    @location(0) world_position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) @interpolate(flat) material: u32,
};

@vertex
fn vs_main(input: VertexIn) -> VertexOut {
    var output: VertexOut;
    output.position = uniforms.view_projection * vec4<f32>(input.position, 1.0);
    output.world_position = input.position;
    output.normal = input.normal;
    output.uv = input.uv;
    output.material = input.material;
    return output;
}

fn hash_cell(cell: vec2<f32>, material: u32) -> f32 {
    let value = dot(cell, vec2<f32>(12.9898, 78.233)) + f32(material) * 31.17;
    return fract(sin(value) * 43758.5453);
}

fn palette(material: u32) -> vec3<f32> {
    switch material {
        case 0u: { return vec3<f32>(0.22, 0.34, 0.25); }
        case 1u: { return vec3<f32>(0.19, 0.20, 0.17); }
        case 2u: { return vec3<f32>(0.57, 0.38, 0.29); }
        case 3u: { return vec3<f32>(0.19, 0.095, 0.045); }
        case 4u: { return vec3<f32>(0.23, 0.28, 0.29); }
        case 5u: { return vec3<f32>(0.38, 0.43, 0.20); }
        case 6u: { return vec3<f32>(0.47, 0.36, 0.18); }
        default: { return vec3<f32>(0.095, 0.105, 0.095); }
    }
}

@fragment
fn fs_main(input: VertexOut) -> @location(0) vec4<f32> {
    let cell = floor(input.uv);
    let noise = hash_cell(cell, input.material);
    var base = palette(input.material);
    if input.material == 0u {
        let panel = select(0.88, 1.08, (i32(cell.x + cell.y) & 3) == 0);
        base *= panel;
    } else if input.material == 3u {
        let strap = select(0.82, 1.10, (i32(cell.x) & 3) == 0);
        base *= strap;
    } else if input.material == 5u || input.material == 6u {
        base *= 0.84 + noise * 0.28;
    } else if input.material == 7u {
        let checker = (i32(floor(input.uv.x * 0.25)) + i32(floor(input.uv.y * 0.25))) & 1;
        base *= select(0.72, 1.08, checker == 0);
    } else {
        base *= 0.88 + noise * 0.20;
    }

    let normal = normalize(input.normal);
    let diffuse = max(dot(normal, -uniforms.light_direction.xyz), 0.0);
    var light = 0.30 + diffuse * 0.78;
    light = floor(light * 5.0 + 0.5) / 5.0;
    let distance_to_camera = distance(input.world_position, uniforms.camera_position.xyz);
    let fog = clamp((distance_to_camera - 18.0) / 70.0, 0.0, 0.72);
    let lit = base * light;
    let fog_color = vec3<f32>(0.018, 0.022, 0.028);
    return vec4<f32>(mix(lit, fog_color, fog), 1.0);
}
"#;

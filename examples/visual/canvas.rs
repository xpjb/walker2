use font8x8::{UnicodeFonts, BASIC_FONTS};
use glam::Vec2;

pub type Color = [u8; 4];

pub struct Canvas<'a> {
    pixels: &'a mut [u8],
    pub width: u32,
    pub height: u32,
}

impl<'a> Canvas<'a> {
    pub fn new(pixels: &'a mut [u8], width: u32, height: u32) -> Self {
        Self {
            pixels,
            width,
            height,
        }
    }

    pub fn clear(&mut self, color: Color) {
        for pixel in self.pixels.chunks_exact_mut(4) {
            pixel.copy_from_slice(&color);
        }
    }

    pub fn fill_rect(&mut self, min: Vec2, max: Vec2, color: Color) {
        let x0 = min.x.floor().max(0.0) as i32;
        let y0 = min.y.floor().max(0.0) as i32;
        let x1 = max.x.ceil().min(self.width as f32) as i32;
        let y1 = max.y.ceil().min(self.height as f32) as i32;
        for y in y0..y1 {
            for x in x0..x1 {
                self.blend_pixel(x, y, color, 1.0);
            }
        }
    }

    pub fn capsule(&mut self, a: Vec2, b: Vec2, radius: f32, color: Color) {
        let radius = radius.max(0.75);
        let min = a.min(b) - Vec2::splat(radius + 1.0);
        let max = a.max(b) + Vec2::splat(radius + 1.0);
        self.raster_sdf(min, max, color, |point| {
            let pa = point - a;
            let ba = b - a;
            let denominator = ba.length_squared();
            let h = if denominator > 1.0e-5 {
                (pa.dot(ba) / denominator).clamp(0.0, 1.0)
            } else {
                0.0
            };
            (pa - ba * h).length() - radius
        });
    }

    pub fn rounded_box(
        &mut self,
        center: Vec2,
        axis: Vec2,
        half_extents: Vec2,
        radius: f32,
        color: Color,
    ) {
        let axis_x = axis.normalize_or(Vec2::X);
        let axis_y = Vec2::new(-axis_x.y, axis_x.x);
        let half_extents = half_extents.max(Vec2::splat(0.75));
        let bound = Vec2::splat(half_extents.length() + 1.0);
        let radius = radius.clamp(0.0, half_extents.min_element());
        self.raster_sdf(center - bound, center + bound, color, |point| {
            let relative = point - center;
            let local = Vec2::new(relative.dot(axis_x), relative.dot(axis_y));
            let q = local.abs() - (half_extents - Vec2::splat(radius));
            q.max(Vec2::ZERO).length() + q.max_element().min(0.0) - radius
        });
    }

    pub fn line(&mut self, a: Vec2, b: Vec2, width: f32, color: Color) {
        self.capsule(a, b, width * 0.5, color);
    }

    pub fn cross(&mut self, center: Vec2, radius: f32, color: Color) {
        self.line(
            center - Vec2::new(radius, 0.0),
            center + Vec2::new(radius, 0.0),
            1.5,
            color,
        );
        self.line(
            center - Vec2::new(0.0, radius),
            center + Vec2::new(0.0, radius),
            1.5,
            color,
        );
    }

    pub fn text(&mut self, mut x: i32, y: i32, text: &str, color: Color, scale: i32) {
        let start_x = x;
        for character in text.chars() {
            if character == '\n' {
                x = start_x;
                continue;
            }
            if let Some(glyph) = BASIC_FONTS.get(character) {
                for (row, bits) in glyph.into_iter().enumerate() {
                    for column in 0..8 {
                        if bits & (1 << column) != 0 {
                            for sy in 0..scale {
                                for sx in 0..scale {
                                    self.blend_pixel(
                                        x + column * scale + sx,
                                        y + row as i32 * scale + sy,
                                        color,
                                        1.0,
                                    );
                                }
                            }
                        }
                    }
                }
            }
            x += 8 * scale;
        }
    }

    fn raster_sdf(
        &mut self,
        min: Vec2,
        max: Vec2,
        color: Color,
        mut distance: impl FnMut(Vec2) -> f32,
    ) {
        let x0 = min.x.floor().max(0.0) as i32;
        let y0 = min.y.floor().max(0.0) as i32;
        let x1 = max.x.ceil().min(self.width as f32 - 1.0) as i32;
        let y1 = max.y.ceil().min(self.height as f32 - 1.0) as i32;
        for y in y0..=y1 {
            for x in x0..=x1 {
                let d = distance(Vec2::new(x as f32 + 0.5, y as f32 + 0.5));
                let coverage = (0.5 - d).clamp(0.0, 1.0);
                if coverage > 0.0 {
                    self.blend_pixel(x, y, color, coverage);
                }
            }
        }
    }

    fn blend_pixel(&mut self, x: i32, y: i32, color: Color, coverage: f32) {
        if x < 0 || y < 0 || x >= self.width as i32 || y >= self.height as i32 {
            return;
        }
        let index = (y as usize * self.width as usize + x as usize) * 4;
        let alpha = coverage * color[3] as f32 / 255.0;
        let inverse = 1.0 - alpha;
        self.pixels[index] = (color[0] as f32 * alpha + self.pixels[index] as f32 * inverse) as u8;
        self.pixels[index + 1] =
            (color[1] as f32 * alpha + self.pixels[index + 1] as f32 * inverse) as u8;
        self.pixels[index + 2] =
            (color[2] as f32 * alpha + self.pixels[index + 2] as f32 * inverse) as u8;
        self.pixels[index + 3] = 255;
    }
}

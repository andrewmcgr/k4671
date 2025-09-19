use core::f32::consts::PI;
use num_traits::real::Real;

pub struct Biquad {
    pub b0: f32,
    pub b1: f32,
    pub a0: f32,
    pub b2: f32,
    pub a1: f32,
    pub a2: f32,
}

pub struct BiquadTmc {
    pub a1: u32,
    pub a2: u32,
    pub b0: u32,
    pub b1: u32,
    pub b2: u32,
}

impl Biquad {
    pub fn new(b0: f32, b1: f32, b2: f32, a0: f32, a1: f32, a2: f32) -> Self {
        Self {
            b0,
            b1,
            b2,
            a0,
            a1,
            a2,
        }
    }

    pub fn to_tmc(&self) -> BiquadTmc {
        let b0 = (self.b0 / self.a0 * (1 << 29) as f32).round() as u32;
        let b1 = (self.b1 / self.a0 * (1 << 29) as f32).round() as u32;
        let b2 = (self.b2 / self.a0 * (1 << 29) as f32).round() as u32;
        let a1 = ((-self.a1) / self.a0 * (1 << 29) as f32).round() as u32;
        let a2 = ((-self.a2) / self.a0 * (1 << 29) as f32).round() as u32;

        BiquadTmc { a1, a2, b0, b1, b2 }
    }
}

pub fn biquad_lpf(fc: f32, fs: f32) -> Biquad {
    let w0 = 2.0 * PI * fc / fs;
    let alpha = w0.sin() / (2.0 * (2.0f32).sqrt());
    let cos_w0 = w0.cos();

    let b0 = (1.0 - cos_w0) / 2.0;
    let b1 = 1.0 - cos_w0;
    let b2 = (1.0 - cos_w0) / 2.0;
    let a0 = 1.0 + alpha;
    let a1 = -2.0 * cos_w0;
    let a2 = 1.0 - alpha;

    Biquad::new(b0, b1, b2, a0, a1, a2)
}

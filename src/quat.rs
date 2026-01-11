use crate::{EPSILON_SQ, vec3::Vec3};
use wasm_bindgen::prelude::*;
pub struct SinCos {
    sin: f32,
    cos: f32
}
pub fn get_sin_cos(angle: f32)->SinCos{
    SinCos { sin: f32::sin(angle), cos: f32::cos(angle) }
}
#[wasm_bindgen]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}
#[wasm_bindgen]
impl Quat {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Quat { x, y, z, w }
    }
    pub fn to_euler_angles(&self) -> Vec3 {
        let x = self.x;
        let y = self.y;
        let z = self.z;
        let w = self.w;
        let sin_x: f32 = 2.0 * (self.y * self.z - self.w * self.x);
        let clamped_sin_x = f32::max(-1.0, f32::min(1.0, sin_x));
        let rot_x = f32::asin(clamped_sin_x);
        let rot_y: f32;
        let rot_z: f32;
        if f32::abs(clamped_sin_x) > 0.999 {
            rot_z = 0.0;
            rot_y = f32::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (x * x + y * y));
        } else {
            rot_y = f32::atan2(-(2.0 * x * z + 2.0 * w * y), 1.0 - 2.0 * (x * x + y * y));
            rot_z = f32::atan2(2.0 * x * y + 2.0 * w * z, 1.0 - 2.0 * (x * x + z * z));
        }
        let mut result = Vec3::new(0.0, 0.0, 0.0);
        result.set(rot_x, rot_y, rot_z);
        result
    }
    pub fn identity() -> Quat {
        Quat {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }
    }
    pub fn slerp(a: &Quat, b: &Quat, t: f32) -> Quat {
        let mut cos = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
        let mut bx = b.x;
        let mut by = b.y;
        let mut bz = b.z;
        let mut bw = b.w;

        if cos < 0.0 {
            cos = -cos;
            bx = -bx;
            by = -by;
            bz = -bz;
            bw = -bw;
        }

        if cos > 0.9995 {
            let x = a.x + t * (bx - a.x);
            let y = a.y + t * (by - a.y);
            let z = a.z + t * (bz - a.z);
            let w = a.w + t * (bw - a.w);
            let inv_len = 1.0 / (x * x + y * y + z * z + w * w).sqrt();
            return Quat::new(x * inv_len, y * inv_len, z * inv_len, w * inv_len);
        }

        let theta0 = cos.acos();
        let sin_theta0 = theta0.sin();
        let theta = theta0 * t;
        let sin_theta = theta.sin();
        let s0 = (theta0 - theta).sin() / sin_theta0;
        let s1 = sin_theta / sin_theta0;

        Quat::new(
            s0 * a.x + s1 * bx,
            s0 * a.y + s1 * by,
            s0 * a.z + s1 * bz,
            s0 * a.w + s1 * bw,
        )
    }

    pub fn from_unit_vectors_to_ref(
        vec_from: &Vec3,
        vec_to: &Vec3,
        result: &mut Quat,
        epsilon: f32,
    ) -> Quat {
        let from_x = vec_from.x;
        let from_y = vec_from.y;
        let from_z = vec_from.z;
        let to_x = vec_to.x;
        let to_y = vec_to.y;
        let to_z = vec_to.z;
        let dot = from_x * to_x + from_y * to_y + from_z * to_z;

        if dot > 1.0 - epsilon {
            result.x = 0.0;
            result.y = 0.0;
            result.z = 0.0;
            result.w = 1.0;
            return result.clone();
        }

        if dot < -1.0 + epsilon {
            let mut axis_x = from_y;
            let mut axis_y = -from_x;
            let mut axis_z = 0.0;
            let len_sq = axis_x * axis_x + axis_y * axis_y;

            if len_sq < epsilon {
                axis_x = 0.0;
                axis_y = from_z;
                axis_z = -from_y;
            }

            let len = (axis_x * axis_x + axis_y * axis_y + axis_z * axis_z).sqrt();
            if len > epsilon {
                let inv_len = 1.0 / len;
                result.x = axis_x * inv_len;
                result.y = axis_y * inv_len;
                result.z = axis_z * inv_len;
                result.w = 0.0;
            } else {
                result.x = 1.0;
                result.y = 0.0;
                result.z = 0.0;
                result.w = 0.0;
            }
            return result.clone();
        }

        let cross_x = from_y * to_z - from_z * to_y;
        let cross_y = from_z * to_x - from_x * to_z;
        let cross_z = from_x * to_y - from_y * to_x;
        let w = ((1.0 + dot) * 2.0).sqrt();
        let inv_w = 1.0 / w;

        result.x = cross_x * inv_w;
        result.y = cross_y * inv_w;
        result.z = cross_z * inv_w;
        result.w = w * 0.5;

        let x = result.x;
        let y = result.y;
        let z = result.z;
        let w_val = result.w;
        let len = (x * x + y * y + z * z + w_val * w_val).sqrt();
        if len > epsilon {
            let inv_len = 1.0 / len;
            result.x = x * inv_len;
            result.y = y * inv_len;
            result.z = z * inv_len;
            result.w = w_val * inv_len;
        } else {
            result.x = 0.0;
            result.y = 0.0;
            result.z = 0.0;
            result.w = 1.0;
        }
        result.clone()
    }

    pub fn from_unit_vectors(vec_from: &Vec3, vec_to: &Vec3, epsilon: f32) -> Quat {
        let mut result = Quat::identity();
        Self::from_unit_vectors_to_ref(vec_from, vec_to, &mut result, epsilon);
        result
    }

    pub fn rotation_yaw_pitch_roll(yaw: f32, pitch: f32, roll: f32) -> Quat {
        let mut result = Quat::identity();
        let half_yaw = yaw * 0.5;
        let half_pitch = pitch * 0.5;
        let half_roll = roll * 0.5;

        let cy = half_yaw.cos();
        let sy = half_yaw.sin();
        let cp = half_pitch.cos();
        let sp = half_pitch.sin();
        let cr = half_roll.cos();
        let sr = half_roll.sin();

        result.x = sp * cy * cr - cp * sy * sr;
        result.y = cp * sy * cr + sp * cy * sr;
        result.z = cp * cy * sr - sp * sy * cr;
        result.w = cp * cy * cr + sp * sy * sr;

        let x = result.x;
        let y = result.y;
        let z = result.z;
        let w = result.w;
        let len = (x * x + y * y + z * z + w * w).sqrt();
        if len > 1e-8 {
            let inv_len = 1.0 / len;
            result.x = x * inv_len;
            result.y = y * inv_len;
            result.z = z * inv_len;
            result.w = w * inv_len;
        } else {
            result.x = 0.0;
            result.y = 0.0;
            result.z = 0.0;
            result.w = 1.0;
        }
        result
    }

    pub fn from_euler(rot_x: f32, rot_y: f32, rot_z: f32) -> Quat {
        let half_x = rot_x * 0.5;
        let half_y = rot_y * 0.5;
        let half_z = rot_z * 0.5;

        let scx = get_sin_cos(half_x);
        let scy = get_sin_cos(half_y);
        let scz = get_sin_cos(half_z);

        let sx = scx.sin;
        let cx = scx.cos;
        let sy = scy.sin;
        let cy = scy.cos;
        let sz = scz.sin;
        let cz = scz.cos;

        let w = cy * cx * cz + sy * sx * sz;
        let x = cy * sx * cz + sy * cx * sz;
        let y = sy * cx * cz - cy * sx * sz;
        let z = cy * cx * sz - sy * sx * cz;

        let q = Quat::new(x, y, z, w);
        q.normalize()
    }
    pub fn normalize(mut self) -> Self {
        let x = &self.x;
        let y = &self.y;
        let z = &self.z;
        let w = &self.w;
        let len_sq = x * x + y * y + z * z + w * w;

        if len_sq > EPSILON_SQ {
            let inv_len = 1.0 / f32::sqrt(len_sq);
            self.x = x * inv_len;
            self.y = y * inv_len;
            self.z = z * inv_len;
            self.w = w * inv_len;
        } else {
            self.x = 0.0;
            self.y = 0.0;
            self.z = 0.0;
            self.w = 1.0;
        }
        return self;
    }
    pub fn rotate_vec(&self, v: Vec3) -> Vec3 {
        let qx = self.x;
        let qy = self.y;
        let qz = self.z;
        let qw = self.w;
        let vx = v.x;
        let vy = v.y;
        let vz = v.z;

        let tx = 2.0 * (qy * vz - qz * vy);
        let ty = 2.0 * (qz * vx - qx * vz);
        let tz = 2.0 * (qx * vy - qy * vx);

        Vec3 {
            x: vx + qw * tx + (qy * tz - qz * ty),
            y: vy + qw * ty + (qz * tx - qx * tz),
            z: vz + qw * tz + (qx * ty - qy * tx),
        }
    }
    pub fn set(mut self, x: f32, y: f32, z: f32) -> Self {
        self.x = x;
        self.y = y;
        self.z = z;
        return self;
    }
    pub fn clone(&self) -> Quat {
        Quat {
            x: self.x,
            y: self.x,
            z: self.x,
            w: self.x,
        }
    }
    pub fn multiply(&mut self, other: Quat) -> Self {
        let ax = self.x;
        let ay = self.y;
        let az = self.z;
        let aw = self.w;
        let bx = other.x;
        let by = other.y;
        let bz = other.z;
        let bw = other.w;
        self.x = aw * bx + ax * bw + ay * bz - az * by;
        self.y = aw * by - ax * bz + ay * bw + az * bx;
        self.z = aw * bz + ax * by - ay * bx + az * bw;
        self.w = aw * bw - ax * bx - ay * by - az * bz;
        self.clone()
    }
}

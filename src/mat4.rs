use js_sys::Float32Array;
use wasm_bindgen::prelude::*;
use crate::EPS;
use crate::quat::Quat;
use crate::vec3::Vec3;
#[wasm_bindgen]
pub struct Mat4 {
    values: Vec<f32>,
}
#[wasm_bindgen]
impl Mat4 {
    #[wasm_bindgen(constructor)]
    pub fn new(val: Vec<f32>) -> Self {
        if val.capacity() != 16 {
            Mat4 { values: val }
        } else {
            Mat4 {
                values: vec![
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                ],
            }
        }
    }
    pub fn look_at(eye: Vec3, target: Vec3, up: Vec3) -> Mat4 {
        let forward_x = target.x - eye.x;
        let forward_y = target.y - eye.y;
        let forward_z = target.z - eye.z;

        let forward_len =
            f32::sqrt(forward_x * forward_x + forward_y * forward_y + forward_z * forward_z);
        let inv_forward_len = if forward_len > EPS {
            1.0 / forward_len
        } else {
            0.0
        };
        let f_x = forward_x * inv_forward_len;
        let f_y = forward_y * inv_forward_len;
        let f_z = forward_z * inv_forward_len;

        let right_x = up.y * f_z - up.z * f_y;
        let right_y = up.z * f_x - up.x * f_z;
        let right_z = up.x * f_y - up.y * f_x;

        let right_len = f32::sqrt(right_x * right_x + right_y * right_y + right_z * right_z);
        let inv_right_len = if right_len > EPS {
            1.0 / right_len
        } else {
            0.0
        };
        let r_x = right_x * inv_right_len;
        let r_y = right_y * inv_right_len;
        let r_z = right_z * inv_right_len;

        let u_x = f_y * r_z - f_z * r_y;
        let u_y = f_z * r_x - f_x * r_z;
        let u_z = f_x * r_y - f_y * r_x;
        Mat4 {
            values: vec![
                r_x,
                u_x,
                f_x,
                0.0,
                r_y,
                u_y,
                f_y,
                0.0,
                r_z,
                u_z,
                f_z,
                0.0,
                -(r_x * eye.x + r_y * eye.y + r_z * eye.z),
                -(u_x * eye.x + u_y * eye.y + u_z * eye.z),
                -(f_x * eye.x + f_y * eye.y + f_z * eye.z),
                1.0,
            ],
        }
    }
    pub fn perspective(fov: f32, aspect: f32, near: f32, far: f32) -> Mat4 {
        let f = 1.0 / f32::tan(fov * 0.5);
        let range_inv = 1.0 / (far - near);
        Mat4 {
            values: vec![
                f / aspect,
                0.0,
                0.0,
                0.0,
                0.0,
                f,
                0.0,
                0.0,
                0.0,
                0.0,
                (far + near) * range_inv,
                1.0,
                0.0,
                0.0,
                -2.0 * near * far * range_inv,
                0.0,
            ],
        }
    }
    pub fn scale_in_place(&mut self, sx: f32, sy: f32, sz: f32) -> Self {
        let v = &mut self.values;

        v[0] *= sx;
        v[1] *= sx;
        v[2] *= sx;
        v[3] *= sx;

        v[4] *= sy;
        v[5] *= sy;
        v[6] *= sy;
        v[7] *= sy;

        v[8] *= sz;
        v[9] *= sz;
        v[10] *= sz;
        v[11] *= sz;
        Mat4::new(v.to_vec())
    }
    pub fn scale_in_place_by_vec3(&mut self, scale: Vec3) -> Self {
        self.scale_in_place(scale.x, scale.y, scale.z)
    }
    pub fn multiply(&self, other: Mat4) -> Mat4 {
        let a = &self.values;
        let b = other.values;
        let mut out = vec![];

        let mut b0: f32;
        let mut b1: f32;
        let mut b2: f32;
        let mut b3: f32;

        b0 = b[0];
        b1 = b[1];
        b2 = b[2];
        b3 = b[3];
        out[0] = a[0] * b0 + a[4] * b1 + a[8] * b2 + a[12] * b3;
        out[1] = a[1] * b0 + a[5] * b1 + a[9] * b2 + a[13] * b3;
        out[2] = a[2] * b0 + a[6] * b1 + a[10] * b2 + a[14] * b3;
        out[3] = a[3] * b0 + a[7] * b1 + a[11] * b2 + a[15] * b3;

        b0 = b[4];
        b1 = b[5];
        b2 = b[6];
        b3 = b[7];
        out[4] = a[0] * b0 + a[4] * b1 + a[8] * b2 + a[12] * b3;
        out[5] = a[1] * b0 + a[5] * b1 + a[9] * b2 + a[13] * b3;
        out[6] = a[2] * b0 + a[6] * b1 + a[10] * b2 + a[14] * b3;
        out[7] = a[3] * b0 + a[7] * b1 + a[11] * b2 + a[15] * b3;

        b0 = b[8];
        b1 = b[9];
        b2 = b[10];
        b3 = b[11];
        out[8] = a[0] * b0 + a[4] * b1 + a[8] * b2 + a[12] * b3;
        out[9] = a[1] * b0 + a[5] * b1 + a[9] * b2 + a[13] * b3;
        out[10] = a[2] * b0 + a[6] * b1 + a[10] * b2 + a[14] * b3;
        out[11] = a[3] * b0 + a[7] * b1 + a[11] * b2 + a[15] * b3;

        b0 = b[12];
        b1 = b[13];
        b2 = b[14];
        b3 = b[15];
        out[12] = a[0] * b0 + a[4] * b1 + a[8] * b2 + a[12] * b3;
        out[13] = a[1] * b0 + a[5] * b1 + a[9] * b2 + a[13] * b3;
        out[14] = a[2] * b0 + a[6] * b1 + a[10] * b2 + a[14] * b3;
        out[15] = a[3] * b0 + a[7] * b1 + a[11] * b2 + a[15] * b3;
        Mat4::new(out)
    }
    pub fn to_quat(self) -> Quat {
        Mat4::to_quat_from_array(self.values, 0)
    }
    pub fn to_quat_from_array(m: Vec<f32>, offset: usize) -> Quat {
        let m00 = m[offset + 0];
        let m01 = m[offset + 4];
        let m02 = m[offset + 8];
        let m10 = m[offset + 1];
        let m11 = m[offset + 5];
        let m12 = m[offset + 9];
        let m20 = m[offset + 2];
        let m21 = m[offset + 6];
        let m22 = m[offset + 10];

        let trace = m00 + m11 + m22;
        let x: f32;
        let y: f32;
        let z: f32;
        let w: f32;

        if trace > 0.0 {
            let s = f32::sqrt(trace + 1.0) * 2.0;
            w = 0.25 * s;
            x = (m21 - m12) / s;
            y = (m02 - m20) / s;
            z = (m10 - m01) / s;
        } else if m00 > m11 && m00 > m22 {
            let s = f32::sqrt(1.0 + m00 - m11 - m22) * 2.0;
            w = (m21 - m12) / s;
            x = 0.25 * s;
            y = (m01 + m10) / s;
            z = (m02 + m20) / s;
        } else if m11 > m22 {
            let s = f32::sqrt(1.0 + m11 - m00 - m22) * 2.0;
            w = (m02 - m20) / s;
            x = (m01 + m10) / s;
            y = 0.25 * s;
            z = (m12 + m21) / s;
        } else {
            let s = f32::sqrt(1.0 + m22 - m00 - m11) * 2.0;
            w = (m10 - m01) / s;
            x = (m02 + m20) / s;
            y = (m12 + m21) / s;
            z = 0.25 * s;
        }

        let len = f32::sqrt(x * x + y * y + z * z + w * w);
        if len > EPS {
            let inv_len = 1.0 / len;
            Quat::new(x * inv_len, y * inv_len, z * inv_len, w * inv_len);
        }
        Quat::new(0.0, 0.0, 0.0, 1.0)
    }
    pub fn from_quat(x: f32, y: f32, z: f32, w: f32) -> Self {
        let x2 = x + x;
        let y2 = y + y;
        let z2 = z + z;
        let xx = x * x2;
        let xy = x * y2;
        let xz = x * z2;
        let yy = y * y2;
        let yz = y * z2;
        let zz = z * z2;
        let wx = w * x2;
        let wy = w * y2;
        let wz = w * z2;
        return Mat4 {
            values: vec![
                1.0 - (yy + zz),
                xy + wz,
                xz - wy,
                0.0,
                xy - wz,
                1.0 - (xx + zz),
                yz + wx,
                0.0,
                xz + wy,
                yz - wx,
                1.0 - (xx + yy),
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
        };
    }

    pub fn set_identity(mut self) -> Self {
        self.values = vec![
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        self
    }
    pub fn translate_in_place(mut self, tx: f32, ty: f32, tz: f32) -> Self {
        self.values[12] += tx;
        self.values[13] += ty;
        self.values[14] += tz;
        self
    }
    #[wasm_bindgen(getter)]
    pub fn values(&self) -> Float32Array {
        Float32Array::from(&self.values[..])
    }
    pub fn from_values(val: Vec<f32>) -> Result<Mat4, String> {
        if val.len() != 16 {
            Err("Mat4.FromValues requires exactly 16 values".to_string())
        } else {
            Ok(Mat4 { values: val })
        }
    }
    pub fn set(mut self, val: Vec<f32>) {
        self.values = val;
    }
    pub fn identity() -> Mat4 {
        Mat4 {
            values: vec![
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
        }
    }
}

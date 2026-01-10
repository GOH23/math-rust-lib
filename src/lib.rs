extern crate wasm_bindgen;
use js_sys::Float32Array;
use wasm_bindgen::prelude::*;
#[wasm_bindgen]
pub fn ease_in_out(t: f32) -> f32 {
    return if t < 0.5 {
        2.0 * t * t
    } else {
        -0.5 * (2.0 * t - 2.0) * (2.0 * t - 2.0) + 1.0
    };
}
//Vec3
#[wasm_bindgen]
pub fn lerp(start: Vec3, end: Vec3, t: f32) -> Vec3 {
    Vec3 {
        x: start.x + (end.x - start.x) * t,
        y: start.y + (end.y - start.y) * t,
        z: start.z + (end.z - start.z) * t,
    }
}
#[wasm_bindgen]
pub fn cross(a: Vec3, b: Vec3) -> Vec3 {
    let mut result: Vec3 = Vec3::new(0.0, 0.0, 0.0);
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    result
}
pub fn transform_coordinates(vector: Vec3, transformation: Mat4) -> Vec3 {
    let x = vector.x;
    let y = vector.y;
    let z = vector.z;
    let m = transformation.values;
    let rx = x * m[0] + y * m[4] + z * m[8] + m[12];
    let ry = x * m[1] + y * m[5] + z * m[9] + m[13];
    let rz = x * m[2] + y * m[6] + z * m[10] + m[14];
    let rw = x * m[3] + y * m[7] + z * m[11] + m[15];
    if rw.abs() > EPSILON {
        let inv_w = 1.0 / rw;
        Vec3::new(rx * inv_w, ry * inv_w, rz * inv_w)
    } else {
        Vec3::new(rx, ry, rz)
    }
}
//consts
const EPSILON: f32 = 1e-8;
const EPSILON_SQ: f32 = 1e-16;



#[wasm_bindgen]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
#[wasm_bindgen]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}
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
        let inv_forward_len = if forward_len > EPSILON {
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
        let inv_right_len = if right_len > EPSILON {
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
        if len > EPSILON {
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

#[wasm_bindgen]
impl Vec3 {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Vec3 { x, y, z }
    }
    pub fn add(&mut self, other: Vec3) -> Self {
        self.x += other.x;
        self.y += other.y;
        self.z += other.z;
        Vec3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
    pub fn set(&mut self, x: f32, y: f32, z: f32) -> Vec3 {
        self.x = x;
        self.y = y;
        self.z = z;
        Vec3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
    pub fn subtract(&mut self, other: Vec3) -> Vec3 {
        self.x -= other.x;
        self.y -= other.y;
        self.z -= other.z;
        Vec3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
    pub fn normalize(&mut self) -> Vec3 {
        let len_sq = self.x * self.x + self.y * self.y + self.z * self.z;
        if len_sq > EPSILON_SQ {
            let inv_len = 1.0 / len_sq.sqrt();
            self.x = self.x * inv_len;
            self.y = self.y * inv_len;
            self.z = self.z * inv_len;
        } else {
            self.x = 0.0;
            self.y = 0.0;
            self.z = 0.0;
        }
        Vec3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
    pub fn cross(&self, other: Vec3) -> Vec3 {
        Vec3 {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }
    pub fn dot(&self, other: Vec3) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
    pub fn scale(&self, scalar: f32) -> Self {
        Vec3 {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
    // pub fn free(self) {
    //     VEC3_POOL.release(self);
    // }
    pub fn clone(&self) -> Self {
        Vec3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
    pub fn to_array(&self) -> Vec<f32> {
        return vec![self.x, self.y, self.z];
    }
    pub fn length(&self) -> f32 {
        f32::sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
    }
    pub fn length_squared(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }
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

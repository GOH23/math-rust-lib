mod quat;
mod vec3;
mod mat4;
extern crate wasm_bindgen;
use wasm_bindgen::prelude::*;
use crate::{mat4::Mat4, vec3::Vec3};

//consts
const EPS: f32 = 1e-8;
const EPSILON_SQ: f32 = 1e-16;
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
    let m = transformation.values().to_vec();
    let rx = x * m[0] + y * m[4] + z * m[8] + m[12];
    let ry = x * m[1] + y * m[5] + z * m[9] + m[13];
    let rz = x * m[2] + y * m[6] + z * m[10] + m[14];
    let rw = x * m[3] + y * m[7] + z * m[11] + m[15];
    if rw.abs() > EPS {
        let inv_w = 1.0 / rw;
        Vec3::new(rx * inv_w, ry * inv_w, rz * inv_w)
    } else {
        Vec3::new(rx, ry, rz)
    }
}
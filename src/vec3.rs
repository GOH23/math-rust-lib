use wasm_bindgen::prelude::*;

use crate::EPSILON_SQ;
#[wasm_bindgen]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
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

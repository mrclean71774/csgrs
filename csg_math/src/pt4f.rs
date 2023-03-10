// MIT License
//
// Copyright (c) 2023 Michael H. Phillips
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

use crate::pt3f::Pt3f;
use crate::rng::MersenneTwister;

/// A $D point.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pt4f {
  pub x: f32,
  pub y: f32,
  pub z: f32,
  pub w: f32,
}

impl std::fmt::Display for Pt4f {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    write!(f, "[ {}, {}, {}, {} ]", self.x, self.y, self.z, self.w)
  }
}

impl std::ops::Index<usize> for Pt4f {
  type Output = f32;

  fn index(&self, index: usize) -> &Self::Output {
    match index {
      0 => &self.x,
      1 => &self.y,
      2 => &self.z,
      3 => &self.w,
      _ => panic!("Index {} is out of bounds.", index),
    }
  }
}

impl std::ops::IndexMut<usize> for Pt4f {
  fn index_mut(&mut self, index: usize) -> &mut Self::Output {
    match index {
      0 => &mut self.x,
      1 => &mut self.y,
      2 => &mut self.z,
      3 => &mut self.w,
      _ => panic!("Index {} is out of bounds.", index),
    }
  }
}

impl std::ops::Add for Pt4f {
  type Output = Self;

  fn add(self, rhs: Self) -> Self::Output {
    Self::new(
      self.x + rhs.x,
      self.y + rhs.y,
      self.z + rhs.z,
      self.w + rhs.w,
    )
  }
}

impl std::ops::AddAssign for Pt4f {
  fn add_assign(&mut self, rhs: Self) {
    *self = *self + rhs;
  }
}

impl std::ops::Sub for Pt4f {
  type Output = Self;

  fn sub(self, rhs: Self) -> Self::Output {
    Self::new(
      self.x - rhs.x,
      self.y - rhs.y,
      self.z - rhs.z,
      self.w - rhs.w,
    )
  }
}

impl std::ops::SubAssign for Pt4f {
  fn sub_assign(&mut self, rhs: Self) {
    *self = *self - rhs;
  }
}

impl std::ops::Mul<f32> for Pt4f {
  type Output = Self;

  fn mul(self, rhs: f32) -> Self::Output {
    Self::new(self.x * rhs, self.y * rhs, self.z * rhs, self.w * rhs)
  }
}

impl std::ops::MulAssign<f32> for Pt4f {
  fn mul_assign(&mut self, rhs: f32) {
    *self = *self * rhs;
  }
}

impl std::ops::Div<f32> for Pt4f {
  type Output = Self;

  fn div(self, rhs: f32) -> Self::Output {
    Self::new(self.x / rhs, self.y / rhs, self.z / rhs, self.w / rhs)
  }
}

impl std::ops::DivAssign<f32> for Pt4f {
  fn div_assign(&mut self, rhs: f32) {
    *self = *self / rhs;
  }
}

impl std::ops::Neg for Pt4f {
  type Output = Self;

  fn neg(self) -> Self::Output {
    self * -1.0
  }
}

impl Pt4f {
  pub fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
    Self { x, y, z, w }
  }

  pub fn dot(self, rhs: Self) -> f32 {
    self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
  }

  pub fn cross(self, rhs: Self) -> Self {
    Pt4f::new(
      self.y * rhs.z - self.z * rhs.y,
      self.z * rhs.x - self.x * rhs.z,
      self.x * rhs.y - self.y * rhs.x,
      0.0,
    )
  }

  pub fn len2(self) -> f32 {
    self.dot(self)
  }

  pub fn len(self) -> f32 {
    self.len2().sqrt()
  }

  pub fn normalize(&mut self) {
    *self /= self.len();
  }

  pub fn normalized(self) -> Self {
    let l = self.len();
    Self::new(self.x / l, self.y / l, self.z / l, 0.0)
  }

  pub fn lerp(self, b: Self, t: f32) -> Self {
    self + (b - self) * t
  }

  pub fn as_pt3f(&self) -> Pt3f {
    Pt3f::new(self.x, self.y, self.z)
  }

  pub fn quadratic_bezier(start: Self, control: Self, end: Self, segments: usize) -> Vec<Self> {
    let delta = 1.0 / segments as f32;
    let mut points = Vec::new();
    for i in 0..(segments + 1) {
      let t = i as f32 * delta;
      points.push(start * (1.0 - t) * (1.0 - t) + control * t * (1.0 - t) * 2.0 + end * t * t);
    }
    points
  }

  pub fn cubic_bezier(
    start: Self,
    control1: Self,
    control2: Self,
    end: Self,
    segments: usize,
  ) -> Vec<Self> {
    let delta = 1.0 / segments as f32;
    let mut points = Vec::new();
    for i in 0..(segments + 1) {
      let t = i as f32 * delta;
      points.push(
        start * (1.0 - t) * (1.0 - t) * (1.0 - t)
          + control1 * t * (1.0 - t) * (1.0 - t) * 3.0
          + control2 * t * t * (1.0 - t) * 3.0
          + end * t * t * t,
      );
    }
    points
  }

  pub fn random_with_max_length(mt: &mut MersenneTwister, length: f32) -> Self {
    assert!(length > 0.0);
    loop {
      let point = Self::new(
        mt.f32_minmax(-1.0, 1.0),
        mt.f32_minmax(-1.0, 1.0),
        mt.f32_minmax(-1.0, 1.0),
        0.0,
      );
      if point.x * point.x + point.y * point.y < length * length {
        return point;
      }
    }
  }
}

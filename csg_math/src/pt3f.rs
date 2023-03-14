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

use crate::dcosf;
use crate::dsinf;
use crate::pt4f::Pt4f;
use crate::rng::MersenneTwister;

pub trait VecPt3f {
  fn translate(&mut self, pt: Pt3f) -> &mut Self;
  fn rotate_x(&mut self, degrees: f32) -> &mut Self;
  fn rotate_y(&mut self, degrees: f32) -> &mut Self;
  fn rotate_z(&mut self, degrees: f32) -> &mut Self;
}

impl VecPt3f for Vec<Pt3f> {
  fn translate(&mut self, pt: Pt3f) -> &mut Self {
    for p in self.iter_mut() {
      *p += pt;
    }
    self
  }

  fn rotate_x(&mut self, degrees: f32) -> &mut Self {
    for p in self.iter_mut() {
      p.rotate_x(degrees);
    }
    self
  }

  fn rotate_y(&mut self, degrees: f32) -> &mut Self {
    for p in self.iter_mut() {
      p.rotate_y(degrees);
    }
    self
  }

  fn rotate_z(&mut self, degrees: f32) -> &mut Self {
    for p in self.iter_mut() {
      p.rotate_z(degrees);
    }
    self
  }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pt3f {
  pub x: f32,
  pub y: f32,
  pub z: f32,
}

impl std::fmt::Display for Pt3f {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    write!(f, "[ {}, {}, {} ]", self.x, self.y, self.z)
  }
}

impl std::ops::Index<usize> for Pt3f {
  type Output = f32;

  fn index(&self, index: usize) -> &Self::Output {
    match index {
      0 => &self.x,
      1 => &self.y,
      2 => &self.z,
      _ => panic!("Index {} is out of bounds.", index),
    }
  }
}

impl std::ops::IndexMut<usize> for Pt3f {
  fn index_mut(&mut self, index: usize) -> &mut Self::Output {
    match index {
      0 => &mut self.x,
      1 => &mut self.y,
      2 => &mut self.z,
      _ => panic!("Index {} is out of bounds.", index),
    }
  }
}

impl std::ops::Add for Pt3f {
  type Output = Self;

  fn add(self, rhs: Self) -> Self::Output {
    Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
  }
}

impl std::ops::AddAssign for Pt3f {
  fn add_assign(&mut self, rhs: Self) {
    *self = *self + rhs;
  }
}

impl std::ops::Sub for Pt3f {
  type Output = Self;

  fn sub(self, rhs: Self) -> Self::Output {
    Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
  }
}

impl std::ops::SubAssign for Pt3f {
  fn sub_assign(&mut self, rhs: Self) {
    *self = *self - rhs;
  }
}

impl std::ops::Mul<f32> for Pt3f {
  type Output = Self;

  fn mul(self, rhs: f32) -> Self::Output {
    Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
  }
}

impl std::ops::MulAssign<f32> for Pt3f {
  fn mul_assign(&mut self, rhs: f32) {
    *self = *self * rhs;
  }
}

impl std::ops::Div<f32> for Pt3f {
  type Output = Self;

  fn div(self, rhs: f32) -> Self::Output {
    Self::new(self.x / rhs, self.y / rhs, self.z / rhs)
  }
}

impl std::ops::DivAssign<f32> for Pt3f {
  fn div_assign(&mut self, rhs: f32) {
    *self = *self / rhs;
  }
}

impl std::ops::Neg for Pt3f {
  type Output = Self;

  fn neg(self) -> Self::Output {
    self * -1.0
  }
}

impl Pt3f {
  pub fn new(x: f32, y: f32, z: f32) -> Self {
    Self { x, y, z }
  }

  pub fn dot(self, rhs: Self) -> f32 {
    self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
  }

  pub fn cross(self, rhs: Self) -> Self {
    Pt3f::new(
      self.y * rhs.z - self.z * rhs.y,
      self.z * rhs.x - self.x * rhs.z,
      self.x * rhs.y - self.y * rhs.x,
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
    Self::new(self.x / l, self.y / l, self.z / l)
  }

  pub fn rotated_x(self, degrees: f32) -> Self {
    let s = dsinf(degrees);
    let c = dcosf(degrees);
    Self::new(self.x, self.y * c - self.z * s, self.y * s + self.z * c)
  }

  pub fn rotate_x(&mut self, degrees: f32) {
    *self = self.rotated_x(degrees);
  }

  pub fn rotated_y(self, degrees: f32) -> Self {
    let s = dsinf(degrees);
    let c = dcosf(degrees);
    Self::new(self.x * c - self.z * s, self.y, self.x * s + self.z * c)
  }

  pub fn rotate_y(&mut self, degrees: f32) {
    *self = self.rotated_y(degrees);
  }

  pub fn rotated_z(self, degrees: f32) -> Self {
    let s = dsinf(degrees);
    let c = dcosf(degrees);
    Self::new(self.x * c - self.y * s, self.x * s + self.y * c, self.z)
  }

  pub fn rotate_z(&mut self, degrees: f32) {
    *self = self.rotated_z(degrees);
  }

  pub fn lerp(self, b: Self, t: f32) -> Self {
    self + (b - self) * t
  }

  pub fn as_pt4f(self, w: f32) -> Pt4f {
    Pt4f::new(self.x, self.y, self.z, w)
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
      );
      if point.x * point.x + point.y * point.y < length * length {
        return point;
      }
    }
  }
}

#[derive(Clone, Copy)]
pub struct QuadraticBezier3Df {
  pub start: Pt3f,
  pub control: Pt3f,
  pub end: Pt3f,
  pub segments: usize,
}

impl QuadraticBezier3Df {
  pub fn new(start: Pt3f, control: Pt3f, end: Pt3f, segments: usize) -> Self {
    Self {
      start,
      control,
      end,
      segments,
    }
  }

  pub fn gen_points(&self) -> Vec<Pt3f> {
    Pt3f::quadratic_bezier(self.start, self.control, self.end, self.segments)
  }
}

#[derive(Clone, Copy)]
pub struct CubicBezier3Df {
  pub start: Pt3f,
  pub control1: Pt3f,
  pub control2: Pt3f,
  pub end: Pt3f,
  pub segments: usize,
}

impl CubicBezier3Df {
  pub fn new(start: Pt3f, control1: Pt3f, control2: Pt3f, end: Pt3f, segments: usize) -> Self {
    Self {
      start,
      control1,
      control2,
      end,
      segments,
    }
  }

  pub fn gen_points(&self) -> Vec<Pt3f> {
    Pt3f::cubic_bezier(
      self.start,
      self.control1,
      self.control2,
      self.end,
      self.segments,
    )
  }
}

#[derive(Clone)]
pub struct CubicBezierChain3Df {
  curves: Vec<CubicBezier3Df>,
  closed: bool,
}

impl CubicBezierChain3Df {
  pub fn new(start: Pt3f, control1: Pt3f, control2: Pt3f, end: Pt3f, segments: usize) -> Self {
    Self {
      curves: vec![CubicBezier3Df {
        start,
        control1,
        control2,
        end,
        segments,
      }],
      closed: false,
    }
  }

  pub fn add(
    &mut self,
    control1_length: f32,
    control2: Pt3f,
    end: Pt3f,
    segments: usize,
  ) -> &mut Self {
    let chain_end = &self.curves[self.curves.len() - 1];
    self.curves.push(CubicBezier3Df {
      start: chain_end.end,
      control1: chain_end.end + (chain_end.end - chain_end.control2).normalized() * control1_length,
      control2,
      end,
      segments,
    });
    self
  }

  pub fn close(
    &mut self,
    control1_length: f32,
    control2: Pt3f,
    start_control1_len: f32,
    segments: usize,
  ) {
    self.closed = true;
    self.add(control1_length, control2, self.curves[0].start, segments);
    let chain_end = &self.curves[self.curves.len() - 1];
    self.curves[0].control1 =
      chain_end.end + (chain_end.end - chain_end.control2).normalized() * start_control1_len;
  }

  pub fn gen_points(&self) -> Vec<Pt3f> {
    let mut pts = vec![Pt3f::new(0.0, 0.0, 0.0)];
    for i in 0..self.curves.len() {
      pts.pop();
      pts.append(&mut Pt3f::cubic_bezier(
        self.curves[i].start,
        self.curves[i].control1,
        self.curves[i].control2,
        self.curves[i].end,
        self.curves[i].segments,
      ));
    }
    if self.closed {
      pts.pop();
    }
    pts
  }
}

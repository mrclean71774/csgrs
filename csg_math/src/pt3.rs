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

use crate::dcos;
use crate::dsin;
use crate::pt4::Pt4;
use crate::rng::MersenneTwister;

pub trait VecPt3 {
  fn translate(&mut self, pt: Pt3) -> &mut Self;
  fn rotate_x(&mut self, degrees: f64) -> &mut Self;
  fn rotate_y(&mut self, degrees: f64) -> &mut Self;
  fn rotate_z(&mut self, degrees: f64) -> &mut Self;
}

impl VecPt3 for Vec<Pt3> {
  fn translate(&mut self, pt: Pt3) -> &mut Self {
    for p in self.iter_mut() {
      *p += pt;
    }
    self
  }

  fn rotate_x(&mut self, degrees: f64) -> &mut Self {
    for p in self.iter_mut() {
      p.rotate_x(degrees);
    }
    self
  }

  fn rotate_y(&mut self, degrees: f64) -> &mut Self {
    for p in self.iter_mut() {
      p.rotate_y(degrees);
    }
    self
  }

  fn rotate_z(&mut self, degrees: f64) -> &mut Self {
    for p in self.iter_mut() {
      p.rotate_z(degrees);
    }
    self
  }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pt3 {
  pub x: f64,
  pub y: f64,
  pub z: f64,
}

impl std::fmt::Display for Pt3 {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    write!(f, "[ {}, {}, {} ]", self.x, self.y, self.z)
  }
}

impl std::ops::Index<usize> for Pt3 {
  type Output = f64;

  fn index(&self, index: usize) -> &Self::Output {
    match index {
      0 => &self.x,
      1 => &self.y,
      2 => &self.z,
      _ => panic!("Index {} is out of bounds.", index),
    }
  }
}

impl std::ops::IndexMut<usize> for Pt3 {
  fn index_mut(&mut self, index: usize) -> &mut Self::Output {
    match index {
      0 => &mut self.x,
      1 => &mut self.y,
      2 => &mut self.z,
      _ => panic!("Index {} is out of bounds.", index),
    }
  }
}

impl std::ops::Add for Pt3 {
  type Output = Self;

  fn add(self, rhs: Self) -> Self::Output {
    Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
  }
}

impl std::ops::AddAssign for Pt3 {
  fn add_assign(&mut self, rhs: Self) {
    *self = *self + rhs;
  }
}

impl std::ops::Sub for Pt3 {
  type Output = Self;

  fn sub(self, rhs: Self) -> Self::Output {
    Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
  }
}

impl std::ops::SubAssign for Pt3 {
  fn sub_assign(&mut self, rhs: Self) {
    *self = *self - rhs;
  }
}

impl std::ops::Mul<f64> for Pt3 {
  type Output = Self;

  fn mul(self, rhs: f64) -> Self::Output {
    Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
  }
}

impl std::ops::MulAssign<f64> for Pt3 {
  fn mul_assign(&mut self, rhs: f64) {
    *self = *self * rhs;
  }
}

impl std::ops::Div<f64> for Pt3 {
  type Output = Self;

  fn div(self, rhs: f64) -> Self::Output {
    Self::new(self.x / rhs, self.y / rhs, self.z / rhs)
  }
}

impl std::ops::DivAssign<f64> for Pt3 {
  fn div_assign(&mut self, rhs: f64) {
    *self = *self / rhs;
  }
}

impl std::ops::Neg for Pt3 {
  type Output = Self;

  fn neg(self) -> Self::Output {
    self * -1.0
  }
}

impl Pt3 {
  pub fn new(x: f64, y: f64, z: f64) -> Self {
    Self { x, y, z }
  }

  pub fn dot(self, rhs: Self) -> f64 {
    self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
  }

  pub fn cross(self, rhs: Self) -> Self {
    Pt3::new(
      self.y * rhs.z - self.z * rhs.y,
      self.z * rhs.x - self.x * rhs.z,
      self.x * rhs.y - self.y * rhs.x,
    )
  }

  pub fn len2(self) -> f64 {
    self.dot(self)
  }

  pub fn len(self) -> f64 {
    self.len2().sqrt()
  }

  pub fn normalize(&mut self) {
    *self /= self.len();
  }

  pub fn normalized(self) -> Self {
    let l = self.len();
    Self::new(self.x / l, self.y / l, self.z / l)
  }

  pub fn rotated_x(self, degrees: f64) -> Self {
    let s = dsin(degrees);
    let c = dcos(degrees);
    Self::new(self.x, self.y * c - self.z * s, self.y * s + self.z * c)
  }

  pub fn rotate_x(&mut self, degrees: f64) {
    *self = self.rotated_x(degrees);
  }

  pub fn rotated_y(self, degrees: f64) -> Self {
    let s = dsin(degrees);
    let c = dcos(degrees);
    Self::new(self.x * c - self.z * s, self.y, self.x * s + self.z * c)
  }

  pub fn rotate_y(&mut self, degrees: f64) {
    *self = self.rotated_y(degrees);
  }

  pub fn rotated_z(self, degrees: f64) -> Self {
    let s = dsin(degrees);
    let c = dcos(degrees);
    Self::new(self.x * c - self.y * s, self.x * s + self.y * c, self.z)
  }

  pub fn rotate_z(&mut self, degrees: f64) {
    *self = self.rotated_z(degrees);
  }

  pub fn lerp(self, b: Self, t: f64) -> Self {
    self + (b - self) * t
  }

  pub fn as_pt4(self, w: f64) -> Pt4 {
    Pt4::new(self.x, self.y, self.z, w)
  }

  pub fn quadratic_bezier(start: Self, control: Self, end: Self, segments: usize) -> Vec<Self> {
    let delta = 1.0 / segments as f64;
    let mut points = Vec::new();
    for i in 0..(segments + 1) {
      let t = i as f64 * delta;
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
    let delta = 1.0 / segments as f64;
    let mut points = Vec::new();
    for i in 0..(segments + 1) {
      let t = i as f64 * delta;
      points.push(
        start * (1.0 - t) * (1.0 - t) * (1.0 - t)
          + control1 * t * (1.0 - t) * (1.0 - t) * 3.0
          + control2 * t * t * (1.0 - t) * 3.0
          + end * t * t * t,
      );
    }
    points
  }

  pub fn random_with_max_length(mt: &mut MersenneTwister, length: f64) -> Self {
    assert!(length > 0.0);
    loop {
      let point = Self::new(
        mt.f64_minmax(-1.0, 1.0),
        mt.f64_minmax(-1.0, 1.0),
        mt.f64_minmax(-1.0, 1.0),
      );
      if point.x * point.x + point.y * point.y < length * length {
        return point;
      }
    }
  }
}

#[derive(Clone, Copy)]
pub struct QuadraticBezier3D {
  pub start: Pt3,
  pub control: Pt3,
  pub end: Pt3,
}

impl QuadraticBezier3D {
  pub fn new(start: Pt3, control: Pt3, end: Pt3) -> Self {
    Self {
      start,
      control,
      end,
    }
  }

  pub fn gen_points(&self, segments: usize) -> Vec<Pt3> {
    Pt3::quadratic_bezier(self.start, self.control, self.end, segments)
  }
}

#[derive(Clone, Copy)]
pub struct CubicBezier3D {
  pub start: Pt3,
  pub control1: Pt3,
  pub control2: Pt3,
  pub end: Pt3,
}

impl CubicBezier3D {
  pub fn new(start: Pt3, control1: Pt3, control2: Pt3, end: Pt3) -> Self {
    Self {
      start,
      control1,
      control2,
      end,
    }
  }

  pub fn gen_points(&self, segments: usize) -> Vec<Pt3> {
    Pt3::cubic_bezier(self.start, self.control1, self.control2, self.end, segments)
  }
}

#[derive(Clone)]
pub struct CubicBezierChain3D {
  curves: Vec<CubicBezier3D>,
  closed: bool,
}

impl CubicBezierChain3D {
  pub fn new(start: Pt3, control1: Pt3, control2: Pt3, end: Pt3) -> Self {
    Self {
      curves: vec![CubicBezier3D {
        start,
        control1,
        control2,
        end,
      }],
      closed: false,
    }
  }

  pub fn add(&mut self, control1_length: f64, control2: Pt3, end: Pt3) -> &mut Self {
    let chain_end = &self.curves[self.curves.len() - 1];
    self.curves.push(CubicBezier3D {
      start: chain_end.end,
      control1: chain_end.end + (chain_end.end - chain_end.control2).normalized() * control1_length,
      control2: control2,
      end: end,
    });
    self
  }

  pub fn close(&mut self, control1_length: f64, control2: Pt3, start_control1_len: f64) {
    self.closed = true;
    self.add(control1_length, control2, self.curves[0].start);
    let chain_end = &self.curves[self.curves.len() - 1];
    self.curves[0].control1 =
      chain_end.end + (chain_end.end - chain_end.control2).normalized() * start_control1_len;
  }

  pub fn gen_points(&self, pts_per_section: usize) -> Vec<Pt3> {
    let mut pts = vec![Pt3::new(0.0, 0.0, 0.0)];
    for i in 0..self.curves.len() {
      pts.pop();
      pts.append(&mut Pt3::cubic_bezier(
        self.curves[i].start,
        self.curves[i].control1,
        self.curves[i].control2,
        self.curves[i].end,
        pts_per_section,
      ));
    }
    if self.closed {
      pts.pop();
    }
    pts
  }
}

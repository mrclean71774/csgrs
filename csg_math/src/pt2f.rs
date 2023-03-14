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
use crate::pt3f::Pt3f;
use crate::pt4f::Pt4f;
use crate::rng::MersenneTwister;

pub trait VecPt2f {
  fn translate(&mut self, pt: Pt2f) -> &mut Self;

  fn rotate(&mut self, degrees: f32) -> &mut Self;
}

impl VecPt2f for Vec<Pt2f> {
  fn translate(&mut self, pt: Pt2f) -> &mut Self {
    for p in self.iter_mut() {
      *p += pt;
    }
    self
  }

  fn rotate(&mut self, degrees: f32) -> &mut Self {
    for p in self.iter_mut() {
      p.rotate(degrees);
    }
    self
  }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pt2f {
  pub x: f32,
  pub y: f32,
}

impl std::fmt::Display for Pt2f {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    write!(f, "[ {}, {} ]", self.x, self.y)
  }
}

impl std::ops::Index<usize> for Pt2f {
  type Output = f32;

  fn index(&self, index: usize) -> &Self::Output {
    match index {
      0 => &self.x,
      1 => &self.y,
      _ => panic!("Index {} is out of bounds.", index),
    }
  }
}

impl std::ops::IndexMut<usize> for Pt2f {
  fn index_mut(&mut self, index: usize) -> &mut Self::Output {
    match index {
      0 => &mut self.x,
      1 => &mut self.y,
      _ => panic!("Index {} is out of bounds.", index),
    }
  }
}

impl std::ops::Add for Pt2f {
  type Output = Self;

  fn add(self, rhs: Self) -> Self::Output {
    Self::new(self.x + rhs.x, self.y + rhs.y)
  }
}

impl std::ops::AddAssign for Pt2f {
  fn add_assign(&mut self, rhs: Self) {
    *self = *self + rhs;
  }
}

impl std::ops::Sub for Pt2f {
  type Output = Self;

  fn sub(self, rhs: Self) -> Self::Output {
    Self::new(self.x - rhs.x, self.y - rhs.y)
  }
}

impl std::ops::SubAssign for Pt2f {
  fn sub_assign(&mut self, rhs: Self) {
    *self = *self - rhs;
  }
}

impl std::ops::Mul<f32> for Pt2f {
  type Output = Self;

  fn mul(self, rhs: f32) -> Self::Output {
    Self::new(self.x * rhs, self.y * rhs)
  }
}

impl std::ops::MulAssign<f32> for Pt2f {
  fn mul_assign(&mut self, rhs: f32) {
    *self = *self * rhs;
  }
}

impl std::ops::Div<f32> for Pt2f {
  type Output = Self;

  fn div(self, rhs: f32) -> Self::Output {
    Self::new(self.x / rhs, self.y / rhs)
  }
}

impl std::ops::DivAssign<f32> for Pt2f {
  fn div_assign(&mut self, rhs: f32) {
    *self = *self / rhs;
  }
}

impl std::ops::Neg for Pt2f {
  type Output = Self;

  fn neg(self) -> Self::Output {
    self * -1.0
  }
}

impl Pt2f {
  pub fn new(x: f32, y: f32) -> Self {
    Self { x, y }
  }

  pub fn dot(self, rhs: Pt2f) -> f32 {
    self.x * rhs.x + self.y * rhs.y
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
    Self::new(self.x / l, self.y / l)
  }

  pub fn rotate(&mut self, degrees: f32) {
    *self = self.rotated(degrees);
  }

  pub fn rotated(self, degrees: f32) -> Self {
    let c = dcosf(degrees);
    let s = dsinf(degrees);
    Self::new(self.x * c - self.y * s, self.x * s + self.y * c)
  }

  pub fn lerp(self, b: Self, t: f32) -> Self {
    self + (b - self) * t
  }

  pub fn to_xz(self) -> Pt3f {
    Pt3f::new(self.x, 0.0, self.y)
  }

  pub fn as_pt3f(self, z: f32) -> Pt3f {
    Pt3f::new(self.x, self.y, z)
  }

  pub fn as_pt4f(self, z: f32, w: f32) -> Pt4f {
    Pt4f::new(self.x, self.y, z, w)
  }

  pub fn arc(start: Self, degrees: f32, segments: usize) -> Vec<Self> {
    assert!(degrees <= 360.0);
    let n_pts = if degrees == 360.0 {
      segments
    } else {
      segments + 1
    };
    let mut pts = Vec::with_capacity(n_pts);
    for i in 0..n_pts {
      let a = i as f32 * degrees / segments as f32;
      pts.push(start.rotated(a));
    }
    pts
  }

  pub fn rounded_rect(
    width: f32,
    height: f32,
    radius: f32,
    segments: usize,
    center: bool,
  ) -> Vec<Self> {
    let mut tr = Self::arc(Self::new(radius, 0.0), 90.0, segments);
    tr.translate(Self::new(width - radius, height - radius));
    let mut tl = Self::arc(Self::new(0.0, radius), 90.0, segments);
    tl.translate(Self::new(radius, height - radius));
    let mut bl = Self::arc(Self::new(-radius, 0.0), 90.0, segments);
    bl.translate(Self::new(radius, radius));
    let mut br = Self::arc(Self::new(0.0, -radius), 90.0, segments);
    br.translate(Self::new(width - radius, radius));

    tr.append(&mut tl);
    tr.append(&mut bl);
    tr.append(&mut br);

    if center {
      tr.translate(Self::new(-width / 2.0, -height / 2.0));
    }
    tr
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
      let point = Self::new(mt.f32_minmax(-1.0, 1.0), mt.f32_minmax(-1.0, 1.0));
      if point.x * point.x + point.y * point.y < length * length {
        return point;
      }
    }
  }

  /// radius: the radius of the circle surrounding the polygon
  pub fn inscribed_polygon(n_sides: usize, radius: f32) -> Vec<Self> {
    let angle = 360.0 / n_sides as f32;
    let mut points = Vec::new();
    for i in 0..n_sides {
      points.push(Pt2f::new(
        dcosf(angle * i as f32) * radius,
        dsinf(angle * i as f32) * radius,
      ));
    }
    points
  }

  /// radius: the radius of the circle inside the polygon
  pub fn circumscribed_polygon(n_sides: usize, radius: f32) -> Vec<Self> {
    let radius = radius / dcosf(180.0 / n_sides as f32);
    Self::inscribed_polygon(n_sides, radius)
  }

  pub fn star(n_points: usize, inner_radius: f32, outer_radius: f32) -> Vec<Self> {
    let angle = 360.0 / n_points as f32;
    let mut points = Vec::new();
    for i in 0..n_points {
      points.push(Pt2f::new(
        dcosf(angle * i as f32) * inner_radius,
        dsinf(angle * i as f32) * inner_radius,
      ));
      points.push(Pt2f::new(
        dcosf(angle * (i as f32 + 0.5)) * outer_radius,
        dsinf(angle * (i as f32 + 0.5)) * outer_radius,
      ));
    }
    points
  }

  pub fn bezier_star(
    n_points: usize,
    inner_radius: f32,
    inner_handle_length: f32,
    outer_radius: f32,
    outer_handle_length: f32,
    segments: usize,
  ) -> Vec<Self> {
    let mut controls = Vec::new();
    let mut knots = Vec::new();

    let angle = 360.0 / n_points as f32;
    for i in 0..n_points {
      knots.push(Pt2f::new(
        dcosf(angle * i as f32) * outer_radius,
        dsinf(angle * i as f32) * outer_radius,
      ));
      knots.push(Pt2f::new(
        dcosf(angle * (i as f32 + 0.5)) * inner_radius,
        dsinf(angle * (i as f32 + 0.5)) * inner_radius,
      ));
    }
    let n_knots = knots.len();
    for i in 0..n_knots {
      controls.push(
        knots[(i + 1) % n_knots]
          - (knots[(i + 2) % n_knots] - knots[i]).normalized()
            * if i % 2 == 0 {
              inner_handle_length
            } else {
              outer_handle_length
            },
      );
    }

    let mut chain = CubicBezierChain2Df::new(knots[0], controls[0], controls[0], knots[1]);
    for i in 1..(n_knots - 1) {
      chain.add(
        if i % 2 == 0 {
          outer_handle_length
        } else {
          inner_handle_length
        },
        controls[i],
        knots[i + 1],
      );
    }
    chain.close(
      inner_handle_length,
      controls[n_knots - 1],
      outer_handle_length,
    );

    chain.gen_points(segments)
  }
  pub fn chamfer(size: f32, oversize: f32) -> Vec<Self> {
    vec![
      Self::new(0.0, 0.0),
      Self::new(0.0, size + oversize),
      Self::new(oversize, size + oversize),
      Self::new(oversize, size),
      Self::new(size, oversize),
      Self::new(size + oversize, oversize),
      Self::new(oversize + size, 0.0),
    ]
  }
}

#[derive(Clone, Copy)]
pub struct QuadraticBezier2Df {
  pub start: Pt2f,
  pub control: Pt2f,
  pub end: Pt2f,
}

impl QuadraticBezier2Df {
  pub fn new(start: Pt2f, control: Pt2f, end: Pt2f) -> Self {
    Self {
      start,
      control,
      end,
    }
  }

  pub fn gen_points(&self, segments: usize) -> Vec<Pt2f> {
    Pt2f::quadratic_bezier(self.start, self.control, self.end, segments)
  }
}

#[derive(Clone, Copy)]
pub struct CubicBezier2Df {
  pub start: Pt2f,
  pub control1: Pt2f,
  pub control2: Pt2f,
  pub end: Pt2f,
}

impl CubicBezier2Df {
  pub fn new(start: Pt2f, control1: Pt2f, control2: Pt2f, end: Pt2f) -> Self {
    Self {
      start,
      control1,
      control2,
      end,
    }
  }

  pub fn gen_points(&self, segments: usize) -> Vec<Pt2f> {
    Pt2f::cubic_bezier(self.start, self.control1, self.control2, self.end, segments)
  }
}

#[derive(Clone)]
pub struct CubicBezierChain2Df {
  curves: Vec<CubicBezier2Df>,
  closed: bool,
}

impl CubicBezierChain2Df {
  pub fn new(start: Pt2f, control1: Pt2f, control2: Pt2f, end: Pt2f) -> Self {
    Self {
      curves: vec![CubicBezier2Df {
        start,
        control1,
        control2,
        end,
      }],
      closed: false,
    }
  }

  pub fn add(&mut self, control1_length: f32, control2: Pt2f, end: Pt2f) -> &mut Self {
    let chain_end = &self.curves[self.curves.len() - 1];
    self.curves.push(CubicBezier2Df {
      start: chain_end.end,
      control1: chain_end.end + (chain_end.end - chain_end.control2).normalized() * control1_length,
      control2: control2,
      end: end,
    });
    self
  }

  pub fn close(&mut self, control1_length: f32, control2: Pt2f, start_control1_len: f32) {
    self.closed = true;
    self.add(control1_length, control2, self.curves[0].start);
    let chain_end = &self.curves[self.curves.len() - 1];
    self.curves[0].control1 =
      chain_end.end + (chain_end.end - chain_end.control2).normalized() * start_control1_len;
  }

  pub fn gen_points(&self, pts_per_section: usize) -> Vec<Pt2f> {
    let mut pts = vec![Pt2f::new(0.0, 0.0)];
    for i in 0..self.curves.len() {
      pts.pop();
      pts.append(&mut Pt2f::cubic_bezier(
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

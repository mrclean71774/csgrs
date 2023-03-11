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
use crate::pt3::Pt3;
use crate::pt4::Pt4;
use crate::rng::MersenneTwister;

/// Functions for Vec<Pt2>
pub trait VecPt2 {
  /// Translate a Vec<Pt2> by translating each Pt2
  fn translate(&mut self, pt: Pt2) -> &mut Self;

  /// Rotate a Vec<Pt2> by rotating each Pt2
  fn rotate(&mut self, degrees: f64) -> &mut Self;
}

impl VecPt2 for Vec<Pt2> {
  /// Translate a Vec<Pt2> by translating each Pt2
  fn translate(&mut self, pt: Pt2) -> &mut Self {
    for p in self.iter_mut() {
      *p += pt;
    }
    self
  }

  /// Rotate a Vec<Pt2> by rotating each Pt2
  fn rotate(&mut self, degrees: f64) -> &mut Self {
    for p in self.iter_mut() {
      p.rotate(degrees);
    }
    self
  }
}

/// A 2D point.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pt2 {
  pub x: f64,
  pub y: f64,
}

impl std::fmt::Display for Pt2 {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    write!(f, "[ {}, {} ]", self.x, self.y)
  }
}

impl std::ops::Index<usize> for Pt2 {
  type Output = f64;

  fn index(&self, index: usize) -> &Self::Output {
    match index {
      0 => &self.x,
      1 => &self.y,
      _ => panic!("Index {} is out of bounds.", index),
    }
  }
}

impl std::ops::IndexMut<usize> for Pt2 {
  fn index_mut(&mut self, index: usize) -> &mut Self::Output {
    match index {
      0 => &mut self.x,
      1 => &mut self.y,
      _ => panic!("Index {} is out of bounds.", index),
    }
  }
}

impl std::ops::Add for Pt2 {
  type Output = Self;

  fn add(self, rhs: Self) -> Self::Output {
    Self::new(self.x + rhs.x, self.y + rhs.y)
  }
}

impl std::ops::AddAssign for Pt2 {
  fn add_assign(&mut self, rhs: Self) {
    *self = *self + rhs;
  }
}

impl std::ops::Sub for Pt2 {
  type Output = Self;

  fn sub(self, rhs: Self) -> Self::Output {
    Self::new(self.x - rhs.x, self.y - rhs.y)
  }
}

impl std::ops::SubAssign for Pt2 {
  fn sub_assign(&mut self, rhs: Self) {
    *self = *self - rhs;
  }
}

impl std::ops::Mul<f64> for Pt2 {
  type Output = Self;

  fn mul(self, rhs: f64) -> Self::Output {
    Self::new(self.x * rhs, self.y * rhs)
  }
}

impl std::ops::MulAssign<f64> for Pt2 {
  fn mul_assign(&mut self, rhs: f64) {
    *self = *self * rhs;
  }
}

impl std::ops::Div<f64> for Pt2 {
  type Output = Self;

  fn div(self, rhs: f64) -> Self::Output {
    Self::new(self.x / rhs, self.y / rhs)
  }
}

impl std::ops::DivAssign<f64> for Pt2 {
  fn div_assign(&mut self, rhs: f64) {
    *self = *self / rhs;
  }
}

impl std::ops::Neg for Pt2 {
  type Output = Self;

  fn neg(self) -> Self::Output {
    self * -1.0
  }
}

impl Pt2 {
  pub fn new(x: f64, y: f64) -> Self {
    Self { x, y }
  }

  pub fn dot(self, rhs: Pt2) -> f64 {
    self.x * rhs.x + self.y * rhs.y
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
    Self::new(self.x / l, self.y / l)
  }

  pub fn rotate(&mut self, degrees: f64) {
    *self = self.rotated(degrees);
  }

  pub fn rotated(self, degrees: f64) -> Self {
    let c = dcos(degrees);
    let s = dsin(degrees);
    Self::new(self.x * c - self.y * s, self.x * s + self.y * c)
  }

  pub fn lerp(self, b: Self, t: f64) -> Self {
    self + (b - self) * t
  }

  pub fn to_xz(self) -> Pt3 {
    Pt3::new(self.x, 0.0, self.y)
  }

  pub fn as_pt3(self, z: f64) -> Pt3 {
    Pt3::new(self.x, self.y, z)
  }

  pub fn as_pt4(self, z: f64, w: f64) -> Pt4 {
    Pt4::new(self.x, self.y, z, w)
  }

  pub fn arc(start: Self, degrees: f64, segments: usize) -> Vec<Self> {
    assert!(degrees <= 360.0);
    let n_pts = if degrees == 360.0 {
      segments
    } else {
      segments + 1
    };
    let mut pts = Vec::with_capacity(n_pts);
    for i in 0..n_pts {
      let a = i as f64 * degrees / segments as f64;
      pts.push(start.rotated(a));
    }
    pts
  }

  pub fn rounded_rect(
    width: f64,
    height: f64,
    radius: f64,
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
      let point = Self::new(mt.f64_minmax(-1.0, 1.0), mt.f64_minmax(-1.0, 1.0));
      if point.x * point.x + point.y * point.y < length * length {
        return point;
      }
    }
  }

  /// radius: the radius of the circle surrounding the polygon
  pub fn inscribed_polygon(n_sides: usize, radius: f64) -> Vec<Self> {
    let angle = 360.0 / n_sides as f64;
    let mut points = Vec::new();
    for i in 0..n_sides {
      points.push(Pt2::new(
        dcos(angle * i as f64) * radius,
        dsin(angle * i as f64) * radius,
      ));
    }
    points
  }

  /// radius: the radius of the circle inside the polygon
  pub fn circumscribed_polygon(n_sides: usize, radius: f64) -> Vec<Self> {
    let radius = radius / dcos(180.0 / n_sides as f64);
    Self::inscribed_polygon(n_sides, radius)
  }

  pub fn star(n_points: usize, inner_radius: f64, outer_radius: f64) -> Vec<Self> {
    let angle = 360.0 / n_points as f64;
    let mut points = Vec::new();
    for i in 0..n_points {
      points.push(Pt2::new(
        dcos(angle * i as f64) * inner_radius,
        dsin(angle * i as f64) * inner_radius,
      ));
      points.push(Pt2::new(
        dcos(angle * (i as f64 + 0.5)) * outer_radius,
        dsin(angle * (i as f64 + 0.5)) * outer_radius,
      ));
    }
    points
  }

  pub fn bezier_star(
    n_points: usize,
    inner_radius: f64,
    inner_handle_length: f64,
    outer_radius: f64,
    outer_handle_length: f64,
    segments: usize,
  ) -> Vec<Self> {
    let mut controls = Vec::new();
    let mut knots = Vec::new();

    let angle = 360.0 / n_points as f64;
    for i in 0..n_points {
      knots.push(Pt2::new(
        dcos(angle * i as f64) * outer_radius,
        dsin(angle * i as f64) * outer_radius,
      ));
      knots.push(Pt2::new(
        dcos(angle * (i as f64 + 0.5)) * inner_radius,
        dsin(angle * (i as f64 + 0.5)) * inner_radius,
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

    let mut chain = CubicBezierChain2D::new(knots[0], controls[0], controls[0], knots[1]);
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
  pub fn chamfer(size: f64, oversize: f64) -> Vec<Self> {
    vec![
      Self::new(0.0, 0.0),
      Self::new(oversize + size, 0.0),
      Self::new(size + oversize, oversize),
      Self::new(size, oversize),
      Self::new(oversize, size),
      Self::new(oversize, size + oversize),
      Self::new(0.0, size + oversize),
    ]
  }
}

#[derive(Clone, Copy)]
struct CubicBezier2D {
  start: Pt2,
  control1: Pt2,
  control2: Pt2,
  end: Pt2,
}

#[derive(Clone)]
pub struct CubicBezierChain2D {
  curves: Vec<CubicBezier2D>,
  closed: bool,
}

impl CubicBezierChain2D {
  pub fn new(start: Pt2, control1: Pt2, control2: Pt2, end: Pt2) -> Self {
    Self {
      curves: vec![CubicBezier2D {
        start,
        control1,
        control2,
        end,
      }],
      closed: false,
    }
  }

  pub fn add(&mut self, control1_length: f64, control2: Pt2, end: Pt2) -> &mut Self {
    let chain_end = &self.curves[self.curves.len() - 1];
    self.curves.push(CubicBezier2D {
      start: chain_end.end,
      control1: chain_end.end + (chain_end.end - chain_end.control2).normalized() * control1_length,
      control2: control2,
      end: end,
    });
    self
  }

  pub fn close(&mut self, control1_length: f64, control2: Pt2, start_control1_len: f64) {
    self.closed = true;
    self.add(control1_length, control2, self.curves[0].start);
    let chain_end = &self.curves[self.curves.len() - 1];
    self.curves[0].control1 =
      chain_end.end + (chain_end.end - chain_end.control2).normalized() * start_control1_len;
  }

  pub fn gen_points(&self, pts_per_section: usize) -> Vec<Pt2> {
    let mut pts = vec![Pt2::new(0.0, 0.0)];
    for i in 0..self.curves.len() {
      pts.pop();
      pts.append(&mut Pt2::cubic_bezier(
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

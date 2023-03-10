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

//! Triangles are the building blocks of meshes.

use crate::Pt3;

#[derive(Clone, Copy)]
pub struct Triangle {
  pub a: Pt3,
  pub b: Pt3,
  pub c: Pt3,
}

impl Triangle {
  pub fn new(a: Pt3, b: Pt3, c: Pt3) -> Self {
    Self { a, b, c }
  }

  pub fn normal(&self) -> Pt3 {
    (self.b - self.a).cross(self.c - self.a)
  }

  pub fn translate(&mut self, p: Pt3) {
    self.a += p;
    self.b += p;
    self.c += p;
  }

  pub fn rotate_x(&mut self, degrees: f64) {
    self.a.rotate_x(degrees);
    self.b.rotate_x(degrees);
    self.c.rotate_x(degrees);
  }

  pub fn rotate_y(&mut self, degrees: f64) {
    self.a.rotate_y(degrees);
    self.b.rotate_y(degrees);
    self.c.rotate_y(degrees);
  }

  pub fn rotate_z(&mut self, degrees: f64) {
    self.a.rotate_z(degrees);
    self.b.rotate_z(degrees);
    self.c.rotate_z(degrees);
  }
}

pub trait VecTriangle {
  fn translate(&mut self, p: Pt3) -> &mut Self;
  fn rotate_x(&mut self, degrees: f64) -> &mut Self;
  fn rotate_y(&mut self, degrees: f64) -> &mut Self;
  fn rotate_z(&mut self, degrees: f64) -> &mut Self;
}

impl VecTriangle for Vec<Triangle> {
  fn translate(&mut self, p: Pt3) -> &mut Self {
    for t in self.iter_mut() {
      t.translate(p);
    }
    self
  }

  fn rotate_x(&mut self, degrees: f64) -> &mut Self {
    for t in self.iter_mut() {
      t.rotate_x(degrees);
    }
    self
  }

  fn rotate_y(&mut self, degrees: f64) -> &mut Self {
    for t in self.iter_mut() {
      t.rotate_y(degrees);
    }
    self
  }

  fn rotate_z(&mut self, degrees: f64) -> &mut Self {
    for t in self.iter_mut() {
      t.rotate_z(degrees);
    }
    self
  }
}

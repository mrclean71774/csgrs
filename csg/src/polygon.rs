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

//! A polygon part of https://github.com/timknip/pycsg port.

use {crate::plane::Plane, math::pt3::Pt3};

#[derive(Clone)]
pub struct Polygon {
  pub vertices: Vec<Pt3>,
  pub plane: Plane,
}

impl Polygon {
  pub fn new(vertices: Vec<Pt3>) -> Self {
    let plane = Plane::from_points(vertices[0], vertices[1], vertices[2]);
    Self { vertices, plane }
  }

  pub fn flip(&mut self) {
    let n_verts = self.vertices.len();
    let mut reversed = Vec::with_capacity(n_verts);
    for i in 0..n_verts {
      reversed.push(self.vertices[n_verts - 1 - i]);
    }
    self.vertices = reversed;
    self.plane.flip();
  }
}

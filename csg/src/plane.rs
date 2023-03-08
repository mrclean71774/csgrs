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

use {crate::polygon::Polygon, math::pt3::Pt3};

#[derive(Clone, Copy)]
pub struct Plane {
  pub normal: Pt3,
  pub w: f64,
}

impl Plane {
  pub fn new(normal: Pt3, w: f64) -> Self {
    Self { normal, w }
  }

  pub fn from_points(a: Pt3, b: Pt3, c: Pt3) -> Self {
    let n = (b - a).cross(c - a).normalized();
    Self::new(n, n.dot(a))
  }

  pub fn flip(&mut self) {
    self.normal = -self.normal;
    self.w = -self.w;
  }

  pub fn split_polygon(
    &self,
    polygon: &Polygon,
    coplanar_front: *mut Vec<Polygon>,
    coplanar_back: *mut Vec<Polygon>,
    front: *mut Vec<Polygon>,
    back: *mut Vec<Polygon>,
  ) {
    const EPSILON: f64 = 1.0e-5;
    const COPLANAR: u32 = 0;
    const FRONT: u32 = 1;
    const BACK: u32 = 2;
    const SPANNING: u32 = 3;

    let mut polygon_type = 0;
    let n_vertices = polygon.vertices.len();
    let mut vertex_locs = Vec::with_capacity(n_vertices);
    for i in 0..n_vertices {
      let t = self.normal.dot(polygon.vertices[i]) - self.w;
      let mut loc = COPLANAR;
      if t < -EPSILON {
        loc = BACK;
      } else if t > EPSILON {
        loc = FRONT;
      }
      polygon_type |= loc;
      vertex_locs.push(loc);
    }

    if polygon_type == COPLANAR {
      if self.normal.dot(polygon.plane.normal) > 0.0 {
        unsafe {
          (*coplanar_front).push(polygon.clone());
        }
      } else {
        unsafe {
          (*coplanar_back).push(polygon.clone());
        }
      }
    } else if polygon_type == FRONT {
      unsafe {
        (*front).push(polygon.clone());
      }
    } else if polygon_type == BACK {
      unsafe {
        (*back).push(polygon.clone());
      }
    } else if polygon_type == SPANNING {
      let mut f = Vec::new();
      let mut b = Vec::new();
      for i in 0..n_vertices {
        let j = (i + 1) % n_vertices;
        let ti = vertex_locs[i];
        let tj = vertex_locs[j];
        let vi = polygon.vertices[i];
        let vj = polygon.vertices[j];
        if ti != BACK {
          f.push(vi);
        }
        if ti != FRONT {
          b.push(vi);
        }
        if (ti | tj) == SPANNING {
          let t = (self.w - self.normal.dot(vi)) / self.normal.dot(vj - vi);
          let v = vi.lerp(vj, t);
          f.push(v);
          b.push(v);
        }
      }
      if f.len() >= 3 {
        unsafe { (*front).push(Polygon::new(f)) };
      }
      if b.len() >= 3 {
        unsafe { (*back).push(Polygon::new(b)) };
      }
    }
  }
}

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

use {
  crate::mesh::Mesh,
  math::{mt4::Mt4, pt2::Pt2, pt3::Pt3},
};

pub struct Viewer {
  vert_size: f64,
  edge_size: f64,
  segments: usize,
  verts: Vec<Pt3>,
  edges: Vec<(Pt3, Pt3)>,
}

impl Viewer {
  pub fn new(vert_size: f64, edge_size: f64, segments: usize) -> Self {
    Self {
      vert_size,
      edge_size,
      segments,
      verts: Vec::new(),
      edges: Vec::new(),
    }
  }

  pub fn add_edge(&mut self, edge: (Pt3, Pt3)) {
    self.edges.push(edge);
  }

  pub fn add_vert(&mut self, vert: Pt3) {
    self.verts.push(vert);
  }

  pub fn add_verts(&mut self, verts: &Vec<Pt3>) {
    for vert in verts {
      self.verts.push(*vert);
    }
  }

  pub fn add_vert2s(&mut self, verts: &Vec<Pt2>) {
    for vert in verts {
      self.verts.push(vert.as_pt3(0.0));
    }
  }

  pub fn render(&self) -> Mesh {
    let mut mesh = Mesh {
      triangles: Vec::new(),
    };
    for vert in &self.verts {
      let mut s = Mesh::sphere(self.vert_size / 2.0, self.segments);
      s.translate(*vert);

      mesh.triangles.append(&mut s.triangles);
    }
    for edge in &self.edges {
      let m = Mt4::look_at_matrix_lh(edge.0, edge.1, Pt3::new(0.0, 0.0, 1.0));
      let mut c = Mesh::cylinder(
        self.edge_size / 2.0,
        self.edge_size / 2.0,
        (edge.1 - edge.0).len(),
        self.segments,
        false,
      );
      for tri in &mut c.triangles {
        tri.a = (m * tri.a.as_pt4(1.0)).as_pt3();
        tri.a += edge.0;
        tri.b = (m * tri.b.as_pt4(1.0)).as_pt3();
        tri.b += edge.0;
        tri.c = (m * tri.c.as_pt4(1.0)).as_pt3();
        tri.c += edge.0;
      }
      mesh.triangles.append(&mut c.triangles);
    }

    mesh
  }
}

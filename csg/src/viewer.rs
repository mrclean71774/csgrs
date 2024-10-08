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

//! A viewer is a way to visualize points and edges.
//!
//! It is used to create a sphere at each vertex/point stored and a cylinder
//! between pairs of points stored for edges. With the viewer you can see what
//! a 2D profile or bezier curve looks like before extrusion etc. Viewer does
//! not do boolean operations on the added points and meshes it just adds more
//! triangles to the mesh.

use crate::{Mesh, Mt4, Pt2, Pt3};

pub struct Viewer {
  point_radius: f64,
  edge_radius: f64,
  segments: usize,
  points: Vec<Pt3>,
  edges: Vec<(Pt3, Pt3)>,
}

impl Viewer {
  /// Create a new viewer.
  ///
  /// vert_radius: The radius of vertex points.
  ///
  /// edge_radius: The radius of an edge.
  ///
  /// segments: The number of segments in a circle.
  ///
  /// return: The viewer.
  pub fn new(point_radius: f64, edge_radius: f64, segments: usize) -> Self {
    Self {
      point_radius,
      edge_radius,
      segments,
      points: Vec::new(),
      edges: Vec::new(),
    }
  }

  /// Add an edge to the viewer.
  ///
  /// edge: The points that define the edge.
  pub fn add_edge(&mut self, edge: (Pt3, Pt3)) {
    self.edges.push(edge);
  }

  /// Add an array of edges to the viewer.
  ///
  /// edges: The array of edges.
  pub fn add_edges(&mut self, mut edges: Vec<(Pt3, Pt3)>) {
    self.edges.append(&mut edges);
  }

  /// Add a vertex to the viewer.
  ///
  /// vert: The vertex to add.
  pub fn add_pt3(&mut self, point: Pt3) {
    self.points.push(point);
  }

  /// Add an array of vertices to the viewer.
  ///
  /// verts: The array of vertices.
  pub fn add_pt3s(&mut self, mut points: Vec<Pt3>) {
    self.points.append(&mut points);
  }

  /// Add a point to the viewer.
  ///
  /// verts: The array of 2D vertices.
  pub fn add_pt2(&mut self, point: Pt2) {
    self.add_pt3(point.as_pt3(0.0));
  }

  /// Add an array of 2D points to the viewer.
  ///
  /// points: The array of 2D points.
  pub fn add_pt2s(&mut self, points: Vec<Pt2>) {
    for point in points {
      self.add_pt2(point);
    }
  }

  /// Render the points and edges stored in viewer to a mesh.
  ///
  /// return: A mesh containing all the points and edges.
  pub fn render(&self) -> Mesh {
    let mut mesh = Mesh {
      triangles: Vec::new(),
    };
    for point in &self.points {
      let mut s = Mesh::sphere(self.point_radius, self.segments);
      s.translate(*point);

      mesh.triangles.append(&mut s.triangles);
    }
    for edge in &self.edges {
      let m = Mt4::look_at_matrix_lh(edge.0, edge.1, Pt3::new(0.0, 0.0, 1.0));
      let mut c = Mesh::cylinder(
        self.edge_radius,
        self.edge_radius,
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

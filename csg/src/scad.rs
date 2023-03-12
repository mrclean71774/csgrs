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

//! An OpenSCAD backend as an alternative to the CSG booleans.

// NOTE: OpenSCAD uses clockwise winding order.

use {
  crate::mesh::Mesh,
  math::pt3::{Pt3, VecPt3},
  std::io::Write,
};

/// The three boolean operations in OpenSCAD
#[derive(Clone, Copy)]
enum BoolOp {
  Union,
  Difference,
  Intersection,
}

/// SCAD is a binary tree. Each node is either a "mesh" or an operation with two children.
#[derive(Clone)]
pub struct SCAD {
  vertices: Vec<Pt3>,
  indices: Vec<usize>,
  op: Option<BoolOp>,
  children: Vec<SCAD>,
}

impl SCAD {
  fn from_verts_and_index(vertices: Vec<Pt3>, indices: Vec<usize>) -> Self {
    Self {
      vertices,
      indices,
      op: None,
      children: Vec::new(),
    }
  }

  /// Create a SCAD from a Mesh.
  ///
  /// NOTE: It's usually more ergonomic to call mesh.into_scad() instead of using this function directly.
  pub fn from_mesh(mesh: Mesh) -> Self {
    let n_triangles = mesh.triangles.len();
    let mut vertices = Vec::new();
    let mut indices = Vec::with_capacity(n_triangles * 3);

    let mut index = 0;
    for triangle in mesh.triangles {
      // We need to flip the winding order as we transition to vertices and indices.
      let mut c_found = false;
      let mut c_index = 0;
      let mut b_found = false;
      let mut b_index = 0;
      let mut a_found = false;
      let mut a_index = 0;

      for (i, vertex) in vertices.iter().enumerate() {
        if *vertex == triangle.c {
          c_found = true;
          c_index = i;
        }
        if *vertex == triangle.b {
          b_found = true;
          b_index = i;
        }
        if *vertex == triangle.a {
          a_found = true;
          a_index = i;
        }
      }

      if c_found {
        indices.push(c_index);
      } else {
        vertices.push(triangle.c);
        indices.push(index);
        index += 1;
      }
      if b_found {
        indices.push(b_index);
      } else {
        vertices.push(triangle.b);
        indices.push(index);
        index += 1;
      }
      if a_found {
        indices.push(a_index);
      } else {
        vertices.push(triangle.a);
        indices.push(index);
        index += 1;
      }
    }
    println!("{} {}", vertices.len(), indices.len());

    Self::from_verts_and_index(vertices, indices)
  }

  pub fn translate(&mut self, v: Pt3) {
    if self.vertices.len() == 0 {
      self.children[0].translate(v);
      self.children[1].translate(v);
    } else {
      self.vertices.translate(v);
    }
  }

  pub fn rotate_x(&mut self, degrees: f64) {
    if self.vertices.len() == 0 {
      self.children[0].rotate_x(degrees);
      self.children[1].rotate_x(degrees);
    } else {
      self.vertices.rotate_x(degrees);
    }
  }

  pub fn rotate_y(&mut self, degrees: f64) {
    if self.vertices.len() == 0 {
      self.children[0].rotate_y(degrees);
      self.children[1].rotate_y(degrees);
    } else {
      self.vertices.rotate_y(degrees);
    }
  }

  pub fn rotate_z(&mut self, degrees: f64) {
    if self.vertices.len() == 0 {
      self.children[0].rotate_z(degrees);
      self.children[1].rotate_z(degrees);
    } else {
      self.vertices.rotate_z(degrees);
    }
  }

  fn is_valid(&self) -> bool {
    if self.vertices.len() > 0 && self.op.is_some() {
      false
    } else if self.vertices.len() == 0 && self.op.is_none() {
      false
    } else {
      true
    }
  }

  pub fn save(&self, path: &str) {
    let s = format!("{}", self);
    let mut file = std::fs::File::create(path).unwrap();
    file.write(s.as_bytes()).unwrap();
    file.flush().unwrap();
  }
}

impl std::fmt::Display for SCAD {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    assert!(self.is_valid());
    if self.op.is_some() {
      match self.op.unwrap() {
        BoolOp::Union => {
          write!(f, "union() {{\n")?;
        }
        BoolOp::Difference => {
          write!(f, "difference() {{\n")?;
        }
        BoolOp::Intersection => {
          write!(f, "intersection() {{\n")?;
        }
      }
      write!(f, "{}\n{}\n}}\n", self.children[0], self.children[1])
    } else {
      write!(f, "polyhedron(\npoints=[")?;
      let n_verts_with_comma = self.vertices.len() - 1;
      for vert_index in 0..n_verts_with_comma {
        write!(
          f,
          "[{},{},{}],",
          self.vertices[vert_index].x, self.vertices[vert_index].y, self.vertices[vert_index].z
        )?;
      }
      write!(
        f,
        "[{},{},{}]],\nfaces=[",
        self.vertices[n_verts_with_comma].x,
        self.vertices[n_verts_with_comma].y,
        self.vertices[n_verts_with_comma].z
      )?;
      let n_indices_with_comma = self.indices.len() / 3 - 1;
      for indices_index in 0..n_indices_with_comma {
        write!(f, "[")?;
        let n_inner_with_comma = 2;
        for inner_index in 0..n_inner_with_comma {
          write!(f, "{},", self.indices[indices_index * 3 + inner_index])?;
        }
        write!(
          f,
          "{}],",
          self.indices[indices_index * 3 + n_inner_with_comma]
        )?;
      }
      write!(f, "[")?;
      let n_inner_with_comma = 2;
      for inner_index in 0..n_inner_with_comma {
        write!(
          f,
          "{},",
          self.indices[n_indices_with_comma * 3 + inner_index]
        )?;
      }
      write!(
        f,
        "{}]]\n);",
        self.indices[n_indices_with_comma * 3 + n_inner_with_comma]
      )
    }
  }
}

impl std::ops::Sub for SCAD {
  type Output = Self;

  fn sub(self, rhs: Self) -> Self::Output {
    Self {
      vertices: Vec::new(),
      indices: Vec::new(),
      op: Some(BoolOp::Difference),
      children: vec![self, rhs],
    }
  }
}

impl std::ops::Add for SCAD {
  type Output = Self;

  fn add(self, rhs: Self) -> Self::Output {
    Self {
      vertices: Vec::new(),
      indices: Vec::new(),
      op: Some(BoolOp::Union),
      children: vec![self, rhs],
    }
  }
}

impl std::ops::Mul for SCAD {
  type Output = Self;

  fn mul(self, rhs: Self) -> Self::Output {
    Self {
      vertices: Vec::new(),
      indices: Vec::new(),
      op: Some(BoolOp::Intersection),
      children: vec![self, rhs],
    }
  }
}

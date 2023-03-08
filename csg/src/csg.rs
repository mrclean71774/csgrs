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
  crate::{bsp_node::BSPNode, mesh::Mesh, polygon::Polygon, triangle::Triangle},
  math::pt3::Pt3,
};

#[derive(Clone)]
pub struct CSG {
  pub polygons: Vec<Polygon>,
}

impl CSG {
  pub fn new() -> Self {
    Self {
      polygons: Vec::new(),
    }
  }

  pub fn from_mesh(mesh: Mesh) -> Self {
    Self::from_triangles(mesh.triangles)
  }

  pub fn from_triangles(triangles: Vec<Triangle>) -> Self {
    let mut polygons: Vec<Polygon> = Vec::with_capacity(triangles.len());
    for triangle in triangles {
      let vertices = vec![triangle.a, triangle.b, triangle.c];
      let polygon = Polygon::new(vertices);
      polygons.push(polygon);
    }
    Self { polygons }
  }

  pub fn into_triangles(self) -> Vec<Triangle> {
    let mut triangles: Vec<Triangle> = Vec::new();
    for polygon in self.polygons {
      for i in 1..(polygon.vertices.len() - 1) {
        triangles.push(Triangle::new(
          polygon.vertices[0],
          polygon.vertices[i],
          polygon.vertices[i + 1],
        ));
      }
    }
    triangles
  }

  pub fn refine(&mut self) -> Self {
    let mut new_csg = CSG::new();
    for poly in &mut self.polygons {
      let verts = &mut poly.vertices;
      let num_verts = verts.len();
      if num_verts == 0 {
        continue;
      }
      let mut mid_pos = Pt3::new(0.0, 0.0, 0.0);
      for i in 0..num_verts {
        mid_pos += verts[i];
      }
      mid_pos /= num_verts as f64;

      let mut new_verts = Vec::new();
      for i in 0..num_verts {
        new_verts.push(verts[i].lerp(verts[(i + 1) % num_verts], 0.5));
      }
      new_verts.push(mid_pos);
      verts.append(&mut new_verts);

      let i = 0;
      let vs = vec![
        new_verts[i],
        new_verts[i + num_verts],
        new_verts[2 * num_verts],
        new_verts[2 * num_verts - 1],
      ];
      let mut new_poly = Polygon::new(vs);
      new_poly.plane = poly.plane;
      new_csg.polygons.push(new_poly);

      for i in 1..num_verts {
        let vs = vec![
          new_verts[i],
          new_verts[i + num_verts],
          new_verts[2 * num_verts],
          new_verts[num_verts + i - 1],
        ];
        new_poly = Polygon::new(vs);
        new_csg.polygons.push(new_poly);
      }
    }
    new_csg
  }

  pub fn translate(&mut self, displacement: Pt3) {
    for poly in &mut self.polygons {
      for v in &mut poly.vertices {
        *v += displacement;
      }
    }
  }

  pub fn union(&self, csg: CSG) -> CSG {
    let mut a = Box::new(BSPNode::new(Some(self.clone().polygons)));
    let mut b = Box::new(BSPNode::new(Some(csg.clone().polygons)));
    a.clip_to(&mut b);
    b.clip_to(&mut a);
    b.invert();
    b.clip_to(&mut a);
    b.invert();
    a.build(b.all_polygons());
    CSG {
      polygons: a.all_polygons(),
    }
  }
}

impl std::ops::Add<CSG> for CSG {
  type Output = Self;

  fn add(self, rhs: CSG) -> Self::Output {
    self.union(rhs)
  }
}

impl CSG {
  pub fn subtract(&self, csg: CSG) -> Self {
    let mut a = Box::new(BSPNode::new(Some(self.clone().polygons)));
    let mut b = Box::new(BSPNode::new(Some(csg.clone().polygons)));
    a.invert();
    a.clip_to(&mut b);
    b.clip_to(&mut a);
    b.invert();
    b.clip_to(&mut a);
    b.invert();
    a.build(b.all_polygons());
    a.invert();
    CSG {
      polygons: a.all_polygons(),
    }
  }
}

impl std::ops::Sub<CSG> for CSG {
  type Output = CSG;

  fn sub(self, rhs: CSG) -> Self::Output {
    self.subtract(rhs)
  }
}

impl CSG {
  pub fn intersect(&self, csg: CSG) -> Self {
    let mut a = Box::new(BSPNode::new(Some(self.clone().polygons)));
    let mut b = Box::new(BSPNode::new(Some(csg.clone().polygons)));
    a.invert();
    b.clip_to(&mut a);
    b.invert();
    a.clip_to(&mut b);
    b.clip_to(&mut a);
    a.build(b.all_polygons());
    a.invert();
    CSG {
      polygons: a.all_polygons(),
    }
  }
}

impl std::ops::Mul<CSG> for CSG {
  type Output = Self;

  fn mul(self, rhs: CSG) -> Self::Output {
    self.intersect(rhs)
  }
}

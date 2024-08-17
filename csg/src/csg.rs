// MIT License
//
// Copyright (c) 2023 Michael H. Phillips
// Copyright (c) 2015 Alexander Pletzer
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

//! Constructive Solid Geometry part of https://github.com/timknip/pycsg port

use crate::{Mesh, Pt3, Triangle};

#[derive(Clone, Default)]
pub struct CSG {
  pub polygons: Vec<Polygon>,
}

impl CSG {
  pub fn new() -> Self {
    Self::default()
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
      for vert in verts.iter() {
        mid_pos += *vert;
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

pub struct BSPNode {
  plane: Option<Plane>,
  front: Option<Box<BSPNode>>,
  back: Option<Box<BSPNode>>,
  polygons: Vec<Polygon>,
}

impl BSPNode {
  pub fn new(polygons: Option<Vec<Polygon>>) -> Self {
    let mut node = Self {
      plane: None,
      front: None,
      back: None,
      polygons: Vec::new(),
    };
    if let Some(polygons) = polygons {
      node.build(polygons);
    }
    node
  }

  pub fn invert(&mut self) {
    for poly in &mut self.polygons {
      poly.flip();
    }
    if self.plane.is_some() {
      self.plane.as_mut().unwrap().flip();
    }
    if self.front.is_some() {
      self.front.as_mut().unwrap().invert();
    }
    if self.back.is_some() {
      self.back.as_mut().unwrap().invert();
    }
    std::mem::swap(&mut self.front, &mut self.back);
  }

  pub fn clip_polygons(&mut self, polygons: Vec<Polygon>) -> Vec<Polygon> {
    if self.plane.is_none() {
      return polygons;
    }
    let mut front: Vec<Polygon> = Vec::new();
    let mut back: Vec<Polygon> = Vec::new();
    for poly in polygons {
      self
        .plane
        .unwrap()
        .split_polygon(&poly, &mut front, &mut back, &mut front, &mut back)
    }
    if self.front.is_some() {
      front = self.front.as_mut().unwrap().clip_polygons(front);
    }
    if self.back.is_some() {
      back = self.back.as_mut().unwrap().clip_polygons(back);
    } else {
      back = Vec::new();
    }
    front.append(&mut back);

    front
  }

  pub fn clip_to(&mut self, bsp: &mut Box<BSPNode>) {
    self.polygons = bsp.clip_polygons(self.polygons.clone());
    if self.front.is_some() {
      self.front.as_mut().unwrap().clip_to(bsp)
    }
    if self.back.is_some() {
      self.back.as_mut().unwrap().clip_to(bsp)
    }
  }

  pub fn all_polygons(&self) -> Vec<Polygon> {
    let mut polygons = self.polygons.clone();
    if self.front.is_some() {
      polygons.append(&mut self.front.as_ref().unwrap().all_polygons());
    }
    if self.back.is_some() {
      polygons.append(&mut self.back.as_ref().unwrap().all_polygons());
    }
    polygons
  }

  pub fn build(&mut self, polygons: Vec<Polygon>) {
    if polygons.is_empty() {
      return;
    }
    if self.plane.is_none() {
      self.plane = Some(polygons[0].plane);
    }
    self.polygons.push(polygons[0].clone());
    let mut front: Vec<Polygon> = Vec::new();
    let mut back: Vec<Polygon> = Vec::new();
    for polygon in polygons.iter().skip(1) {
      self.plane.as_mut().unwrap().split_polygon(
        polygon,
        &mut self.polygons,
        &mut self.polygons,
        &mut front,
        &mut back,
      );
    }
    if !front.is_empty() {
      if self.front.is_none() {
        self.front = Some(Box::new(BSPNode::new(None)));
      }
      self.front.as_mut().unwrap().build(front);
    }
    if !back.is_empty() {
      if self.back.is_none() {
        self.back = Some(Box::new(BSPNode::new(None)));
      }
      self.back.as_mut().unwrap().build(back);
    }
  }
}

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

  fn split_polygon(
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

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
  crate::{
    csg::CSG,
    triangle::{Triangle, VecTriangle},
  },
  math::{
    pt2::{Pt2, VecPt2},
    pt3::{Pt3, VecPt3},
  },
  std::io::{Read, Write},
};

#[derive(Clone)]
pub struct Mesh {
  pub triangles: Vec<Triangle>,
}

impl Mesh {
  pub fn from_triangles(triangles: Vec<Triangle>) -> Self {
    Self { triangles }
  }

  pub fn from_csg(csg: CSG) -> Self {
    Self::from_triangles(csg.into_triangles())
  }

  pub fn from_verts(vertices: &Vec<Pt3>, indices: &Vec<usize>) -> Self {
    assert!(indices.len() % 3 == 0);
    let mut triangles = Vec::with_capacity(indices.len() / 3);
    for i in (0..indices.len()).step_by(3) {
      triangles.push(Triangle::new(
        vertices[indices[i]],
        vertices[indices[i + 1]],
        vertices[indices[i + 2]],
      ));
    }
    Self::from_triangles(triangles)
  }

  pub fn cube(x: f64, y: f64, z: f64, center: bool) -> Self {
    let s = Pt3::new(x, y, z) / 2.0;
    let mut vertices = vec![
      Pt3::new(-s.x, -s.y, s.z),
      Pt3::new(s.x, -s.y, s.z),
      Pt3::new(-s.x, -s.y, -s.z),
      Pt3::new(s.x, -s.y, -s.z),
      Pt3::new(-s.x, s.y, s.z),
      Pt3::new(s.x, s.y, s.z),
      Pt3::new(-s.x, s.y, -s.z),
      Pt3::new(s.x, s.y, -s.z),
    ];
    let indices = vec![
      0, 2, 1, //
      1, 2, 3, //
      1, 3, 5, //
      5, 3, 7, //
      4, 0, 5, //
      5, 0, 1, //
      4, 5, 6, //
      6, 5, 7, //
      0, 4, 2, //
      2, 4, 6, //
      6, 7, 2, //
      2, 7, 3, //
    ];
    if !center {
      vertices.translate(s);
    }
    Self::from_verts(&vertices, &indices)
  }

  pub fn sphere(r: f64, segments: usize) -> Self {
    let points = Pt2::arc(Pt2::new(0.0, -r), 180.0, segments);
    Self::revolve(&points, segments)
  }

  /// Create an arrow!
  pub fn arrow(
    shaft_radius: f64,
    shaft_length: f64,
    head_radius: f64,
    head_start: f64,
    head_length: f64,
    segments: usize,
    center: bool,
  ) -> Self {
    let mut points_2d = Vec::new();
    points_2d.push(Pt2::new(0.0, 0.0));
    points_2d.push(Pt2::new(shaft_radius, 0.0));
    points_2d.push(Pt2::new(shaft_radius, shaft_length));
    points_2d.push(Pt2::new(shaft_radius + head_radius, head_start));
    points_2d.push(Pt2::new(0.0, shaft_length + head_length));
    if center {
      points_2d.translate(
        Pt2::new(
          0.0,
          -(shaft_length - (shaft_length - head_start) + head_length),
        ) / 2.0,
      );
    }
    Self::revolve(&points_2d, segments)
  }

  pub fn cylinder(r1: f64, r2: f64, height: f64, segments: usize, center: bool) -> Self {
    let mut result = Self::revolve(
      &vec![
        Pt2::new(0.0, -height / 2.0),
        Pt2::new(r1, -height / 2.0),
        Pt2::new(r2, height / 2.0),
        Pt2::new(0.0, height / 2.0),
      ],
      segments,
    );
    if !center {
      result.triangles.translate(Pt3::new(0.0, 0.0, height / 2.0));
    }
    result
  }

  /// Revolve a string of 2D points around the Z axis after
  /// mapping the 2D Y axis to the 3D Z axis.  This follows
  /// OpenSCAD's transition from 2D to 3D.  The points are
  /// assumed to start and end on the Z axis.  Who knows what
  /// happens if they are not!
  pub fn revolve(points: &Vec<Pt2>, segments: usize) -> Self {
    assert!(points.len() > 2);
    let stride = points.len() - 2;
    let mut vertices = Vec::new();
    vertices.push(points[0].to_xz());
    vertices.push(points[points.len() - 1].to_xz());
    for i in 1..(points.len() - 1) {
      vertices.push(points[i].to_xz());
    }
    let mut indices = Vec::new();
    for segment in 1..segments {
      for i in 1..(points.len() - 1) {
        let v = &points[i].to_xz();
        vertices.push(Pt3::new(
          v.x
            * (segment as f64 * 360.0 / segments as f64)
              .to_radians()
              .cos(),
          v.x
            * (segment as f64 * 360.0 / segments as f64)
              .to_radians()
              .sin(),
          v.z,
        ));
      }
      let index = segment * stride + 2;
      indices.append(&mut vec![index - stride, 0, index]);
      indices.append(&mut vec![index + stride - 1, 1, index - 1]);
      for i in 0..(stride - 1) {
        let p0 = index + i + 1 - stride;
        let p1 = index + i + 1;
        let p2 = index + i - stride;
        let p3 = index + i;
        indices.append(&mut vec![p0, p2, p1]);
        indices.append(&mut vec![p1, p2, p3]);
      }
    }
    for i in 0..(stride - 1) {
      let p0 = vertices.len() + i + 1 - stride;
      let p1 = i + 3;
      let p2 = vertices.len() + i - stride;
      let p3 = 2 + i;
      indices.append(&mut vec![p0, p2, p1]);
      indices.append(&mut vec![p1, p2, p3]);
    }
    indices.append(&mut vec![0, 2, vertices.len() - stride]);
    indices.append(&mut vec![1, vertices.len() - 1, stride + 1]);

    Self::from_verts(&vertices, &indices)
  }

  pub fn save_stl_bin(&self, path: &str) {
    let mut header: Vec<u8> = vec![0; 80];
    let mut v: Vec<u8> = Vec::new();
    v.append(&mut header);
    let mut n_triangles = (self.triangles.len() as u32).to_le_bytes().to_vec();
    v.append(&mut n_triangles);
    for triangle in &self.triangles {
      let normal = triangle.normal();

      let mut nx = (normal.x as f32).to_le_bytes().to_vec();
      v.append(&mut nx);
      let mut ny = (normal.y as f32).to_le_bytes().to_vec();
      v.append(&mut ny);
      let mut nz = (normal.z as f32).to_le_bytes().to_vec();
      v.append(&mut nz);

      let mut v1x = (triangle.a.x as f32).to_le_bytes().to_vec();
      v.append(&mut v1x);
      let mut v1y = (triangle.a.y as f32).to_le_bytes().to_vec();
      v.append(&mut v1y);
      let mut v1z = (triangle.a.z as f32).to_le_bytes().to_vec();
      v.append(&mut v1z);

      let mut v2x = (triangle.b.x as f32).to_le_bytes().to_vec();
      v.append(&mut v2x);
      let mut v2y = (triangle.b.y as f32).to_le_bytes().to_vec();
      v.append(&mut v2y);
      let mut v2z = (triangle.b.z as f32).to_le_bytes().to_vec();
      v.append(&mut v2z);

      let mut v3x = (triangle.c.x as f32).to_le_bytes().to_vec();
      v.append(&mut v3x);
      let mut v3y = (triangle.c.y as f32).to_le_bytes().to_vec();
      v.append(&mut v3y);
      let mut v3z = (triangle.c.z as f32).to_le_bytes().to_vec();
      v.append(&mut v3z);
      let mut space = 0u16.to_le_bytes().to_vec();
      v.append(&mut space);
    }
    let mut file = std::fs::File::create(path).unwrap();
    file.write_all(&mut v).unwrap();
    file.flush().unwrap();
  }

  pub fn save_stl_ascii(&self, path: &str) {
    let mut v: Vec<u8> = Vec::new();
    v.append(&mut b"solid ".to_vec());
    v.append(&mut path.as_bytes().to_vec());
    v.append(&mut b"\n".to_vec());
    for triangle in &self.triangles {
      let normal = triangle.normal();
      v.append(
        &mut format!(
          "facet normal {} {} {}\n",
          normal.x as f32, normal.y as f32, normal.z as f32
        )
        .as_bytes()
        .to_vec(),
      );
      v.append(&mut b"    outer loop\n".to_vec());
      v.append(
        &mut format!(
          "        vertex {} {} {}\n",
          triangle.a.x as f32, triangle.a.y as f32, triangle.a.z as f32
        )
        .as_bytes()
        .to_vec(),
      );
      v.append(
        &mut format!(
          "        vertex {} {} {}\n",
          triangle.b.x as f32, triangle.b.y as f32, triangle.b.z as f32
        )
        .as_bytes()
        .to_vec(),
      );
      v.append(
        &mut format!(
          "        vertex {} {} {}\n",
          triangle.c.x as f32, triangle.c.y as f32, triangle.c.z as f32
        )
        .as_bytes()
        .to_vec(),
      );
      v.append(&mut b"    endloop\n".to_vec());
      v.append(&mut b"endfacet\n".to_vec());
    }
    v.append(&mut b"endsolid ".to_vec());
    v.append(&mut path.as_bytes().to_vec());
    v.append(&mut b"\n".to_vec());

    let mut file = std::fs::File::create(path).unwrap();
    file.write_all(&mut v).unwrap();
    file.flush().unwrap();
  }

  pub fn load_stl(path: &str) -> Self {
    let mut file = std::fs::File::open(path).unwrap();
    let mut data = Vec::new();
    file.read_to_end(&mut data).unwrap();
    assert!(data.len() > 5);
    if data[0] == b's' && data[1] == b'o' && data[2] == b'l' && data[3] == b'i' && data[4] == b'd' {
      Self::parse_ascii(data)
    } else {
      Self::parse_binary(data)
    }
  }

  fn parse_binary(data: Vec<u8>) -> Self {
    let ptr = &data[80] as *const u8 as *const u32;
    let n_triangles = unsafe { *ptr };
    let mut triangles = Vec::with_capacity(n_triangles as usize);
    for i in 0..n_triangles as usize {
      let ptr = &data[84 + i * 50] as *const u8 as *const f32;
      //let normal;
      let vert1;
      let vert2;
      let vert3;
      unsafe {
        vert1 = Pt3::new(
          *ptr.offset(3) as f64,
          *ptr.offset(4) as f64,
          *ptr.offset(5) as f64,
        );
        vert2 = Pt3::new(
          *ptr.offset(6) as f64,
          *ptr.offset(7) as f64,
          *ptr.offset(8) as f64,
        );
        vert3 = Pt3::new(
          *ptr.offset(9) as f64,
          *ptr.offset(10) as f64,
          *ptr.offset(11) as f64,
        );
      }
      triangles.push(Triangle::new(vert1, vert2, vert3));
    }
    Self::from_triangles(triangles)
  }

  fn parse_ascii(_data: Vec<u8>) -> Self {
    panic!("Loading ascii stl files is not implemented.")
  }
}

impl std::ops::Add for Mesh {
  type Output = Self;

  fn add(self, rhs: Self) -> Self::Output {
    let a = CSG::from_mesh(self);
    let b = CSG::from_mesh(rhs);
    Mesh::from_csg(a + b)
  }
}

impl std::ops::AddAssign for Mesh {
  fn add_assign(&mut self, rhs: Self) {
    *self = self.clone() + rhs;
  }
}

impl std::ops::Sub for Mesh {
  type Output = Self;

  fn sub(self, rhs: Self) -> Self::Output {
    let a = CSG::from_mesh(self);
    let b = CSG::from_mesh(rhs);
    Mesh::from_csg(a - b)
  }
}

impl std::ops::SubAssign for Mesh {
  fn sub_assign(&mut self, rhs: Self) {
    *self = self.clone() - rhs;
  }
}

impl std::ops::Mul for Mesh {
  type Output = Self;

  fn mul(self, rhs: Self) -> Self::Output {
    let a = CSG::from_mesh(self);
    let b = CSG::from_mesh(rhs);
    Mesh::from_csg(a * b)
  }
}

impl std::ops::MulAssign for Mesh {
  fn mul_assign(&mut self, rhs: Self) {
    *self = self.clone() * rhs
  }
}

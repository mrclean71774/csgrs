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
  crate::{csg::CSG, triangle::Triangle},
  math::pt3::{Pt3, VecPt3},
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

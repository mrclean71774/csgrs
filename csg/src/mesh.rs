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
    ear_clip::triangulate3d,
    triangle::{Triangle, VecTriangle},
  },
  math::{
    dcos, dsin,
    mt4::Mt4,
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

  pub fn translate(&mut self, pt: Pt3) -> &mut Self {
    self.triangles.translate(pt);
    self
  }

  pub fn rotate_x(&mut self, degrees: f64) -> &mut Self {
    self.triangles.rotate_x(degrees);
    self
  }

  pub fn rotate_y(&mut self, degrees: f64) -> &mut Self {
    self.triangles.rotate_y(degrees);
    self
  }

  pub fn rotate_z(&mut self, degrees: f64) -> &mut Self {
    self.triangles.rotate_z(degrees);
    self
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

  pub fn chamfer(size: f64, length: f64, oversize: f64) -> Self {
    Self::linear_extrude(&Pt2::chamfer(size, oversize), length)
  }

  pub fn external_circle_chamfer(
    size: f64,
    radius: f64,
    degrees: f64,
    oversize: f64,
    segments: usize,
  ) -> Self {
    let mut points2 = Pt2::chamfer(size, oversize);
    points2
      .rotate(90.0)
      .translate(Pt2::new(radius + size / 2.0 + oversize / 2.0, -oversize));
    Self::rotate_extrude(&points2, degrees, segments)
  }

  pub fn external_cylinder_chamfer(
    size: f64,
    radius: f64,
    height: f64,
    oversize: f64,
    segments: usize,
    center: bool,
  ) -> Self {
    let mut result = Self::external_circle_chamfer(size, radius, 360.0, oversize, segments);
    let mut result1 = Self::external_circle_chamfer(size, radius, 360.0, oversize, segments);
    result1.rotate_x(180.0);
    result1.translate(Pt3::new(0.0, 0.0, height));
    if center {
      result.translate(Pt3::new(0.0, 0.0, -height / 2.0));
    }
    result + result1
  }

  pub fn linear_extrude(profile: &Vec<Pt2>, length: f64) -> Self {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let profile_len = profile.len();
    let mut profile3d = Vec::with_capacity(profile_len);

    for i in 0..profile_len {
      profile3d.push(profile[i].as_pt3(0.0));
      vertices.push(profile[profile_len - 1 - i].as_pt3(0.0));
    }
    indices.append(&mut triangulate3d(&vertices, Pt3::new(0.0, 0.0, -1.0)));
    for i in 0..profile_len {
      vertices.push(profile[i].as_pt3(length));
    }
    for p0 in 0..profile_len {
      let p1 = (p0 + 1) % profile_len;
      let p2 = profile_len + profile_len - 1 - p0;
      let p3 = profile_len + (profile_len - p0) % profile_len;
      indices.append(&mut vec![p3, p2, p0]);
      indices.append(&mut vec![p2, p1, p0]);
    }
    let mut indies = triangulate3d(&profile3d, Pt3::new(0.0, 0.0, 1.0));
    for indie in &mut indies {
      *indie += profile_len;
    }
    indices.append(&mut indies);
    Self::from_verts(&vertices, &indices)
  }

  pub fn linear_twist_extrude(
    profile: &Vec<Pt2>,
    length: f64,
    twist: f64,
    segments: usize,
  ) -> Self {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let profile: Vec<Pt3> = profile.iter().map(|p| p.as_pt3(0.0)).collect();
    let profile_len = profile.len();
    let profile_rev: Vec<Pt3> = profile.clone().into_iter().rev().collect();
    let z_step = length / segments as f64;
    let twist_a = twist / segments as f64;
    indices.append(&mut triangulate3d(&profile_rev, Pt3::new(0.0, 0.0, -1.0)));

    for i in 0..profile_len {
      vertices.push(profile_rev[i]);
    }

    for segment in 1..segments {
      for i in 0..profile_len {
        vertices.push(
          profile_rev[i].rotated_z(twist_a * segment as f64)
            + Pt3::new(0.0, 0.0, z_step * segment as f64),
        );
        let p0 = (segment - 1) * profile_len + i;
        let p1 = (segment - 1) * profile_len + ((i + 1) % profile_len);
        let p2 = segment * profile_len + i;
        let p3 = segment * profile_len + ((i + 1) % profile_len);
        indices.append(&mut vec![p3, p1, p0]);
        indices.append(&mut vec![p0, p2, p3]);
      }
    }

    for i in 0..profile_len {
      vertices.push(
        profile[i].rotated_z(twist_a * segments as f64)
          + Pt3::new(0.0, 0.0, z_step * segments as f64),
      );
    }
    for i in 0..profile_len {
      let p0 = (segments - 1) * profile_len + i;
      let p1 = (segments - 1) * profile_len + ((i + 1) % profile_len);
      let p2 = segments * profile_len + (profile_len - 1 - i);
      let p3 = segments * profile_len + (profile_len - 1 - ((i + 1) % profile_len));
      indices.append(&mut vec![p3, p1, p0]);
      indices.append(&mut vec![p0, p2, p3]);
    }
    let mut indies = triangulate3d(&profile, Pt3::new(0.0, 0.0, 1.0));
    for indie in &mut indies {
      *indie += vertices.len() - profile_len;
    }
    indices.append(&mut indies);

    Self::from_verts(&vertices, &indices)
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
          v.x * dcos(segment as f64 * 360.0 / segments as f64),
          v.x * dsin(segment as f64 * 360.0 / segments as f64),
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

  pub fn rotate_extrude(points: &Vec<Pt2>, angle: f64, segments: usize) -> Self {
    assert!(angle >= 0.0 && angle <= 360.0);
    assert!(segments >= 3);
    let points3d: Vec<Pt3> = points.iter().map(|p| Pt3::new(p.x, 0.0, p.y)).collect();
    let points3d_len = points3d.len();
    let a = angle / segments as f64;
    let mut vertices = points3d.clone();
    let mut indices = Vec::new();
    if angle != 360.0 {
      indices.append(&mut triangulate3d(&points3d, Pt3::new(0.0, -1.0, 0.0)));
    }
    for segment in 1..segments {
      let s = dsin(a * segment as f64);
      let c = dcos(a * segment as f64);
      for p in 0..points3d.len() {
        vertices.push(Pt3::new(
          points3d[p].x * c,
          points3d[p].x * s,
          points3d[p].z,
        ));
        let p3 = segment * points3d_len + p;
        let p1 = segment * points3d_len + ((p + 1) % points3d.len());
        let p2 = (segment - 1) * points3d_len + p;
        let p0 = (segment - 1) * points3d_len + ((p + 1) % points3d.len());
        indices.append(&mut vec![p1, p0, p2]);
        indices.append(&mut vec![p1, p2, p3]);
      }
    }
    if angle != 360.0 {
      let mut pts: Vec<Pt3> = points3d.into_iter().rev().collect();
      let s = dsin(a * segments as f64);
      let c = dcos(a * segments as f64);
      for i in 0..pts.len() {
        pts[i] = Pt3::new(pts[i].x * c, pts[i].x * s, pts[i].z);
      }
      let nml = Pt3::new(0.0, -1.0, 0.0).rotated_z(angle + 180.0);
      let mut indies = triangulate3d(&pts, nml);
      for index in &mut indies {
        *index += vertices.len();
      }
      indices.append(&mut indies);
      vertices.append(&mut pts);
      for p in 0..points3d_len {
        let p3 = vertices.len() - points3d_len * 2 + p;
        let p1 = vertices.len() - points3d_len * 2 + ((p + 1) % points3d_len);
        let p2 = vertices.len() - points3d_len + (points3d_len - 1 - p);
        let p0 = vertices.len() - points3d_len + (points3d_len - 1 - ((p + 1) % points3d_len));
        indices.append(&mut vec![p2, p0, p1]);
        indices.append(&mut vec![p3, p2, p1]);
      }
    } else {
      for p in 0..points3d_len {
        let p3 = p;
        let p1 = (p + 1) % points3d_len;
        let p2 = vertices.len() - points3d_len + p;
        let p0 = vertices.len() - points3d_len + ((p + 1) % points3d_len);
        indices.append(&mut vec![p1, p0, p2]);
        indices.append(&mut vec![p1, p2, p3]);
      }
    }

    Self::from_verts(&vertices, &indices)
  }

  pub fn rotate_twist_extrude(
    points: &Vec<Pt2>,
    angle: f64,
    twist: f64,
    rot_center: Pt2,
    segments: usize,
  ) -> Self {
    assert!(angle >= 0.0 && angle <= 360.0);
    assert!(segments >= 3);
    let points3d: Vec<Pt3> = points.iter().map(|p| Pt3::new(p.x, 0.0, p.y)).collect();
    let points3d_len = points3d.len();
    let a = angle / segments as f64;
    let ta = twist / segments as f64;
    let mut vertices = points3d.clone();
    let mut indices = Vec::new();
    if angle != 360.0 {
      indices.append(&mut triangulate3d(&points3d, Pt3::new(0.0, -1.0, 0.0)));
    }
    for segment in 1..segments {
      let s = dsin(a * segment as f64);
      let c = dcos(a * segment as f64);
      for p in 0..points3d.len() {
        let mut pt = points3d[p];
        pt = pt - Pt3::new(rot_center.x, 0.0, rot_center.y);
        pt.rotate_y(ta * segment as f64);
        pt = pt + Pt3::new(rot_center.x, 0.0, rot_center.y);
        vertices.push(Pt3::new(pt.x * c, pt.x * s, pt.z));
        let p3 = segment * points3d_len + p;
        let p1 = segment * points3d_len + ((p + 1) % points3d_len);
        let p2 = (segment - 1) * points3d_len + p;
        let p0 = (segment - 1) * points3d_len + ((p + 1) % points3d_len);
        indices.append(&mut vec![p1, p0, p2]);
        indices.append(&mut vec![p1, p2, p3]);
      }
    }
    if angle != 360.0 {
      let mut pts: Vec<Pt3> = points3d.into_iter().rev().collect();
      let s = dsin(a * segments as f64);
      let c = dcos(a * segments as f64);

      for i in 0..pts.len() {
        pts[i] -= Pt3::new(rot_center.x, 0.0, rot_center.y);
        pts[i].rotate_y(ta * segments as f64);
        pts[i] += Pt3::new(rot_center.x, 0.0, rot_center.y);
        pts[i] = Pt3::new(pts[i].x * c, pts[i].x * s, pts[i].z);
      }
      let nml = Pt3::new(0.0, -1.0, 0.0).rotated_z(angle + 180.0);
      let mut indies = triangulate3d(&pts, nml);
      for index in &mut indies {
        *index += vertices.len();
      }
      indices.append(&mut indies);
      vertices.append(&mut pts);
      for p in 0..points3d_len {
        let p3 = vertices.len() - points3d_len * 2 + p;
        let p1 = vertices.len() - points3d_len * 2 + ((p + 1) % points3d_len);
        let p2 = vertices.len() - points3d_len + (points3d_len - 1 - p);
        let p0 = vertices.len() - points3d_len + (points3d_len - 1 - ((p + 1) % points3d_len));
        indices.append(&mut vec![p2, p0, p1]);
        indices.append(&mut vec![p3, p2, p1]);
      }
    } else {
      for p in 0..points3d.len() {
        let p3 = p;
        let p1 = (p + 1) % points3d.len();
        let p2 = vertices.len() - points3d_len + p;
        let p0 = vertices.len() - points3d_len + ((p + 1) % points3d_len);
        indices.append(&mut vec![p1, p0, p2]);
        indices.append(&mut vec![p1, p2, p3]);
      }
    }
    Self::from_verts(&vertices, &indices)
  }

  pub fn sweep(profile: &Vec<Pt2>, path: &Vec<Pt3>, twist_degrees: f64) -> Self {
    let profile: Vec<Pt3> = profile.iter().map(|p| p.as_pt3(0.0)).collect();
    let profile_len = profile.len();
    let mut vertices: Vec<Pt3> = Vec::new();
    let mut indices: Vec<usize> = Vec::new();
    let twist_angle = twist_degrees / (path.len() - 1) as f64;

    let m = Mt4::look_at_matrix_lh(path[0], path[1], Pt3::new(0.0, 0.0, 1.0));
    let profile_rev = profile.clone().into_iter().rev().collect::<Vec<Pt3>>();
    for p in &profile_rev {
      vertices.push((m * p.as_pt4(1.0)).as_pt3() + path[0]);
    }
    indices.append(&mut triangulate3d(&vertices, path[0] - path[1]));

    for path_i in 1..path.len() - 1 {
      let m = Mt4::look_at_matrix_lh(path[path_i - 1], path[path_i + 1], Pt3::new(0.0, 0.0, 1.0));
      for profile_i in 0..profile_len {
        let point = profile_rev[profile_i].rotated_z(twist_angle * path_i as f64);
        vertices.push((m * point.as_pt4(1.0)).as_pt3() + path[path_i % path.len()]);
        let p3 = path_i * profile_len + profile_i;
        let p1 = path_i * profile_len + ((profile_i + 1) % profile_len);
        let p2 = (path_i - 1) * profile_len + profile_i;
        let p0 = (path_i - 1) * profile_len + ((profile_i + 1) % profile_len);
        indices.append(&mut vec![p1, p0, p2]);
        indices.append(&mut vec![p1, p2, p3]);
      }
    }

    let m = Mt4::look_at_matrix_lh(
      path[path.len() - 2],
      path[path.len() - 1],
      Pt3::new(0.0, 0.0, 1.0),
    );
    let mut last_verts = Vec::with_capacity(profile_len);
    for profile_i in 0..profile_len {
      let point = profile[profile_i].rotated_z(twist_angle * (path.len() - 1) as f64);
      vertices.push((m * point.as_pt4(1.0)).as_pt3() + path[path.len() - 1]);
      last_verts.push((m * point.as_pt4(1.0)).as_pt3() + path[path.len() - 1]);
      let p3 = (path.len() - 1) * profile_len + (profile_len - 1 - profile_i);
      let p1 = (path.len() - 1) * profile_len + (profile_len - 1 - ((profile_i + 1) % profile_len));
      let p2 = (path.len() - 2) * profile_len + profile_i;
      let p0 = (path.len() - 2) * profile_len + ((profile_i + 1) % profile_len);
      indices.append(&mut vec![p1, p0, p2]);
      indices.append(&mut vec![p1, p2, p3]);
    }

    let mut indies = triangulate3d(&last_verts, path[path.len() - 1] - path[path.len() - 2]);
    for indie in &mut indies {
      *indie += vertices.len() - profile_len;
    }
    indices.append(&mut indies);

    Self::from_verts(&vertices, &indices)
  }

  pub fn sweep_closed(profile: &Vec<Pt2>, path: &Vec<Pt3>, twists: i32) -> Self {
    assert!(path.len() >= 4);
    let profile: Vec<Pt3> = profile.iter().map(|p| p.as_pt3(0.0)).collect();
    let mut vertices: Vec<Pt3> = Vec::new();
    let mut indices: Vec<usize> = Vec::new();
    let twist_angle = 360.0 * twists as f64 / (path.len()) as f64;

    let m = Mt4::look_at_matrix_rh(path[path.len() - 1], path[1], Pt3::new(0.0, 0.0, 1.0));
    for p in &profile {
      vertices.push((m * p.as_pt4(1.0)).as_pt3() + path[0]);
    }

    for path_i in 1..path.len() - 1 {
      let m = Mt4::look_at_matrix_rh(path[path_i - 1], path[path_i + 1], Pt3::new(0.0, 0.0, 1.0));
      for profile_i in 0..profile.len() {
        let point = profile[profile_i].rotated_z(twist_angle * path_i as f64);
        vertices.push((m * point.as_pt4(1.0)).as_pt3() + path[path_i]);
        let p3 = path_i * profile.len() + profile_i;
        let p1 = path_i * profile.len() + ((profile_i + 1) % profile.len());
        let p2 = (path_i - 1) * profile.len() + profile_i;
        let p0 = (path_i - 1) * profile.len() + ((profile_i + 1) % profile.len());
        indices.append(&mut vec![p1, p0, p2]);
        indices.append(&mut vec![p1, p2, p3]);
      }
    }

    let m = Mt4::look_at_matrix_rh(path[path.len() - 2], path[0], Pt3::new(0.0, 0.0, 1.0));
    for profile_i in 0..profile.len() {
      let point = profile[profile_i].rotated_z(twist_angle * (path.len() - 1) as f64);
      vertices.push((m * point.as_pt4(1.0)).as_pt3() + path[path.len() - 1]);
      let p3 = (path.len() - 1) * profile.len() + profile_i;
      let p1 = (path.len() - 1) * profile.len() + ((profile_i + 1) % profile.len());
      let p2 = (path.len() - 2) * profile.len() + profile_i;
      let p0 = (path.len() - 2) * profile.len() + ((profile_i + 1) % profile.len());
      indices.append(&mut vec![p1, p0, p2]);
      indices.append(&mut vec![p1, p2, p3]);
    }
    for profile_i in 0..profile.len() {
      let point = profile[profile_i].rotated_z(twist_angle * path.len() as f64);
      vertices.push((m * point.as_pt4(1.0)).as_pt3() + path[path.len() - 1]);
      let p3 = profile_i;
      let p1 = (profile_i + 1) % profile.len();
      let p2 = (path.len() - 1) * profile.len() + profile_i;
      let p0 = (path.len() - 1) * profile.len() + ((profile_i + 1) % profile.len());
      indices.append(&mut vec![p1, p0, p2]);
      indices.append(&mut vec![p1, p2, p3]);
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

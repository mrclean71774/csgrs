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

//! Meshes are composed of triangles only. The triangles are using ccw winding.
//! All 2d profiles that are used to create meshes are also specified in ccw
//! order as viewed down the Z axis. Boolean mesh operations are '+' for union,
//! '-' for difference, and '*' for intersection.

use {
  crate::{
    csg::CSG,
    ear_clip::triangulate3d,
    scad::SCAD,
    triangle::{Triangle, VecTriangle},
  },
  math::{
    dcos, dsin,
    mt4::Mt4,
    pt2::{Pt2, VecPt2},
    pt3::{Pt3, VecPt3},
  },
  std::{
    collections::HashMap,
    io::{Read, Write},
  },
};

/// A mesh composed of triangles.
#[derive(Clone)]
pub struct Mesh {
  pub triangles: Vec<Triangle>,
}

impl Mesh {
  /// Creates a mesh from a list of triangles.
  ///
  /// triangles: The triangles used to create the mesh.
  ///
  /// return: The resulting mesh.
  pub fn from_triangles(triangles: Vec<Triangle>) -> Self {
    Self { triangles }
  }

  /// Creates a mesh from a CSG object.
  ///
  /// csg: The CSG object.
  ///
  /// return: The mesh.
  pub fn from_csg(csg: CSG) -> Self {
    Self::from_triangles(csg.into_triangles())
  }

  /// Creates a mesh from a list of vertices and an index that specifies
  /// the triagles.
  ///
  /// vertices: The 3D points of the mesh.
  ///
  /// indices: The list of indices into the vertices. Always a multiple of three long.
  ///
  /// return: The mesh.
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

  /// Turn the Mesh into a SCAD object for use with the OpenSCAD backend.
  pub fn into_scad(self) -> SCAD {
    SCAD::from_mesh(self)
  }

  /// Return an array of the unique vertices in a mesh.
  pub fn vertices(&self) -> Vec<Pt3> {
    let mut points: Vec<Pt3> = Vec::new();
    for triangle in &self.triangles {
      let mut a_found = false;
      let mut b_found = false;
      let mut c_found = false;
      for point in &points {
        if triangle.a == *point {
          a_found = true;
        }
        if triangle.b == *point {
          b_found = true;
        }
        if triangle.c == *point {
          c_found = true;
        }
      }
      if !a_found {
        points.push(triangle.a);
      }
      if !b_found {
        points.push(triangle.b);
      }
      if !c_found {
        points.push(triangle.c);
      }
    }
    points
  }

  /// Return all the unique edges in a mesh
  pub fn edges(&self) -> Vec<(Pt3, Pt3)> {
    let mut edges: Vec<(Pt3, Pt3)> = Vec::new();
    for triangle in &self.triangles {
      let edge1 = (triangle.a, triangle.b);
      let edge2 = (triangle.b, triangle.c);
      let edge3 = (triangle.c, triangle.a);

      let mut edge1_found = false;
      let mut edge2_found = false;
      let mut edge3_found = false;

      for edge in &edges {
        if (edge1.0 == edge.0 && edge1.1 == edge.1) || (edge1.1 == edge.0 && edge1.0 == edge.1) {
          edge1_found = true;
        }
        if (edge2.0 == edge.0 && edge2.1 == edge.1) || (edge2.1 == edge.0 && edge2.0 == edge.1) {
          edge2_found = true;
        }
        if (edge3.0 == edge.0 && edge3.1 == edge.1) || (edge3.1 == edge.0 && edge3.0 == edge.1) {
          edge3_found = true;
        }
      }
      if !edge1_found {
        edges.push(edge1);
      }
      if !edge2_found {
        edges.push(edge2);
      }
      if !edge3_found {
        edges.push(edge3);
      }
    }
    edges
  }

  /// Translate a mesh by the given vector.
  ///
  /// v: The translation vector.
  ///
  /// return: A mutable reference to the mesh.
  pub fn translate(&mut self, v: Pt3) -> &mut Self {
    self.triangles.translate(v);
    self
  }

  /// Rotate a mesh around the X axis.
  ///
  /// degrees: The degrees of rotation.
  ///
  /// return: A mutable reference to the mesh.
  pub fn rotate_x(&mut self, degrees: f64) -> &mut Self {
    self.triangles.rotate_x(degrees);
    self
  }

  /// Rotate a mesh around the Y axis.
  ///
  /// degrees: The degrees of rotation.
  ///
  /// return: A mutable reference to the mesh.
  pub fn rotate_y(&mut self, degrees: f64) -> &mut Self {
    self.triangles.rotate_y(degrees);
    self
  }

  /// Rotate a mesh around the Z axis.
  ///
  /// degrees: The degrees of rotation.
  ///
  /// return: A mutable reference to the mesh.
  pub fn rotate_z(&mut self, degrees: f64) -> &mut Self {
    self.triangles.rotate_z(degrees);
    self
  }

  /// Creates a cube primitive.
  ///
  /// x: The X dimension of the cube.
  ///
  /// y: The Y dimension of the cube.
  ///
  /// z: The Z dimension of the cube.
  ///
  /// center: If true the cube is centered at the world origin else
  /// the cube is created in the all posative octant of the world.
  ///
  /// return: The mesh.
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

  /// Creates a sphere primitive centered at the world origin.
  ///
  /// r: The radius of the sphere.
  ///  
  /// segments: The number of segments in a circle.
  ///
  /// return: The mesh.
  pub fn sphere(r: f64, segments: usize) -> Self {
    let points = Pt2::arc(Pt2::new(0.0, -r), 180.0, segments / 2);
    Self::revolve(&points, segments)
  }

  /// Create an arrow shape.
  ///
  /// shaft_radius: The radius of the shaft portion.
  ///
  /// shaft_length: The length of the shaft.
  ///
  /// head_radius: The radius of the largest part of the arrow head.
  ///
  /// head_start: How far up the shaft the largest part of the arrow head is.
  /// Having the head start before or after the shaft_length will produce different
  /// head shapes.
  ///
  /// head_length: The total length of the arrow head.
  ///
  /// segments: The number of segments in a circle.
  ///
  /// center: When true the arrow is centered at the world origen else the arrow
  /// "sits" on the world origin.
  ///
  /// return: The mesh.
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

  /// Creates a cylinder or cone.
  ///
  /// r1: The radius at the bottom of the cylinder.
  ///
  /// r2: The radius at the top of the cylinder.
  ///
  /// height: The height of the cylinder.
  ///
  /// segments: The number of segments in a circle.
  ///
  /// center: When true the cylinder is centered at the world origin else the cylinder
  /// "sits" on the world origin.
  ///
  /// return: The mesh.
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

  /// Creates a linear shape that can be used to chamfer a corner.
  ///
  /// size: The height and width of the angled part of the chamfer.
  ///
  /// oversize: How much non-angled part there is on the chamfer.
  ///
  /// return: The mesh.
  pub fn chamfer(size: f64, length: f64, oversize: f64) -> Self {
    Self::linear_extrude(&Pt2::chamfer(size, oversize), length)
  }

  /// Creates a curved chamfer shape.
  ///
  /// size: The size of the angled part of the chamfer profile.
  ///
  /// oversize: How much non-angled part there is on the chamfer.
  ///
  /// radius: The radius of the arc that the chamfer takes.
  ///
  /// degrees: The degrees of the arc that the chamfer is extruded through.
  ///
  /// segments: The number of segments in a circle.
  ///
  /// return: The mesh.
  pub fn external_circle_chamfer(
    size: f64,
    oversize: f64,
    radius: f64,
    degrees: f64,
    segments: usize,
  ) -> Self {
    let mut points2 = Pt2::chamfer(size, oversize);
    points2
      .rotate(90.0)
      .translate(Pt2::new(radius + size / 2.0 + oversize / 2.0, -oversize));
    Self::rotate_extrude(&points2, degrees, segments)
  }

  /// Creates two external circle chamfers for chamfering a cylinder.
  ///
  /// size: The size of the angled part of the chamfer profile.
  ///
  /// radius: The radius of the cylinder to be chamfered.
  ///
  /// height: The height of the cylinder to be chamfered.
  ///
  /// segments: The number of segments in a circle.
  ///
  /// return: The mesh.
  pub fn external_cylinder_chamfer(
    size: f64,
    radius: f64,
    height: f64,
    segments: usize,
    center: bool,
  ) -> Self {
    let mut result = Self::external_circle_chamfer(size, radius, 360.0, 1.0, segments);
    let mut result1 = Self::external_circle_chamfer(size, radius, 360.0, 1.0, segments);
    result1.rotate_x(180.0);
    result1.translate(Pt3::new(0.0, 0.0, height));
    if center {
      result.translate(Pt3::new(0.0, 0.0, -height / 2.0));
    }
    result + result1
  }

  /// Create a regular polygon.
  ///
  /// n_sides: The number of sides for the polygon.
  ///
  /// radius: The distance from the center of the polygon to the center of a side.
  ///
  /// height: The height of the polygon.
  ///
  /// center: When true the polygon is centered at the world origin else the polygon "sits"
  /// on the world origin.
  ///
  /// return: The mesh;
  pub fn circumscribed_polygon(n_sides: usize, radius: f64, height: f64, center: bool) -> Self {
    let pts = Pt2::circumscribed_polygon(n_sides, radius);
    let mut polygon = Self::linear_extrude(&pts, height);
    if center {
      polygon.translate(Pt3::new(0.0, 0.0, -height / 2.0));
    }
    polygon
  }

  /// Create a regular polygon.
  ///
  /// n_sides: The number of sides for the polygon.
  ///
  /// radius: The distance from the center of the polygon to an exterior point.
  ///
  /// height: The height of the polygon.
  ///
  /// center: When true the polygon is centered at the world origin else the polygon "sits"
  /// on the world origin.
  ///
  /// return: The mesh;
  pub fn inscribed_polygon(n_sides: usize, radius: f64, height: f64, center: bool) -> Self {
    let pts = Pt2::inscribed_polygon(n_sides, radius);
    let mut polygon = Self::linear_extrude(&pts, height);
    if center {
      polygon.translate(Pt3::new(0.0, 0.0, -height / 2.0));
    }
    polygon
  }

  /// Extrude a 2D profile along the positive Z axis.
  ///  
  /// profile: The 2D profile to be extruded.
  ///
  /// height: The height of the resulting shape.
  ///
  /// return: The mesh.
  pub fn linear_extrude(profile: &Vec<Pt2>, height: f64) -> Self {
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
      vertices.push(profile[i].as_pt3(height));
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

  /// Spin a profile around the Z axis to create a shape.
  ///
  /// profile: The 2D profile. Should start and end at x=0.0.
  ///
  /// segments: The number of segments in a circle.
  ///
  /// return: The mesh.
  pub fn revolve(profile: &Vec<Pt2>, segments: usize) -> Self {
    assert!(profile.len() > 2);
    let stride = profile.len() - 2;
    let mut vertices = Vec::new();
    vertices.push(profile[0].to_xz());
    vertices.push(profile[profile.len() - 1].to_xz());
    for i in 1..(profile.len() - 1) {
      vertices.push(profile[i].to_xz());
    }
    let mut indices = Vec::new();
    for segment in 1..segments {
      for i in 1..(profile.len() - 1) {
        let v = &profile[i].to_xz();
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

  /// Rotate a 2D profile around the Z axis.
  ///
  /// profile: The 2D profile to extrude. Should be located in the positive X.
  ///
  /// degrees: The degrees of rotation.
  ///
  /// segments: The number of segments in a circle.
  ///
  /// return: The mesh.
  pub fn rotate_extrude(profile: &Vec<Pt2>, degrees: f64, segments: usize) -> Self {
    assert!(degrees >= 0.0 && degrees <= 360.0);
    assert!(segments >= 3);
    let points3d: Vec<Pt3> = profile.iter().map(|p| Pt3::new(p.x, 0.0, p.y)).collect();
    let points3d_len = points3d.len();
    let a = degrees / segments as f64;
    let mut vertices = points3d.clone();
    let mut indices = Vec::new();
    if degrees != 360.0 {
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
    if degrees != 360.0 {
      let mut pts: Vec<Pt3> = points3d.into_iter().rev().collect();
      let s = dsin(a * segments as f64);
      let c = dcos(a * segments as f64);
      for i in 0..pts.len() {
        pts[i] = Pt3::new(pts[i].x * c, pts[i].x * s, pts[i].z);
      }
      let nml = Pt3::new(0.0, -1.0, 0.0).rotated_z(degrees + 180.0);
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

  /// Rotate a 2D profile around the Z axis.
  ///
  /// profile: The 2D profile to extrude. Should be located in the positive X.
  ///
  /// extrude_degrees: The degrees of rotation.
  ///
  /// twist_degrees: The degrees of twist in the shape. If extrude degrees = 360 then
  /// twist_degrees should be a multiple of 360.
  ///
  /// rotation_center: The point that the profile is twisted around.
  ///
  /// segments: The number of segments in a circle.
  ///
  /// return: The mesh.
  pub fn rotate_twist_extrude(
    profile: &Vec<Pt2>,
    extrude_degrees: f64,
    twist_degrees: f64,
    roatation_center: Pt2,
    segments: usize,
  ) -> Self {
    assert!(extrude_degrees >= 0.0 && extrude_degrees <= 360.0);
    assert!(segments >= 3);
    let points3d: Vec<Pt3> = profile.iter().map(|p| Pt3::new(p.x, 0.0, p.y)).collect();
    let points3d_len = points3d.len();
    let a = extrude_degrees / segments as f64;
    let ta = twist_degrees / segments as f64;
    let mut vertices = points3d.clone();
    let mut indices = Vec::new();
    if extrude_degrees != 360.0 {
      indices.append(&mut triangulate3d(&points3d, Pt3::new(0.0, -1.0, 0.0)));
    }
    for segment in 1..segments {
      let s = dsin(a * segment as f64);
      let c = dcos(a * segment as f64);
      for p in 0..points3d.len() {
        let mut pt = points3d[p];
        pt = pt - Pt3::new(roatation_center.x, 0.0, roatation_center.y);
        pt.rotate_y(ta * segment as f64);
        pt = pt + Pt3::new(roatation_center.x, 0.0, roatation_center.y);
        vertices.push(Pt3::new(pt.x * c, pt.x * s, pt.z));
        let p3 = segment * points3d_len + p;
        let p1 = segment * points3d_len + ((p + 1) % points3d_len);
        let p2 = (segment - 1) * points3d_len + p;
        let p0 = (segment - 1) * points3d_len + ((p + 1) % points3d_len);
        indices.append(&mut vec![p1, p0, p2]);
        indices.append(&mut vec![p1, p2, p3]);
      }
    }
    if extrude_degrees != 360.0 {
      let mut pts: Vec<Pt3> = points3d.into_iter().rev().collect();
      let s = dsin(a * segments as f64);
      let c = dcos(a * segments as f64);

      for i in 0..pts.len() {
        pts[i] -= Pt3::new(roatation_center.x, 0.0, roatation_center.y);
        pts[i].rotate_y(ta * segments as f64);
        pts[i] += Pt3::new(roatation_center.x, 0.0, roatation_center.y);
        pts[i] = Pt3::new(pts[i].x * c, pts[i].x * s, pts[i].z);
      }
      let nml = Pt3::new(0.0, -1.0, 0.0).rotated_z(extrude_degrees + 180.0);
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

  /// Sweeps a profile through a set of points.
  ///
  /// NOTE: A problem shows up when sweeping from vertical to horizontal.  A work around
  /// is to sweep horizontally and then rotate the resulting mesh.
  ///
  /// profile: The 2d points that are swept along the path.
  ///
  /// path: The 3D path the profile is swept along.
  ///
  /// return: The resulting mesh.
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

  /// Sweeps a profile around a set of points and connects the ends.
  ///
  /// NOTE: A problem shows up when sweeping from vertical to horizontal.  A work around
  /// is to sweep horizontally and then rotate the resulting mesh.
  ///
  /// profile: The 2d points that are swept along the path.
  ///
  /// path: The 3D path the profile is swept along.
  ///
  /// return: The resulting mesh.
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

  /// Saves the mesh as an binary stl file.
  ///
  /// path: The path of the file relative to the working directory of the executable.
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

  /// Saves the mesh as an ascii stl file.
  ///
  /// path: The path of the file relative to the working directory of the executable.
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

  /// Load an stl file.
  ///
  /// path: The path of the file relative to the working directory of the executable.
  ///
  /// return: The mesh.
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

  /// Parse the binary stl data.
  ///
  /// data: The bytes of the file.
  ///
  /// return: The mesh.
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

fn lerp(start: Pt3, end: Pt3, n_steps: usize, step: usize) -> Pt3 {
  start + ((end - start) / n_steps as f64 * step as f64)
}

/// Returns the dictionary for the given M size.
///
/// This function always returns a valid
/// dictionary by giving the next smallest size if the requested size is not found. If
/// a size smaller than the smallest is requested the smallest size in dict is returned.
///
/// m: The size of the thread you want dict for e.g. 6 for M6 screw threads.
///
/// return: The dictionary of thread attributes.
fn m_table_lookup(m: i32) -> HashMap<&'static str, f64> {
  let m_table = m_table();
  let mut m = m;
  if m < 2 {
    m = 2;
  }
  loop {
    if m_table.contains_key(&m) {
      break;
    }
    m -= 1;
  }
  return m_table[&m].clone();
}

/// Calculates the thread height from the given pitch.
///
/// pitch: The pitch of the threads.
///
/// return: The height of the threads.
fn thread_height_from_pitch(pitch: f64) -> f64 {
  3.0f64.sqrt() / 2.0 * pitch
}

///  Calculates the dMin of a thread based on the dMaj and pitch.
///
///  d_maj: The dMaj of the threads.
///
///  pitch: The pitch of the threads.
///
///  return: The dMin of the threads.
fn d_min_from_d_maj_pitch(d_maj: f64, pitch: f64) -> f64 {
  d_maj - 2.0 * 5.0 / 8.0 * thread_height_from_pitch(pitch)
}

impl Mesh {
  /// Creates a threaded cylinder.
  ///
  /// d_min: dMin of thread.
  ///
  /// d_maj: dMaj of thread.
  ///
  /// pitch: Pitch of the thread.
  ///
  /// length: The length of the threaded rod.
  ///
  /// segments: The number of segments in a full revolution.
  ///
  /// lead_in: Add lead in on lower Z.
  ///
  /// lead_in_degrees: The total angle of lead in.
  ///
  /// lead_out_degrees: The total angle of lead out.
  ///
  /// left_hand_thread: lefty tighty?
  ///
  /// center: Center vertically.
  ///
  /// return: The threaded cylinder.
  pub fn threaded_cylinder(
    d_min: f64,
    d_maj: f64,
    pitch: f64,
    length: f64,
    segments: usize,
    lead_in: bool,
    lead_in_degrees: f64,
    lead_out: bool,
    lead_out_degrees: f64,
    left_hand_thread: bool,
    center: bool,
  ) -> Self {
    let thread_length = length - 0.7 * pitch;
    let n_revolutions = thread_length / pitch;
    let n_steps = (n_revolutions * segments as f64) as usize;
    let z_step = thread_length / n_steps as f64;
    let step_angle = 360.0 / segments as f64;
    let n_lead_in_steps = (segments as f64 * lead_in_degrees / 360.0 + 2.0) as usize;
    let n_lead_out_steps = (segments as f64 * lead_out_degrees / 360.0) as usize;
    let mut lead_in_step = 2;
    let mut lead_out_step = n_lead_out_steps;

    let thread_profile0 = Pt3::new(d_min / 2.0, 0.0, 3.0 / 4.0 * pitch);
    let thread_profile1 = Pt3::new(d_maj / 2.0, 0.0, 7.0 / 16.0 * pitch);
    let thread_profile2 = Pt3::new(d_min / 2.0, 0.0, 0.0);
    let thread_profile3 = Pt3::new(d_maj / 2.0, 0.0, 5.0 / 16.0 * pitch);

    let lerp_profile1 = Pt3::new(d_min / 2.0, 0.0, 7.0 / 16.0 * pitch);
    let lerp_profile3 = Pt3::new(d_min / 2.0, 0.0, 5.0 / 16.0 * pitch);

    let lead_in_start_profile0 = thread_profile0;
    let lead_in_start_profile2 = thread_profile2;
    let lead_in_start_profile1 = lerp(
      lerp_profile1,
      thread_profile1,
      n_lead_in_steps,
      lead_in_step,
    );
    let lead_in_start_profile3 = lerp(
      lerp_profile3,
      thread_profile3,
      n_lead_in_steps,
      lead_in_step,
    );
    lead_in_step += 1;

    let mut vertices: Vec<Pt3> = Vec::new();
    let mut indices: Vec<usize> = Vec::new();

    // Create the starting end face
    vertices.push(lead_in_start_profile0);
    vertices.push(lead_in_start_profile1);
    vertices.push(lead_in_start_profile2);
    vertices.push(lead_in_start_profile3);

    if left_hand_thread {
      indices.append(&mut vec![0, 1, 2]);
      indices.append(&mut vec![2, 1, 3]);
    } else {
      indices.append(&mut vec![2, 1, 0]);
      indices.append(&mut vec![3, 1, 2]);
    }

    // Vertices used for the middle sections
    let mut p4;
    let mut p5;
    let mut p6;
    let mut p7;

    let lead_in_profile0 = lead_in_start_profile0;
    let mut lead_in_profile1 = lead_in_start_profile1;
    let lead_in_profile2 = lead_in_start_profile2;
    let mut lead_in_profile3 = lead_in_start_profile3;

    let lead_out_profile0 = thread_profile0;
    let mut lead_out_profile1 = thread_profile1;
    let lead_out_profile2 = thread_profile2;
    let mut lead_out_profile3 = thread_profile3;

    for step in 0..(n_steps - 1) {
      let mut angle = step_angle * (step + 1) as f64;
      if left_hand_thread {
        angle *= -1.0;
      }
      let c = dcos(angle);
      let s = dsin(angle);
      if lead_in_step < n_lead_in_steps && lead_in {
        p4 = Pt3::new(
          c * lead_in_profile0.x,
          s * lead_in_profile0.x,
          z_step * step as f64 + lead_in_profile0.z,
        );
        p5 = Pt3::new(
          c * lead_in_profile1.x,
          s * lead_in_profile1.x,
          z_step * step as f64 + lead_in_profile1.z,
        );
        p6 = Pt3::new(
          c * lead_in_profile2.x,
          s * lead_in_profile2.x,
          z_step * step as f64 + lead_in_profile2.z,
        );
        p7 = Pt3::new(
          c * lead_in_profile3.x,
          s * lead_in_profile3.x,
          z_step * step as f64 + lead_in_profile3.z,
        );

        lead_in_step += 1;
        lead_in_profile1 = lerp(
          lead_in_start_profile1,
          thread_profile1,
          n_lead_in_steps,
          lead_in_step,
        );
        lead_in_profile3 = lerp(
          lead_in_start_profile3,
          thread_profile3,
          n_lead_in_steps,
          lead_in_step,
        );
      } else if lead_out_step > 0 && step >= n_steps - n_lead_out_steps && lead_out {
        p4 = Pt3::new(
          c * lead_out_profile0.x,
          s * lead_out_profile0.x,
          z_step * step as f64 + lead_out_profile0.z,
        );
        p5 = Pt3::new(
          c * lead_out_profile1.x,
          s * lead_out_profile1.x,
          z_step * step as f64 + lead_out_profile1.z,
        );
        p6 = Pt3::new(
          c * lead_out_profile2.x,
          s * lead_out_profile2.x,
          z_step * step as f64 + lead_out_profile2.z,
        );
        p7 = Pt3::new(
          c * lead_out_profile3.x,
          s * lead_out_profile3.x,
          z_step * step as f64 + lead_out_profile3.z,
        );
        lead_out_step -= 1;
        lead_out_profile1 = lerp(
          thread_profile1,
          lead_in_start_profile1,
          n_lead_out_steps,
          n_lead_out_steps - lead_out_step,
        );
        lead_out_profile3 = lerp(
          thread_profile3,
          lead_in_start_profile3,
          n_lead_out_steps,
          n_lead_out_steps - lead_out_step,
        );
      } else {
        p4 = Pt3::new(
          c * thread_profile0.x,
          s * thread_profile0.x,
          z_step * step as f64 + thread_profile0.z,
        );
        p5 = Pt3::new(
          c * thread_profile1.x,
          s * thread_profile1.x,
          z_step * step as f64 + thread_profile1.z,
        );
        p6 = Pt3::new(
          c * thread_profile2.x,
          s * thread_profile2.x,
          z_step * step as f64 + thread_profile2.z,
        );
        p7 = Pt3::new(
          c * thread_profile3.x,
          s * thread_profile3.x,
          z_step * step as f64 + thread_profile3.z,
        );
      }

      vertices.push(p4);
      vertices.push(p5);
      vertices.push(p6);
      vertices.push(p7);

      let index_offset = step * 4;
      if left_hand_thread {
        indices.append(&mut vec![
          1 + index_offset,
          5 + index_offset,
          3 + index_offset,
        ]);
        indices.append(&mut vec![
          3 + index_offset,
          5 + index_offset,
          7 + index_offset,
        ]);
        indices.append(&mut vec![
          0 + index_offset,
          4 + index_offset,
          1 + index_offset,
        ]);
        indices.append(&mut vec![
          1 + index_offset,
          4 + index_offset,
          5 + index_offset,
        ]);
        indices.append(&mut vec![
          2 + index_offset,
          6 + index_offset,
          0 + index_offset,
        ]);
        indices.append(&mut vec![
          0 + index_offset,
          6 + index_offset,
          4 + index_offset,
        ]);
        indices.append(&mut vec![
          3 + index_offset,
          7 + index_offset,
          2 + index_offset,
        ]);
        indices.append(&mut vec![
          2 + index_offset,
          7 + index_offset,
          6 + index_offset,
        ]);
      } else {
        indices.append(&mut vec![
          3 + index_offset,
          5 + index_offset,
          1 + index_offset,
        ]);
        indices.append(&mut vec![
          7 + index_offset,
          5 + index_offset,
          3 + index_offset,
        ]);
        indices.append(&mut vec![
          1 + index_offset,
          4 + index_offset,
          0 + index_offset,
        ]);
        indices.append(&mut vec![
          5 + index_offset,
          4 + index_offset,
          1 + index_offset,
        ]);
        indices.append(&mut vec![
          0 + index_offset,
          6 + index_offset,
          2 + index_offset,
        ]);
        indices.append(&mut vec![
          4 + index_offset,
          6 + index_offset,
          0 + index_offset,
        ]);
        indices.append(&mut vec![
          2 + index_offset,
          7 + index_offset,
          3 + index_offset,
        ]);
        indices.append(&mut vec![
          6 + index_offset,
          7 + index_offset,
          2 + index_offset,
        ]);
      }
    } // end loop

    let index_offset = (n_steps - 2) * 4;
    if left_hand_thread {
      indices.append(&mut vec![
        6 + index_offset,
        7 + index_offset,
        5 + index_offset,
      ]);
      indices.append(&mut vec![
        6 + index_offset,
        5 + index_offset,
        4 + index_offset,
      ]);
    } else {
      indices.append(&mut vec![
        5 + index_offset,
        7 + index_offset,
        6 + index_offset,
      ]);
      indices.append(&mut vec![
        4 + index_offset,
        5 + index_offset,
        6 + index_offset,
      ]);
    }

    let threads = Self::from_verts(&vertices, &indices);
    let rod = Self::cylinder(
      d_min / 2.0 + 0.0001,
      d_min / 2.0 + 0.0001,
      length,
      segments,
      false,
    );
    let mut result = threads + rod;

    if center {
      result.translate(Pt3::new(0.0, 0.0, -length / 2.0));
    }
    result
  }

  /// Creates a threaded rod at the world origin.
  ///
  /// m: The metric size of the rod.
  ///
  /// length: The length of the rod in mm.
  ///
  /// segments: The number of segments in a circle.
  ///
  /// lead_in: Tapers off the threads at the bottom of the rod.
  ///
  /// lead_in_degrees: Span of the lead in.
  ///
  /// lead_out: Tapers off the threads at the top of the rod.
  ///
  /// lead_out_degrees: Span of the lead out.
  ///
  /// left_hand_thread: lefty tighty?
  ///
  /// center: Center vertically.
  ///
  /// return: The threaded rod.
  pub fn threaded_rod(
    m: i32,
    length: f64,
    segments: usize,
    lead_in: bool,
    lead_in_degrees: f64,
    lead_out: bool,
    lead_out_degrees: f64,
    left_hand_thread: bool,
    center: bool,
  ) -> Self {
    let thread_info = m_table_lookup(m);
    let pitch = thread_info["pitch"];
    let d_maj = thread_info["external_dMaj"];
    let d_min = d_min_from_d_maj_pitch(d_maj, pitch);

    Self::threaded_cylinder(
      d_min,
      d_maj,
      pitch,
      length,
      segments,
      lead_in,
      lead_in_degrees,
      lead_out,
      lead_out_degrees,
      left_hand_thread,
      center,
    )
  }

  /// Create a hex head bolt at the world origin.
  ///
  /// m: The metric bolt size.
  ///
  /// length: The length of the threaded part.
  ///
  /// head_height: The height of the hex head.
  ///
  /// segments: The number of segments in a circle.
  ///
  /// lead_in: Tapers off the thread at the end.
  ///
  /// lead_in_degrees: The amount of degrees the tapered thread occupies.
  ///
  /// chamfered: Whether or not to chamfer the top and bottom of the head.
  ///
  /// chamfer_size: The size of the chamfer.
  ///
  /// left_hand_thread: lefty tighty?
  ///
  /// center: Center vertically.
  ///
  /// return: The hex bolt.
  pub fn hex_bolt(
    m: i32,
    length: f64,
    head_height: f64,
    segments: usize,
    lead_in: bool,
    lead_in_degrees: f64,
    chamfered: bool,
    chamfer_size: f64,
    left_hand_thread: bool,
    center: bool,
  ) -> Self {
    let thread_info = m_table_lookup(m);
    let pitch = thread_info["pitch"];
    let d_maj = thread_info["external_dMaj"];
    let head_diameter = thread_info["nut_width"];
    let d_min = d_min_from_d_maj_pitch(d_maj, pitch);

    let mut rod = Self::threaded_cylinder(
      d_min,
      d_maj,
      pitch,
      length,
      segments,
      false,
      180.0,
      lead_in,
      lead_in_degrees,
      left_hand_thread,
      center,
    );
    rod.translate(Pt3::new(0.0, 0.0, head_height));

    let mut head = Self::circumscribed_polygon(6, head_diameter / 2.0, head_height, false);
    let mut chamfer_size = chamfer_size;
    if chamfered {
      if chamfer_size == 0.0 {
        chamfer_size = 0.2 * head_diameter;
      }
      head = head
        - Self::external_cylinder_chamfer(
          chamfer_size,
          (0.25 * head_diameter * 0.25 * head_diameter + 0.5 * head_diameter * 0.5 * head_diameter)
            .sqrt(),
          head_height,
          segments,
          false,
        );
    }
    let mut bolt = rod + head;
    if center {
      bolt.translate(Pt3::new(0.0, 0.0, -((head_height + length) / 2.0)));
    }
    bolt
  }

  /// Create a tap for making threaded holes in things.
  ///
  /// m: The metric size of the tap.
  ///
  /// length: The length of the tap.
  ///
  /// segments: The number of segmentst in a circle.
  ///
  /// left_hand_thread: lefty tighty?
  ///
  /// center: Center vertically.
  ///
  /// return: The tap.
  pub fn tap(m: i32, length: f64, segments: usize, left_hand_thread: bool, center: bool) -> Self {
    let thread_info = m_table_lookup(m);
    let pitch = thread_info["pitch"];
    let d_maj = thread_info["internal_dMaj"];
    let d_min = d_min_from_d_maj_pitch(d_maj, pitch);

    Self::threaded_cylinder(
      d_min,
      d_maj,
      pitch,
      length,
      segments,
      false,
      90.0,
      false,
      90.0,
      left_hand_thread,
      center,
    )
  }

  /// Create a hex nut.
  ///
  /// m: The metric size of the nut.
  ///
  /// height: The height of the nut.
  ///
  /// segments: The number of segments in a circle.
  ///
  /// chamfered: Adds a chamfer to the nut.
  ///
  /// chamfer_size: The size of the chamfer, leave at 0.0 for the default size.
  ///
  /// left_hand_thread: lefty tighty?
  ///
  /// center: Center horizontally.
  ///
  /// return: The nut.
  pub fn hex_nut(
    m: i32,
    height: f64,
    segments: usize,
    chamfered: bool,
    chamfer_size: f64,
    left_hand_thread: bool,
    center: bool,
  ) -> Self {
    let thread_info = m_table_lookup(m);
    let nut_width = thread_info["nut_width"];

    let mut nut_tap = Self::tap(m, height + 20.0, segments, left_hand_thread, center);
    nut_tap.translate(Pt3::new(0.0, 0.0, -10.0));

    let nut_blank = Self::circumscribed_polygon(6, nut_width / 2.0, height, false);

    let mut nut = nut_blank - nut_tap;
    let mut chamfer_size = chamfer_size;
    if chamfered {
      if chamfer_size == 0.0 {
        chamfer_size = 2.0;
      }
      nut = nut
        - Self::external_cylinder_chamfer(
          chamfer_size,
          (0.25 * nut_width * 0.25 * nut_width + 0.5 * nut_width * 0.5 * nut_width).sqrt(),
          height,
          segments,
          center,
        )
    }

    if center {
      nut.translate(Pt3::new(0.0, 0.0, -height / 2.0));
    }

    nut
  }
}

/// Returns the hashmap of iso metric thread profiles
fn m_table() -> HashMap<i32, HashMap<&'static str, f64>> {
  HashMap::from([
    (
      2,
      HashMap::from([
        ("pitch", 0.4),
        ("external_dMaj", 1.886),
        ("internal_dMaj", 2.148),
        ("nut_width", 4.0),
      ]),
    ),
    (
      3,
      HashMap::from([
        ("pitch", 0.5),
        ("external_dMaj", 2.874),
        ("internal_dMaj", 3.172),
        ("nut_width", 5.5),
      ]),
    ),
    (
      4,
      HashMap::from([
        ("pitch", 0.7),
        ("external_dMaj", 3.838),
        ("internal_dMaj", 4.219),
        ("nut_width", 7.0),
      ]),
    ),
    (
      5,
      HashMap::from([
        ("pitch", 0.8),
        ("external_dMaj", 5.826),
        ("internal_dMaj", 5.24),
        ("nut_width", 8.0),
      ]),
    ),
    (
      6,
      HashMap::from([
        ("pitch", 1.0),
        ("external_dMaj", 5.794),
        ("internal_dMaj", 6.294),
        ("nut_width", 10.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      7,
      HashMap::from([
        ("pitch", 1.0),
        ("external_dMaj", 6.794),
        ("internal_dMaj", 7.294),
        ("nut_width", 13.0),
      ]),
    ),
    (
      8,
      HashMap::from([
        ("pitch", 1.25),
        ("external_dMaj", 7.76),
        ("internal_dMaj", 8.34),
        ("nut_width", 13.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      9,
      HashMap::from([
        ("pitch", 1.25),
        ("external_dMaj", 8.76),
        ("internal_dMaj", 9.34),
        ("nut_width", 16.0),
      ]),
    ),
    (
      10,
      HashMap::from([
        ("pitch", 1.5),
        ("external_dMaj", 9.732),
        ("internal_dMaj", 10.396),
        ("nut_width", 16.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      11,
      HashMap::from([
        ("pitch", 1.5),
        ("external_dMaj", 10.73),
        ("internal_dMaj", 11.387),
        ("nut_width", 18.0),
      ]),
    ),
    (
      12,
      HashMap::from([
        ("pitch", 1.75),
        ("external_dMaj", 11.7),
        ("internal_dMaj", 12.453),
        ("nut_width", 18.0),
      ]),
    ),
    (
      14,
      HashMap::from([
        ("pitch", 2.0),
        ("external_dMaj", 13.68),
        ("internal_dMaj", 14.501),
        ("nut_width", 21.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      15,
      HashMap::from([
        ("pitch", 1.5),
        ("external_dMaj", 14.73),
        ("internal_dMaj", 15.407),
        ("nut_width", 24.0),
      ]),
    ),
    (
      16,
      HashMap::from([
        ("pitch", 2.0),
        ("external_dMaj", 15.68),
        ("internal_dMaj", 16.501),
        ("nut_width", 24.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      17,
      HashMap::from([
        ("pitch", 1.5),
        ("external_dMaj", 16.73),
        ("internal_dMaj", 17.407),
        ("nut_width", 27.0),
      ]),
    ),
    (
      18,
      HashMap::from([
        ("pitch", 2.5),
        ("external_dMaj", 17.62),
        ("internal_dMaj", 18.585),
        ("nut_width", 27.0),
      ]),
    ),
    (
      20,
      HashMap::from([
        ("pitch", 2.5),
        ("external_dMaj", 19.62),
        ("internal_dMaj", 20.585),
        ("nut_width", 30.0),
      ]),
    ),
    (
      22,
      HashMap::from([
        ("pitch", 3.0),
        ("external_dMaj", 21.58),
        ("internal_dMaj", 22.677),
        ("nut_width", 34.0),
      ]),
    ),
    (
      24,
      HashMap::from([
        ("pitch", 3.0),
        ("external_dMaj", 23.58),
        ("internal_dMaj", 24.698),
        ("nut_width", 36.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      25,
      HashMap::from([
        ("pitch", 2.0),
        ("external_dMaj", 24.68),
        ("internal_dMaj", 25.513),
        ("nut_width", 41.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      26,
      HashMap::from([
        ("pitch", 1.5),
        ("external_dMaj", 25.73),
        ("internal_dMaj", 26.417),
        ("nut_width", 41.0),
      ]),
    ),
    (
      27,
      HashMap::from([
        ("pitch", 3.0),
        ("external_dMaj", 26.58),
        ("internal_dMaj", 27.698),
        ("nut_width", 41.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      28,
      HashMap::from([
        ("pitch", 2.0),
        ("external_dMaj", 27.68),
        ("internal_dMaj", 28.513),
        ("nut_width", 46.0),
      ]),
    ),
    (
      30,
      HashMap::from([
        ("pitch", 3.5),
        ("external_dMaj", 29.52),
        ("internal_dMaj", 30.785),
        ("nut_width", 46.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      32,
      HashMap::from([
        ("pitch", 2.0),
        ("external_dMaj", 31.68),
        ("internal_dMaj", 32.513),
        ("nut_width", 49.0),
      ]),
    ),
    (
      33,
      HashMap::from([
        ("pitch", 3.5),
        ("external_dMaj", 32.54),
        ("internal_dMaj", 33.785),
        ("nut_width", 49.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      35,
      HashMap::from([
        ("pitch", 1.5),
        ("external_dMaj", 34.73),
        ("internal_dMaj", 35.416),
        ("nut_width", 55.0),
      ]),
    ),
    (
      36,
      HashMap::from([
        ("pitch", 4.0),
        ("external_dMaj", 35.47),
        ("internal_dMaj", 36.877),
        ("nut_width", 55.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      38,
      HashMap::from([
        ("pitch", 1.5),
        ("external_dMaj", 37.73),
        ("internal_dMaj", 38.417),
        ("nut_width", 60.0),
      ]),
    ),
    (
      39,
      HashMap::from([
        ("pitch", 4.0),
        ("external_dMaj", 38.47),
        ("internal_dMaj", 39.877),
        ("nut_width", 60.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      40,
      HashMap::from([
        ("pitch", 3.0),
        ("external_dMaj", 39.58),
        ("internal_dMaj", 40.698),
        ("nut_width", 65.0),
      ]),
    ),
    (
      42,
      HashMap::from([
        ("pitch", 4.5),
        ("external_dMaj", 41.44),
        ("internal_dMaj", 42.965),
        ("nut_width", 65.0),
      ]),
    ),
    (
      45,
      HashMap::from([
        ("pitch", 4.5),
        ("external_dMaj", 44.44),
        ("internal_dMaj", 45.965),
        ("nut_width", 70.0),
      ]),
    ),
    (
      48,
      HashMap::from([
        ("pitch", 5.0),
        ("external_dMaj", 47.4),
        ("internal_dMaj", 49.057),
        ("nut_width", 75.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      50,
      HashMap::from([
        ("pitch", 4.0),
        ("external_dMaj", 49.47),
        ("internal_dMaj", 50.892),
        ("nut_width", 80.0),
      ]),
    ),
    (
      52,
      HashMap::from([
        ("pitch", 5.0),
        ("external_dMaj", 51.4),
        ("internal_dMaj", 53.037),
        ("nut_width", 80.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      55,
      HashMap::from([
        ("pitch", 4.0),
        ("external_dMaj", 54.47),
        ("internal_dMaj", 55.892),
        ("nut_width", 85.0),
      ]),
    ),
    (
      56,
      HashMap::from([
        ("pitch", 5.5),
        ("external_dMaj", 55.37),
        ("internal_dMaj", 57.149),
        ("nut_width", 85.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      58,
      HashMap::from([
        ("pitch", 4.0),
        ("external_dMaj", 57.47),
        ("internal_dMaj", 58.892),
        ("nut_width", 90.0),
      ]),
    ),
    (
      60,
      HashMap::from([
        ("pitch", 5.5),
        ("external_dMaj", 59.37),
        ("internal_dMaj", 61.149),
        ("nut_width", 90.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      62,
      HashMap::from([
        ("pitch", 4.0),
        ("external_dMaj", 61.47),
        ("internal_dMaj", 62.892),
        ("nut_width", 95.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      63,
      HashMap::from([
        ("pitch", 1.5),
        ("external_dMaj", 62.73),
        ("internal_dMaj", 63.429),
        ("nut_width", 95.0),
      ]),
    ),
    (
      64,
      HashMap::from([
        ("pitch", 6.0),
        ("external_dMaj", 63.32),
        ("internal_dMaj", 65.421),
        ("nut_width", 95.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      65,
      HashMap::from([
        ("pitch", 4.0),
        ("external_dMaj", 64.47),
        ("internal_dMaj", 65.892),
        ("nut_width", 100.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      68,
      HashMap::from([
        ("pitch", 6.0),
        ("external_dMaj", 67.32),
        ("internal_dMaj", 69.241),
        ("nut_width", 100.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      70,
      HashMap::from([
        ("pitch", 6.0),
        ("external_dMaj", 69.32),
        ("internal_dMaj", 71.241),
        ("nut_width", 100.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      72,
      HashMap::from([
        ("pitch", 6.0),
        ("external_dMaj", 71.32),
        ("internal_dMaj", 73.241),
        ("nut_width", 110.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      75,
      HashMap::from([
        ("pitch", 6.0),
        ("external_dMaj", 74.32),
        ("internal_dMaj", 76.241),
        ("nut_width", 110.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      76,
      HashMap::from([
        ("pitch", 6.0),
        ("external_dMaj", 75.32),
        ("internal_dMaj", 77.241),
        ("nut_width", 110.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      78,
      HashMap::from([
        ("pitch", 2.0),
        ("external_dMaj", 77.68),
        ("internal_dMaj", 78.525),
        ("nut_width", 120.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      80,
      HashMap::from([
        ("pitch", 6.0),
        ("external_dMaj", 79.32),
        ("internal_dMaj", 81.241),
        ("nut_width", 120.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      82,
      HashMap::from([
        ("pitch", 2.0),
        ("external_dMaj", 81.68),
        ("internal_dMaj", 82.525),
        ("nut_width", 120.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      85,
      HashMap::from([
        ("pitch", 6.0),
        ("external_dMaj", 84.32),
        ("internal_dMaj", 86.241),
        ("nut_width", 130.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      90,
      HashMap::from([
        ("pitch", 6.0),
        ("external_dMaj", 89.32),
        ("internal_dMaj", 91.241),
        ("nut_width", 130.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      95,
      HashMap::from([
        ("pitch", 6.0),
        ("external_dMaj", 94.32),
        ("internal_dMaj", 96.266),
        ("nut_width", 130.0),
      ]),
    ),
    // nut_width made up for next entry
    (
      100,
      HashMap::from([
        ("pitch", 6.0),
        ("external_dMaj", 99.32),
        ("internal_dMaj", 101.27),
        ("nut_width", 140.0),
      ]),
    ),
  ])
}

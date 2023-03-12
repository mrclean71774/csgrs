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

//! In this example we create a police badge shape to demonstrate cubic bezier chains
//! and quadradic bezier curves.
//!
//! When creating a cubic bezier chain you specify the start, end, and 2 control points
//! for the first link. For subsequent links you only specify the length/strength of the
//! first control point because it's position must be tangent to the previous links second
//! control point. And if you call close on the chain you provide a length for control1
//! in the first link to replace the absolute position you provided upon creation of the chain.

use csg::{CubicBezierChain2D, Mesh, Pt2, Viewer};

fn main() {
  // whether to save the viewer or mesh output
  let save_viewer = false;

  // cubic bezier chain points
  let bottom_center = Pt2::new(0.0, -4.0);
  let cp1 = Pt2::new(1.0, -2.0);
  let cp2 = Pt2::new(4.0, -4.0);
  let end1 = Pt2::new(4.0, -1.0);
  let cp3 = Pt2::new(1.5, 2.5);
  let end2 = Pt2::new(3.75, 3.0);

  // quadratic bezier points
  let top_right = Pt2::new(2.5, 4.5);
  let cp4 = Pt2::new(1.0, 3.0);
  let top_center = Pt2::new(0.0, 4.5);

  // create the right side profile with a 2 link cubic bezier chain
  // and a quadratic bezier
  let mut lower_right_curve = CubicBezierChain2D::new(bottom_center, cp1, cp2, end1);
  lower_right_curve.add(1.0, cp3, end2);
  let mut profile = lower_right_curve.gen_points(12);
  profile.append(&mut Pt2::quadratic_bezier(top_right, cp4, top_center, 6));

  // copy everything but the first and last points in the right side to the left
  // side in reverse order to complete the profile and ensure ccw winding
  let n_points = profile.len();
  for i in 1..n_points - 1 {
    profile.push(Pt2::new(
      -profile[n_points - 1 - i].x,
      profile[n_points - 1 - i].y,
    ));
  }

  if save_viewer {
    let mut viewer = Viewer::new(0.3, 0.15, 6);
    viewer.add_vert2s(&profile);
    viewer.render().into_scad().save_scad("out/shield.scad");
  } else {
    let shield = Mesh::linear_extrude(&profile, 0.5);
    shield.save_stl_bin("out/shield.stl")
  }
}

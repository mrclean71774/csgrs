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

//! In this example we will use the SCADViewer to visualize a curve. The control
//! points of the curves will be red and the points will be yellow in the viewer.
//! The main reason to use a SCADViewer over a Viewer is the ease of displaying
//! curves.

use csg::{CubicBezier2D, Pt2, SCADViewer};

fn main() {
  let cubic_bezier_2d = CubicBezier2D::new(
    Pt2::new(0.0, 0.0),
    Pt2::new(-5.0, 5.0),
    Pt2::new(5.0, 10.0),
    Pt2::new(0.0, 20.0),
    10,
  );
  let mut viewer = SCADViewer::new(0.5, 0.25, 6);
  viewer.add_cubic_bezier2d(cubic_bezier_2d);
  viewer.render().save_scad("out/scad_viewer.scad");
}

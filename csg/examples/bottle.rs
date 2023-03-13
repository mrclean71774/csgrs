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

//! This example makes a cap and bottle using the OpenSCAD backend that can be 3D printed.

use csg::{Mesh, Pt2, Pt3, SCAD};

fn main() {
  make_bottle();
  make_cap();
}

// The cap is pretty simple just a cylinder minus a tap. It's important
// to use a tap to make the hole and not a threaded rod because taps are
// bigger than the corresponding rod.
fn make_cap() {
  let cylinder = Mesh::cylinder(23.0, 23.0, 12.0, 36, false).into_scad();
  let mut tap = SCAD::tap(40, 14.0, 36, false, false);
  tap.translate(Pt3::new(0.0, 0.0, 2.0));

  let cap = cylinder - tap;
  cap.save_scad("out/bottle_cap.scad");
}

// The bottle is a little more complicated but with bezier curves we can
// make a nice shape. When writing this I used the Viewer to display the
// points in the profiles. To make the bottle we need an outside shape,
// a little threaded rod and an inside shape. After making the union of
// the outside and the threaded rod we subtract the inside.
//
// To make the outside and inside shapes we make half the profile of a
// bottle and use revolve to make the solid. The points in the profile
// need to be specified in a counter clockwise order so we start at the
// origin and make our way to the right, then up, and finally back to
// the Y axis at the top.
//
// The revolve function takes this 2D profile and converts it to 3D by
// leaving X were it is and mapping the Y axis to the Z axis. Basically
// standing up the 2D profile. It then revolves that profile around the
// Z axis to make a solid shape.
//
// After making the outside profile we can make the inside profile by just
// insetting the points a little. I chose to inset by 2mm for mose of the
// bottle but by 3mm at the top because it looked weak at the threaded part.
//
// After making the three parts we add the outside and the threaded rod and
// subtract the inside shape to hollow it out. Finally we save the scad file
// so that we can render it in OpenSCAD to get our bottle and cap.
fn make_bottle() {
  let mut outside_profile = Vec::new();
  outside_profile.push(Pt2::new(0.0, 0.0));
  outside_profile.append(&mut Pt2::quadratic_bezier(
    Pt2::new(27.5, 0.0),
    Pt2::new(32.5, 0.0),
    Pt2::new(32.5, 5.0),
    6,
  ));
  outside_profile.append(&mut Pt2::cubic_bezier(
    Pt2::new(32.5, 100.0),
    Pt2::new(32.5, 105.0),
    Pt2::new(22.5, 110.0),
    Pt2::new(19.0, 125.0),
    6,
  ));
  outside_profile.push(Pt2::new(0.0, 125.0));

  let mut inside_profile = Vec::new();
  inside_profile.push(Pt2::new(0.0, 2.0));
  inside_profile.append(&mut Pt2::quadratic_bezier(
    Pt2::new(25.5, 2.0),
    Pt2::new(30.5, 2.0),
    Pt2::new(30.5, 7.0),
    6,
  ));
  inside_profile.append(&mut Pt2::cubic_bezier(
    Pt2::new(30.5, 100.0),
    Pt2::new(30.5, 105.0),
    Pt2::new(20.5, 110.0),
    Pt2::new(16.0, 125.0),
    6,
  ));
  inside_profile.push(Pt2::new(16.0, 140.0));
  inside_profile.push(Pt2::new(0.0, 140.0));

  let outside = Mesh::revolve(&outside_profile, 36).into_scad();
  let inside = Mesh::revolve(&inside_profile, 36).into_scad();

  let mut threaded_rod = SCAD::threaded_rod(40, 15.0, 36, false, 0.0, true, 180.0, false, false);
  threaded_rod.translate(Pt3::new(0.0, 0.0, 120.0));

  let bottle = outside + threaded_rod - inside;

  bottle.save_scad("out/bottle.scad");
}

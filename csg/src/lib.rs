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

mod bsp_node;
mod csg;
mod ear_clip;
mod mesh;
mod plane;
mod polygon;
mod scad;
mod triangle;
mod viewer;

pub use {
  bsp_node::BSPNode,
  csg::CSG,
  csg_math::{
    approx_eq, dacos, dasin, datan, dcos, dsin, dtan, CubicBezier2D, CubicBezier3D,
    CubicBezierChain2D, CubicBezierChain3D, MersenneTwister, Mt4, Pt2, Pt3, Pt4, QuadraticBezier2D,
    QuadraticBezier3D, VecPt2, VecPt3,
  },
  ear_clip::{triangulate2d, triangulate3d},
  mesh::Mesh,
  plane::Plane,
  polygon::Polygon,
  scad::{SCADColor, SCAD},
  triangle::{Triangle, VecTriangle},
  viewer::Viewer,
};

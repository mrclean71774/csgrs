//MIT License
//
//Copyright (c) 2023 Michael H. Phillips
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

//! A double precision, non generic, math library.

mod mt4;
mod mt4f;
mod pt2;
mod pt2f;
mod pt3;
mod pt3f;
mod pt4;
mod pt4f;
mod rng;

pub use {
  mt4::Mt4,
  mt4f::Mt4f,
  pt2::{CubicBezier2D, CubicBezierChain2D, Pt2, QuadraticBezier2D, VecPt2},
  pt2f::{CubicBezier2Df, CubicBezierChain2Df, Pt2f, QuadraticBezier2Df, VecPt2f},
  pt3::{CubicBezier3D, CubicBezierChain3D, Pt3, QuadraticBezier3D, VecPt3},
  pt3f::{CubicBezier3Df, CubicBezierChain3Df, Pt3f, QuadraticBezier3Df, VecPt3f},
  pt4::Pt4,
  pt4f::Pt4f,
  rng::MersenneTwister,
};

pub const PI: f64 = std::f64::consts::PI;
pub const PI_O_2: f64 = std::f64::consts::FRAC_PI_2;
pub const ROOT2: f64 = std::f64::consts::SQRT_2;
pub const ROOT2_O_2: f64 = ROOT2 / 2.0;
pub const GOLDEN_RATIO: f64 = 1.6180339887498948482045868343656;

/// Returns the minimum of a and b
#[macro_export]
macro_rules! min {
  ($a:expr, $b:expr) => {
    if ($a) < ($b) {
      ($a)
    } else {
      ($b)
    }
  };
}

/// Returns the maximum of a and b
#[macro_export]
macro_rules! max {
  ($a:expr, $b:expr) => {
    if ($a) > ($b) {
      ($a)
    } else {
      ($b)
    }
  };
}

/// Returns val if val is in between min and max. Returns min if val < min.
/// Returns max if val is > max.
#[macro_export]
macro_rules! clamp {
  ($val:expr, $min:expr, $max:expr) => {
    min!(($max), max!(($min), ($val)))
  };
}

/// Returns the sine of degrees
#[inline(always)]
pub fn dsin(degrees: f64) -> f64 {
  degrees.to_radians().sin()
}

/// Returns the cosine of degrees
#[inline(always)]
pub fn dcos(degrees: f64) -> f64 {
  degrees.to_radians().cos()
}

/// Returns the tangent of degrees
#[inline(always)]
pub fn dtan(degrees: f64) -> f64 {
  degrees.to_radians().tan()
}

/// Returns the arc-sine of degrees
#[inline(always)]
pub fn dasin(degrees: f64) -> f64 {
  degrees.to_radians().asin()
}

/// Returns the arc-cosine of degrees
#[inline(always)]
pub fn dacos(degrees: f64) -> f64 {
  degrees.to_radians().acos()
}

/// Returns the arc-tangent of degrees
#[inline(always)]
pub fn datan(degrees: f64) -> f64 {
  degrees.to_radians().atan()
}

/// Returns the sine of degrees
#[inline(always)]
pub fn dsinf(degrees: f32) -> f32 {
  degrees.to_radians().sin()
}

/// Returns the cosine of degrees
#[inline(always)]
pub fn dcosf(degrees: f32) -> f32 {
  degrees.to_radians().cos()
}

/// Returns the tangent of degrees
#[inline(always)]
pub fn dtanf(degrees: f32) -> f32 {
  degrees.to_radians().tan()
}

/// Returns the arc-sine of degrees
#[inline(always)]
pub fn dasinf(degrees: f32) -> f32 {
  degrees.to_radians().asin()
}

/// Returns the arc-cosine of degrees
#[inline(always)]
pub fn dacosf(degrees: f32) -> f32 {
  degrees.to_radians().acos()
}

/// Returns the arc-tangent of degrees
#[inline(always)]
pub fn datanf(degrees: f32) -> f32 {
  degrees.to_radians().atan()
}

/// Returns true if a and b are within epsilon
#[inline(always)]
pub fn approx_eq(a: f64, b: f64, epsilon: f64) -> bool {
  (a - b).abs() < epsilon
}

/// Returns true if a and b are within epsilon
#[inline(always)]
pub fn approx_eqf(a: f32, b: f32, epsilon: f32) -> bool {
  (a - b).abs() < epsilon
}

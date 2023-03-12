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

//! Binary Space Partitioning part of https://github.com/timknip/pycsg port.

use crate::{plane::Plane, polygon::Polygon};

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
    if polygons.is_some() {
      node.build(polygons.unwrap());
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
    let n_polygons = polygons.len();
    if n_polygons == 0 {
      return;
    }
    if self.plane.is_none() {
      self.plane = Some(polygons[0].plane);
    }
    self.polygons.push(polygons[0].clone());
    let mut front: Vec<Polygon> = Vec::new();
    let mut back: Vec<Polygon> = Vec::new();
    for i in 1..n_polygons {
      self.plane.as_mut().unwrap().split_polygon(
        &polygons[i],
        &mut self.polygons,
        &mut self.polygons,
        &mut front,
        &mut back,
      );
    }
    if front.len() > 0 {
      if self.front.is_none() {
        self.front = Some(Box::new(BSPNode::new(None)));
      }
      self.front.as_mut().unwrap().build(front);
    }
    if back.len() > 0 {
      if self.back.is_none() {
        self.back = Some(Box::new(BSPNode::new(None)));
      }
      self.back.as_mut().unwrap().build(back);
    }
  }
}

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

use math::{approx_eq, pt2::Pt2, pt3::Pt3};

fn is_ccw(pts: &Vec<(usize, Pt2)>) -> bool {
  (pts[1].1.x - pts[0].1.x) * (pts[2].1.y - pts[0].1.y)
    - (pts[2].1.x - pts[0].1.x) * (pts[1].1.y - pts[0].1.y)
    > 0.0
}

fn in_triangle(v: &(usize, Pt2), a: &(usize, Pt2), b: &(usize, Pt2), c: &(usize, Pt2)) -> bool {
  let mut denom = (b.1.y - c.1.y) * (a.1.x - c.1.x) + (c.1.x - b.1.x) * (a.1.y - c.1.y);
  if approx_eq(denom, 0.0, 1.0e-5) {
    return true;
  }
  denom = 1.0 / denom;

  let alpha = denom * ((b.1.y - c.1.y) * (v.1.x - c.1.x) + (c.1.x - b.1.x) * (v.1.y - c.1.y));
  if alpha < 0.0 {
    return false;
  }

  let beta = denom * ((c.1.y - a.1.y) * (v.1.x - c.1.x) + (a.1.x - c.1.x) * (v.1.y - c.1.y));
  if beta < 0.0 {
    return false;
  }

  let gamma = 1.0 - alpha - beta;
  if gamma < 0.0 {
    return false;
  }
  true
}

pub fn triangulate3d(vertices: &Vec<Pt3>, normal: Pt3) -> Vec<usize> {
  assert!(vertices.len() > 3);
  const PX: u8 = 1;
  const NX: u8 = 2;
  const PY: u8 = 3;
  const NY: u8 = 4;
  const PZ: u8 = 5;
  const NZ: u8 = 6;
  let mut nml_type = 0;
  if normal.x.abs() >= normal.y.abs() && normal.x.abs() >= normal.z.abs() {
    if normal.x >= 0.0 {
      nml_type = PX;
    } else {
      nml_type = NX;
    }
  }
  if normal.y.abs() >= normal.x.abs() && normal.y.abs() >= normal.z.abs() {
    if normal.y >= 0.0 {
      nml_type = PY;
    } else {
      nml_type = NY;
    }
  }
  if normal.z.abs() >= normal.x.abs() && normal.z.abs() >= normal.y.abs() {
    if normal.z >= 0.0 {
      nml_type = PZ;
    } else {
      nml_type = NZ;
    }
  }
  let mut polygon = Vec::with_capacity(vertices.len());
  match nml_type {
    PX => {
      // x = y, y = z
      for (i, v) in vertices.iter().enumerate() {
        polygon.push((i, Pt2::new(v.y, v.z)));
      }
    }
    NX => {
      // x = -y, y = z
      for (i, v) in vertices.iter().enumerate() {
        polygon.push((i, Pt2::new(-v.y, v.z)));
      }
    }
    PY => {
      // x = -x, y = z
      for (i, v) in vertices.iter().enumerate() {
        polygon.push((i, Pt2::new(-v.x, v.z)));
      }
    }
    NY => {
      // x = x, y = z
      for (i, v) in vertices.iter().enumerate() {
        polygon.push((i, Pt2::new(v.x, v.z)));
      }
    }
    PZ => {
      // x = x, y = y
      for (i, v) in vertices.iter().enumerate() {
        polygon.push((i, Pt2::new(v.x, v.y)));
      }
    }
    NZ => {
      // x = -x, y =  y
      for (i, v) in vertices.iter().enumerate() {
        polygon.push((i, Pt2::new(-v.x, v.y)));
      }
    }
    _ => {}
  }

  let mut triangles: Vec<usize> = Vec::with_capacity((vertices.len() - 2) * 3);

  let mut left = polygon[0].1;
  let mut index = 0usize;

  for i in 0..polygon.len() {
    if polygon[i].1.x < left.x
      || (approx_eq(polygon[i].1.x, left.x, 1.0e-5) && polygon[i].1.y < left.y)
    {
      index = i;
      left = polygon[i].1;
    }
  }

  let tri = vec![
    polygon[if index == 0 {
      polygon.len() - 1
    } else {
      index - 1
    }],
    polygon[index],
    polygon[if index == polygon.len() - 1 {
      0
    } else {
      index + 1
    }],
  ];
  assert!(is_ccw(&tri));

  while polygon.len() >= 3 {
    let mut eartip = -1i16;
    let mut index = -1i16;

    for i in &polygon {
      index += 1;
      if eartip >= 0 {
        break;
      }

      let p: u16 = if index == 0 {
        (polygon.len() - 1) as u16
      } else {
        (index - 1) as u16
      };
      let n: u16 = if index as usize == polygon.len() - 1 {
        0
      } else {
        (index + 1) as u16
      };

      let tri = vec![polygon[p as usize], *i, polygon[n as usize]];
      if !is_ccw(&tri) {
        continue;
      }

      let mut ear = true;
      for j in ((index + 1) as usize)..polygon.len() {
        let v = &polygon[j];
        if std::ptr::eq(v, &polygon[p as usize])
          || std::ptr::eq(v, &polygon[n as usize])
          || std::ptr::eq(v, &polygon[index as usize])
        {
          continue;
        }
        if in_triangle(v, &polygon[p as usize], i, &polygon[n as usize]) {
          ear = false;
          break;
        }
      }

      if ear {
        eartip = index;
      }
    } // for i in &polygon
    if eartip < 0 {
      break;
    }
    let p = if eartip == 0 {
      polygon.len() - 1
    } else {
      eartip as usize - 1
    };
    let n = if eartip == (polygon.len() - 1) as i16 {
      0
    } else {
      eartip as usize + 1
    };
    triangles.push(polygon[p].0);
    triangles.push(polygon[eartip as usize].0);
    triangles.push(polygon[n].0);

    polygon.remove(eartip as usize);
  } // while polygon.len()

  assert!(triangles.len() == (vertices.len() - 2) * 3);
  triangles
}

pub fn triangulate2d(vertices: &Vec<Pt2>) -> Vec<usize> {
  assert!(vertices.len() > 3);
  let mut polygon = Vec::with_capacity(vertices.len());
  for (i, v) in vertices.iter().enumerate() {
    polygon.push((i, *v));
  }

  let mut triangles: Vec<usize> = Vec::with_capacity((vertices.len() - 2) * 3);

  let mut left = polygon[0].1;
  let mut index = 0usize;

  for i in 0..polygon.len() {
    if polygon[i].1.x < left.x
      || (approx_eq(polygon[i].1.x, left.x, 1.0e-5) && polygon[i].1.y < left.y)
    {
      index = i;
      left = polygon[i].1;
    }
  }

  let tri = vec![
    polygon[if index == 0 {
      polygon.len() - 1
    } else {
      index - 1
    }],
    polygon[index],
    polygon[if index == polygon.len() - 1 {
      0
    } else {
      index + 1
    }],
  ];
  assert!(is_ccw(&tri));

  while polygon.len() >= 3 {
    let mut eartip = -1i16;
    let mut index = -1i16;

    for i in &polygon {
      index += 1;
      if eartip >= 0 {
        break;
      }

      let p: u16 = if index == 0 {
        (polygon.len() - 1) as u16
      } else {
        (index - 1) as u16
      };
      let n: u16 = if index as usize == polygon.len() - 1 {
        0
      } else {
        (index + 1) as u16
      };

      let tri = vec![polygon[p as usize], *i, polygon[n as usize]];
      if !is_ccw(&tri) {
        continue;
      }

      let mut ear = true;

      for j in ((index + 1) as usize)..polygon.len() {
        let v = &polygon[j];
        if std::ptr::eq(v, &polygon[p as usize])
          || std::ptr::eq(v, &polygon[n as usize])
          || std::ptr::eq(v, &polygon[index as usize])
        {
          continue;
        }
        if in_triangle(v, &polygon[p as usize], i, &polygon[n as usize]) {
          ear = false;
          break;
        }
      }

      if ear {
        eartip = index;
      }
    } // for i in &polygon
    if eartip < 0 {
      break;
    }
    let p = if eartip == 0 {
      polygon.len() - 1
    } else {
      eartip as usize - 1
    };
    let n = if eartip == (polygon.len() - 1) as i16 {
      0
    } else {
      eartip as usize + 1
    };
    triangles.push(polygon[p].0);
    triangles.push(polygon[eartip as usize].0);
    triangles.push(polygon[n].0);

    polygon.remove(eartip as usize);
  } // while polygon.len()

  triangles
}

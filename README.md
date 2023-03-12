# csgrs

Programmatic CAD in Rust

The CSG boolean operations are a port of [pycsg](https://github.com/timknip/pycsg) and are not very fast.
They also take alot of memory both on the heap and the stack. The boolean ops produce meshes that look ok
but may have errors and not be watertight. Some fixing of the stl files produced is probably needed for 3D
printing.

There is now an [OpenSCAD](openscad.org) backend that will output a scad file. OpenSCAD is not particularly
fast either but at least the meshes come out watertight.

The csg crate is mostly documented with RustDoc comments now.

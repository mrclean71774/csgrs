# csgrs

Programmatic CAD in Rust

---

To use this library add the following to your Cargo.toml under dependencies.

``` toml
csg = { git="https://github.com/mrclean71774/csgrs" }
```

To run the examples clone this repository and from the root folder run `cargo run --example --release example_name`
and the output will go into the out directory.

The csg library was started because I like programmatic CAD, having used [OpenSCAD](openscad.org) for several
years, but not liking the language that much.

I started with a port of [pycsg](https://github.com/timknip/pycsg) and the algorithm works but does not
always produce watertight meshes. In fact the stl files produced, while still looking ok, are not watertight
for all but the simplest meshes.

Since the meshes produced by the [pycsg](https://github.com/timknip/pycsg) port are not watertight I have added a
way to export [OpenSCAD](openscad.org) code and then [OpenSCAD](openscad.org) can do the booleans and produce watertight meshes for
3D printing etc.

---

## Project Structure

* csg - csg is the main crate with some examples of usage.
* csg_math - csg_math crate has linear algebra and other math stuff most of which is re-exported from csg.
* images - images contains images of things created with this library.
* in - in contains any files needed for input for the examples.
* out - out contains output from the examples minus the .scad files.

---

A [Blender](blender.org) rendor of some nuts, bolts, and threaded rod that were produced with the OpenSCAD backend.

![iso_metric.png](https://github.com/mrclean71774/csgrs/blob/main/images/iso_metric.png)

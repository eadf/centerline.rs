[![Crates.io](https://meritbadge.herokuapp.com/centerline)](https://crates.io/crates/centerline)
[![Documentation](https://docs.rs/centerline/badge.svg)](https://docs.rs/centerline)
[![Workflow](https://github.com/eadf/centerline.rs/workflows/Rust/badge.svg)](https://github.com/eadf/centerline.rs/workflows/Rust/badge.svg)
[![Workflow](https://github.com/eadf/centerline.rs/workflows/Clippy/badge.svg)](https://github.com/eadf/centerline.rs/workflows/Clippy/badge.svg)
[![dependency status](https://deps.rs/crate/centerline/0.0.2/status.svg)](https://deps.rs/crate/centerline/0.0.2)

# Centerline

Finds 'a' centerline of closed 2D geometries.
It uses a [segmented voronoi diagram](https://crates.io/crates/boostvoronoi) as a base, then it filters out the 
'spiky' bits (green) by comparing the angle between the edge (green), and the input geometry (red) that created it.
If the angle is close to 90°, it will be ignored. Note that the result (blue) technically is not a true centerline after the spikes has been filtered out, but it 
makes for much cleaner tool-paths etc. 

![unfiltered](unfiltered.png) ![filtered](filtered.png)

```rust
let segments = ...same as boost voronoi segments...
let mut centerline = Centerline::<i32, f32, i64, f64>::with_segments(segments);
centerline.build_voronoi()?;
let _= centerline.calculate_centerline(0.38, 0.1)?;
println!(
   "Result: lines:{}, line_strings:{}",
   centerline.lines.as_ref().map_or(0,|x|x.len()),
   centerline.line_strings.as_ref().map_or(0,|x|x.len())
);
```

## Gui example

```fish
cargo +nightly run --example fltk_gui
```
The example only displays 2D, but the generated center-line is actually 3D line segments.\
The Z coordinate is the distance between the 2D center-line, and the geometry that created it. 

It should be possible to replace the example input data by overwriting the ```rust.obj``` file in the ```example``` folder.
The new .obj file just needs to be 2D in some axis aligned plane.

## Rust requirement

Requires ```#![feature(hash_drain_filter)]``` i.e. ```+nightly```

## Todo

- [ ] Group related shapes together based on convex hull and not just AABB
- [ ] Rayon over each shape group (problem: boostvoronoi diagrams uses std::cell::Cell)
- [ ] Open file dialogue for fltk_gui
- [ ] Maybe, maybe make it possible to save result from fltk_gui :)

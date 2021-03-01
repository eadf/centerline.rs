#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![deny(unused_results)]
//#![deny(unused_imports)]
#![allow(unused_imports)]
#![feature(hash_drain_filter)]

use boostvoronoi::builder as VB;
use boostvoronoi::InputType;
//use geo::{Coordinate, Line};
//use intersect2d;
use linestring::cgmath_2d;
use linestring::cgmath_3d;
use std::fmt::Display;
use std::ops::Neg;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum CenterlineError {
    #[error("Something is wrong with the input data")]
    CouldNotCalculateInverseMatrix,

    #[error("Your line-strings are self-intersecting.")]
    SelfIntersectingData,

    #[error("The input data is not 2D")]
    InputNotPLane,

    #[error("Invalid data")]
    InvalidData,

    //#[error(transparent)]
    //IoRead(#[from]  std::io::Read),
    #[error(transparent)]
    ObjError(#[from] obj::ObjError),

    #[error(transparent)]
    IoError(#[from] std::io::Error),
    //#[error(transparent)]
    //Intersect(#[from] intersect2d::Error),
}

#[derive(Debug)]
struct Vertices {
    id: usize,
    point: (f32, f32, f32, f32),
    edges: Vec<usize>,
    shape: Option<usize>,
}

fn paint_vertex(vertices: &mut fnv::FnvHashMap<usize, Vertices>, vertex_id: usize, color: usize) {
    let edges = if let Some(v) = vertices.get_mut(&vertex_id) {
        if v.shape.is_none() {
            v.shape = Some(color);
        } else {
            return;
        }
        v.edges.clone()
    } else {
        // vertex already culled as internal
        return;
    };

    for i in edges.iter() {
        if *i == vertex_id {
            panic!();
        }
        paint_vertex(vertices, *i, color);
    }
}

/// remove internal edges from a wavefront-obj object
pub fn remove_internal_edges(
    obj: obj::raw::RawObj,
) -> Result<Vec<cgmath_3d::LineStringSet3<f32>>, CenterlineError> {
    for p in obj.points.iter() {
        // Ignore all points
        println!("Ignored point:{:?}", p);
    }
    //dbg!(1);
    let mut all_edges = fnv::FnvHashSet::<(usize, usize)>::default();
    let mut internal_edges = fnv::FnvHashSet::<(usize, usize)>::default();

    for i in 0..obj.lines.len() {
        // keep all lines
        //println!("Line:{:?}", obj.lines[i]);

        let v = match &obj.lines[i] {
            obj::raw::object::Line::P(a) => a.clone(),
            obj::raw::object::Line::PT(a) => a.iter().map(|x| x.0).collect::<Vec<usize>>(),
        };
        //println!("Line Vec:{:?}", v);
        let mut i1 = v.iter();

        for i in v.iter().skip(1) {
            let i1_v = *i1.next().unwrap();
            let i2_v = *i;
            let key = (*std::cmp::min(&i1_v, &i2_v), *std::cmp::max(&i1_v, &i2_v));
            if all_edges.contains(&key) {
                let _ = internal_edges.insert(key);
            } else {
                let _ = all_edges.insert(key);
            }
        }
    }
    //println!("Internal edges: {:?}", internal_edges);
    //println!("All edges: {:?}", all_edges);
    //println!("Vertices: {:?}", obj.positions);
    //dbg!(2);
    for i in 0..obj.polygons.len() {
        // keep edges without twins, drop the rest
        let v = match &obj.polygons[i] {
            obj::raw::object::Polygon::P(a) => {
                //println!("P{:?}", a);
                let mut v = a.clone();
                v.push(a[0]);
                v
            }
            obj::raw::object::Polygon::PT(a) => {
                //println!("PT{:?}", a);
                let mut v = a.iter().map(|x| x.0).collect::<Vec<usize>>();
                v.push(a[0].0);
                v
            }
            obj::raw::object::Polygon::PN(a) => {
                //println!("PN{:?}", a);
                let mut v = a.iter().map(|x| x.0).collect::<Vec<usize>>();
                v.push(a[0].0);
                v
            }
            obj::raw::object::Polygon::PTN(a) => {
                //println!("PTN{:?}", a);
                let mut v = a.iter().map(|x| x.0).collect::<Vec<usize>>();
                v.push(a[0].0);
                v
            }
        };

        let mut i1 = v.iter();
        for i in v.iter().skip(1) {
            let i1_v = *i1.next().unwrap();
            let i2_v = *i;
            let key = (*std::cmp::min(&i1_v, &i2_v), *std::cmp::max(&i1_v, &i2_v));
            if all_edges.contains(&key) {
                let _ = internal_edges.insert(key);
            } else {
                let _ = all_edges.insert(key);
            }
        }
    }
    //println!("Internal edges: {:?}", internal_edges);
    //println!("All edges: {:?}", all_edges);
    //println!("Vertices: {:?}", obj.positions);
    //dbg!(3);
    let _ = all_edges.drain_filter(|x| internal_edges.contains(x));
    // all_edges should now contain the outline and none of the internal edges.
    //println!("All edges: {:?}", all_edges);
    // put all edges into a hashmap of Vertices, this will make it possible to
    // arrange them in the order they are connected
    let mut vertices = fnv::FnvHashMap::<usize, Vertices>::default();
    for e in all_edges.iter() {
        let id = e.0;
        let other = e.1;
        if let Some(v) = vertices.get_mut(&id) {
            v.edges.push(other);
        } else {
            let _ = vertices.insert(
                id,
                Vertices {
                    id,
                    point: obj.positions[id],
                    edges: vec![other],
                    shape: None,
                },
            );
        }
        let id = e.1;
        let other = e.0;
        if let Some(v) = vertices.get_mut(&id) {
            v.edges.push(other);
        } else {
            let _ = vertices.insert(
                id,
                Vertices {
                    id,
                    point: obj.positions[id],
                    edges: vec![other],
                    shape: None,
                },
            );
        }
    }
    //dbg!(4);
    //println!("Vertices: {:?}", vertices.iter().map(|x|x.1.id).collect::<Vec<usize>>());
    // Do a recursive search on one vertex, paint all connected vertices with the same number.
    let mut shape_iter = 0..usize::MAX;
    let vertex_ids: Vec<usize> = vertices.iter().map(|x| *x.0).collect();
    for vertex_id in vertex_ids.into_iter() {
        if let Some(v) = vertices.get(&vertex_id) {
            if v.shape.is_some() {
                continue;
            }
        }
        // found an un-painted vertex
        paint_vertex(&mut vertices, vertex_id, shape_iter.next().unwrap());
    }

    for v in vertices.iter() {
        if *v.0 != v.1.id {
            println!("Id and key does not match key:{} id:{}", v.0, v.1.id);
        }
        if v.1.shape.is_none() {
            println!(
                "unpainted vertex: {} shape:{:?} edges:{:?}",
                v.1.id, v.1.shape, v.1.edges
            );
            //panic!();
        }
    }

    //println!("Vertices: {:?}", vertices.iter().map(|x|x.1.id).collect::<Vec<usize>>());
    //println!("Color: {:?}", vertices.iter().map(|x|x.1.shape).collect::<Vec<Option<usize>>>());
    //dbg!(5);
    // Spit all detected connected vertices into separate sets.
    // i.e. every vertex with the same color goes into the same set.
    let mut shape_iter = 0..usize::MAX;
    let mut shape_separation = Vec::<fnv::FnvHashMap<usize, Vertices>>::new();
    //dbg!(5.5);
    loop {
        if vertices.is_empty() {
            break;
        }
        let current_shape = shape_iter.next().unwrap();
        //let mut shape = fnv::FnvHashMap::<usize, Vertices>::new();
        let drained = vertices
            .drain_filter(|_, x| {
                if let Some(shape) = x.shape {
                    shape == current_shape
                } else {
                    false
                }
            })
            .collect();
        shape_separation.push(drained);
    }
    //println!("shape_separation.len()={}", shape_separation.len());
    //dbg!(6);
    // now we have a list of groups of vertices, each group are connected by edges.
    // Create lists of linestrings3 by walking the edges of each vertex set.
    let mut rv = Vec::<cgmath_3d::LineStringSet3<f32>>::with_capacity(shape_separation.len());

    #[allow(unused_assignments)]
    for rvi in shape_separation.iter() {
        if rvi.is_empty() {
            continue;
        }

        let mut rvs = cgmath_3d::LineStringSet3::<f32>::with_capacity(shape_separation.len());
        let mut als = cgmath_3d::LineString3::with_capacity(rvi.len());

        let started_with: usize = rvi.iter().next().unwrap().1.id;
        let mut prev: usize = started_with;
        let mut current: usize = started_with;
        let mut next: usize = started_with;
        let mut first_loop = true;
        //dbg!(7);

        loop {
            prev = current;
            current = next;
            if let Some(current_vertex) = rvi.get(&current) {
                als.push(cgmath::point3(
                    current_vertex.point.0,
                    current_vertex.point.1,
                    current_vertex.point.2,
                ));

                //assert_eq!(newV.edges.len(),2);
                next = *current_vertex
                    .edges
                    .iter().find(|x| **x != prev)
                    .unwrap();

                //println!("current:{} prev:{} next:{} startedwith:{}", current, prev, next, started_with);
            } else {
                //println!("Could not get vertex data");
                break;
            }
            // allow the start point to be added twice (in case of a loop)
            if !first_loop && current == started_with {
                break;
            }
            first_loop = false;
        }
        //dbg!(8);
        if als.points().last() != als.points().first() {
            println!(
                "Linestring is not connected ! {:?} {:?}",
                als.points().first(),
                als.points().last()
            );
            println!("Linestring is not connected ! {:?}", als.points());
        }
        //dbg!(9);
        rvs.push(als);
        rv.push(rvs);
    }
    //dbg!(10);
    Ok(rv)
}

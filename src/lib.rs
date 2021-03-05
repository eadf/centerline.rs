#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![deny(unused_results)]
//#![deny(unused_imports)]
#![allow(unused_imports)]
#![feature(hash_drain_filter)]

use boostvoronoi::builder as VB;
use boostvoronoi::diagram as VD;
use boostvoronoi::{BigFloatType, BigIntType, InputType, OutputType};

use boostvoronoi::builder::Builder;
use cgmath::{Point2, Point3};
use fnv;
use linestring::cgmath_2d;
use linestring::cgmath_2d::Shape2d::Line;
use linestring::cgmath_2d::{Line2, VoronoiParabolicArc};
use linestring::cgmath_3d;
use linestring::cgmath_3d::{Line3, LineString3, LineStringSet3};
use std::borrow::Borrow;
use std::cell::RefCell;
use std::fmt::Display;
use std::ops::Neg;
use std::rc::Rc;
use std::sync::{Arc, RwLock};
use thiserror::Error;

const EXTERNAL_COLOR: u32 = 1;

#[derive(Error, Debug)]
pub enum CenterlineError {
    #[error("Something is wrong with the internal logic")]
    InternalError { txt: String },

    #[error("Something is wrong with the input data")]
    CouldNotCalculateInverseMatrix,

    #[error("Your line-strings are self-intersecting.")]
    SelfIntersectingData,

    #[error("The input data is not 2D")]
    InputNotPLane,

    #[error("Invalid data")]
    InvalidData,

    #[error(transparent)]
    BvError(#[from] boostvoronoi::BvError),

    #[error(transparent)]
    ObjError(#[from] obj::ObjError),

    #[error(transparent)]
    IoError(#[from] std::io::Error),
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
) -> Result<Vec<LineStringSet3<f32>>, CenterlineError> {
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
    let mut rv = Vec::<LineStringSet3<f32>>::with_capacity(shape_separation.len());

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
                next = *current_vertex.edges.iter().find(|x| **x != prev).unwrap();

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

pub struct Centerline<I1, F1, I2, F2>
where
    I1: InputType + Neg<Output = I1>,
    F1: cgmath::BaseFloat + OutputType + Neg<Output = F1>,
    I2: BigIntType + Neg<Output = I2>,
    F2: BigFloatType + Neg<Output = F2>,
{
    // the input data to the voronoi diagram
    pub segments: Vec<boostvoronoi::Line<I1>>,
    // the voronoi diagram itself
    pub diagram: Option<VD::VoronoiDiagram<I1, F1, I2, F2>>,
    // rejected or already processed edges
    rejected_edges: fnv::FnvHashSet<usize>,
    // rejected or already processed vertices
    rejected_vertices: fnv::FnvHashSet<usize>,
    pub lines: Vec<Line3<F1>>,
    pub arcs: Vec<VoronoiParabolicArc<F1>>,
    pub linestrings: Vec<LineString3<F1>>,
}

impl<I1, F1, I2, F2> Centerline<I1, F1, I2, F2>
where
    I1: InputType + Neg<Output = I1>,
    F1: cgmath::BaseFloat + OutputType + Neg<Output = F1>,
    I2: BigIntType + Neg<Output = I2>,
    F2: BigFloatType + Neg<Output = F2>,
{
    /// Creates a Centerline container with a set of segments
    pub fn default() -> Self {
        Self {
            diagram: None,
            segments: Vec::<boostvoronoi::Line<I1>>::default(),
            rejected_edges: fnv::FnvHashSet::<usize>::default(),
            rejected_vertices: fnv::FnvHashSet::<usize>::default(),
            lines: Vec::<Line3<F1>>::new(),
            linestrings: Vec::<LineString3<F1>>::new(),
            arcs: Vec::<VoronoiParabolicArc<F1>>::new(),
        }
    }

    /// Creates a Centerline container with a set of segments
    pub fn with_segments(segments: Vec<boostvoronoi::Line<I1>>) -> Self {
        Self {
            diagram: None,
            segments,
            rejected_edges: fnv::FnvHashSet::<usize>::default(),
            rejected_vertices: fnv::FnvHashSet::<usize>::default(),
            lines: Vec::<Line3<F1>>::new(),
            linestrings: Vec::<LineString3<F1>>::new(),
            arcs: Vec::<VoronoiParabolicArc<F1>>::new(),
        }
    }

    pub fn build_voronoi(&mut self) -> Result<(), CenterlineError> {
        self.rejected_edges.clear();
        self.rejected_vertices.clear();

        let mut vb = Builder::new();
        vb.with_segments(self.segments.iter())?;
        let rv = vb.construct()?;
        self.diagram = Some(rv);
        Ok(())
    }

    pub fn calculate_centerline(&mut self) -> Result<(), CenterlineError> {
        if self.diagram.is_none() {
            return Err(CenterlineError::InternalError {
                txt: "self.diagram was none".to_string(),
            });
        }

        let diagram = self.diagram.take().unwrap();

        self.reject_exterior(&diagram);
        self.traverse_edges(&diagram);

        self.diagram = Some(diagram);
        Ok(())
    }

    pub fn retrieve_point(
        diagram: &VD::VoronoiDiagram<I1, F1, I2, F2>,
        cell_id: VD::VoronoiCellIndex,
        segments: &Vec<boostvoronoi::Line<I1>>,
    ) -> boostvoronoi::Point<I1> {
        let (index, category) = diagram.get_cell(cell_id).get().source_index_2();
        match category {
            VD::SourceCategory::SinglePoint => panic!("No points in the input data"),
            VD::SourceCategory::SegmentStart => segments[index].start,
            VD::SourceCategory::Segment | VD::SourceCategory::SegmentEnd => segments[index].end,
        }
    }

    pub fn retrieve_segment(
        diagram: &VD::VoronoiDiagram<I1, F1, I2, F2>,
        cell_id: VD::VoronoiCellIndex,
        segments: &Vec<boostvoronoi::Line<I1>>,
    ) -> boostvoronoi::Line<I1> {
        let cell = diagram.get_cell(cell_id).get();
        segments[cell.source_index()]
    }

    pub fn diagram(&self) -> Result<&VD::VoronoiDiagram<I1, F1, I2, F2>, CenterlineError> {
        if let Some(ref diagram) = self.diagram {
            Ok(diagram)
        } else {
            Err(CenterlineError::InternalError {
                txt: "self.diagram was none".to_string(),
            })
        }
    }

    // Color exterior edges.
    fn reject_exterior(&mut self, diagram: &VD::VoronoiDiagram<I1, F1, I2, F2>) {
        for it in diagram.edges().iter() {
            let edge_id = Some(it.get().get_id());
            if !diagram.edge_is_finite(edge_id).unwrap() {
                self.reject_edge(diagram, edge_id);
            }
        }
    }

    #[inline(always)]
    fn is_edge_rejected(
        &self,
        //rejected_edges: &fnv::FnvHashSet<usize>,
        edge_id: Option<VD::VoronoiEdgeIndex>,
    ) -> bool {
        if let Some(edge_id) = edge_id {
            return self.rejected_edges.contains(&edge_id.0);
        }
        false
    }

    #[inline(always)]
    fn set_edge_rejected(&mut self, edge_id: Option<VD::VoronoiEdgeIndex>) {
        if let Some(edge_id) = edge_id {
            let _ = self.rejected_edges.insert(edge_id.0);
        }
    }

    #[inline(always)]
    fn is_vertex_rejected(
        rejected_vertices: &mut fnv::FnvHashSet<usize>,
        vertex_id: Option<VD::VoronoiEdgeIndex>,
    ) -> bool {
        if let Some(vertex_id) = vertex_id {
            return rejected_vertices.contains(&vertex_id.0);
        }
        false
    }

    #[inline(always)]
    fn set_vertex_rejected(&mut self, vertex_id: Option<VD::VoronoiVertexIndex>) {
        if let Some(vertex_id) = vertex_id {
            let _ = self.rejected_vertices.insert(vertex_id.0);
        }
    }

    /// Recursively marks this edge and all other edges connecting to it as rejected.
    /// Recursion stops when connecting to input geometry.
    fn reject_edge(
        &mut self,
        diagram: &VD::VoronoiDiagram<I1, F1, I2, F2>,
        edge_id: Option<VD::VoronoiEdgeIndex>,
    ) {
        if edge_id.is_none() || self.is_edge_rejected(edge_id) {
            return;
        }
        self.set_edge_rejected(edge_id);
        // todo: is this correct?
        self.set_edge_rejected(diagram.edge_get_twin(edge_id));
        let v = diagram.edge_get_vertex1(edge_id);
        if v.is_none() || !diagram.get_edge(edge_id.unwrap()).get().is_primary() {
            return;
        }
        self.set_vertex_rejected(v);
        let mut e = diagram.vertex_get_incident_edge(v);
        let v_incident_edge = e;
        while e.is_some() {
            self.reject_edge(diagram, e);
            e = diagram.edge_rot_next(e);
            if e == v_incident_edge {
                break;
            }
        }
    }

    /// move across each edge and sample the lines and arcs
    fn traverse_edges(&mut self, diagram: &VD::VoronoiDiagram<I1, F1, I2, F2>) {
        self.lines.clear();
        self.arcs.clear();
        self.linestrings.clear();
        for it in diagram.edges().iter().enumerate() {
            let edge_id = VD::VoronoiEdgeIndex(it.0);

            let edge = it.1.get();
            if !edge.is_primary() || self.is_edge_rejected(Some(edge_id)) {
                continue;
            }
            let edge_twin_id = diagram.edge_get_twin(Some(edge_id));

            if !diagram.edge_is_finite(Some(edge_id)).unwrap() {
                println! {"Error: Edge is NOT finite! {:?}", edge_id};
                self.reject_edge(diagram, Some(edge_id));
                continue;
            } else {
                let vertex0 = diagram.vertex_get(edge.vertex0()).unwrap().get();

                let vertex1 = diagram.edge_get_vertex1(Some(edge_id));
                let vertex1 = diagram.vertex_get(vertex1).unwrap().get();

                let start_point = Point2 {
                    x: vertex0.x(),
                    y: vertex0.y(),
                };
                let end_point = Point2 {
                    x: vertex1.x(),
                    y: vertex1.y(),
                };
                let cell_id = diagram.edge_get_cell(Some(edge_id)).unwrap();
                let cell = diagram.get_cell(cell_id).get();
                let twin_id = diagram.edge_get_twin(Some(edge_id)).unwrap();
                let twin_cell_id = diagram.edge_get_cell(Some(twin_id)).unwrap();

                let cell_point = if cell.contains_point() {
                    Self::retrieve_point(diagram, cell_id, &self.segments)
                } else {
                    Self::retrieve_point(diagram, twin_cell_id, &self.segments)
                };
                let segment = if cell.contains_point() {
                    Self::retrieve_segment(diagram, twin_cell_id, &self.segments)
                } else {
                    Self::retrieve_segment(diagram, cell_id, &self.segments)
                };

                let segment_start_point = Point2 {
                    x: Self::i2f(segment.start.x),
                    y: Self::i2f(segment.start.y),
                };
                let segment_end_point = Point2 {
                    x: Self::i2f(segment.end.x),
                    y: Self::i2f(segment.end.y),
                };
                let cell_point = Point2 {
                    x: Self::i2f(cell_point.x),
                    y: Self::i2f(cell_point.y),
                };
                let distance_to_start = linestring::cgmath_2d::distance_to_line_squared(
                    &start_point,
                    &segment_start_point,
                    &segment_end_point,
                )
                .sqrt();
                let distance_to_end = linestring::cgmath_2d::distance_to_line_squared(
                    &end_point,
                    &segment_start_point,
                    &segment_end_point,
                )
                .sqrt();

                if edge.is_curved() {
                    let arc = VoronoiParabolicArc::new(
                        Line2 {
                            start: segment_start_point,
                            end: segment_end_point,
                        },
                        cell_point,
                        start_point,
                        end_point,
                    );
                    self.arcs.push(arc);
                } else {
                    // Edge is not curved

                    if distance_to_start.ulps_eq(
                        &F1::zero(),
                        F1::default_epsilon(),
                        F1::default_max_ulps(),
                    ) {
                        println!("distance is zero");
                        continue;
                    }
                    if distance_to_end.ulps_eq(
                        &F1::zero(),
                        F1::default_epsilon(),
                        F1::default_max_ulps(),
                    ) {
                        println!("distance is zero");
                        continue;
                    }
                    let line = Line3 {
                        start: Point3 {
                            x: start_point.x,
                            y: start_point.y,
                            z: distance_to_start,
                        },
                        end: Point3 {
                            x: end_point.x,
                            y: end_point.y,
                            z: distance_to_end,
                        },
                    };
                    self.lines.push(line);
                }
                self.set_edge_rejected(edge_twin_id)
            }
        }
    }

    #[inline(always)]
    pub fn i2f(input: I1) -> F1 {
        num::cast::<I1, F1>(input).unwrap()
    }
}

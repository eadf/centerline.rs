#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![deny(unused_results)]
//#![deny(unused_imports)]
#![allow(unused_imports)]
#![feature(hash_drain_filter)]
#![feature(test)]

#[cfg(test)]
mod bench;

use boostvoronoi::builder as VB;
use boostvoronoi::diagram as VD;
use boostvoronoi::{BigFloatType, BigIntType, InputType, OutputType, TypeConverter};

use boostvoronoi::builder::Builder;
use cgmath::InnerSpace;
use linestring::cgmath_2d;
use linestring::cgmath_2d::{Line2, VoronoiParabolicArc};
use linestring::cgmath_3d;
use linestring::cgmath_3d::{Line3, LineString3, LineStringSet3};
use std::borrow::Borrow;
use std::cell::RefCell;
use std::collections::VecDeque;
use std::fmt::Display;
use std::ops::Neg;
use std::rc::Rc;
use std::sync::{Arc, RwLock};
use thiserror::Error;

#[macro_use]
extern crate bitflags;

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

bitflags! {
    /// bit field defining various reasons for edge/vertex rejection
    pub struct ColorFlag: VD::ColorType {
        /// Edge is directly or indirectly connected to an INFINITE edge
        const EXTERNAL     = 0b00000001;
        /// Edge is secondary
        const SECONDARY    = 0b00000010;
        /// Edge has only one vertex
        const INFINITE     = 0b00000100;
        /// Edge does not pass the normalized edge<->segment dot product test
        const DOTLIMIT     = 0b00001000;
    }
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
    pub diagram: VD::VoronoiDiagram<I1, F1, I2, F2>,
    pub lines: Option<Vec<Line3<F1>>>,
    pub line_strings: Option<Vec<LineString3<F1>>>,

    /// bit field defining edges rejected by EXTERNAL or INFINITE
    rejected_edges: Option<yabf::Yabf>,
    /// bit field defining edges rejected by 'rejected_edges' + dot test
    ignored_edges: Option<yabf::Yabf>,
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
            diagram: VD::VoronoiDiagram::default(),
            segments: Vec::<boostvoronoi::Line<I1>>::default(),
            lines: Some(Vec::<Line3<F1>>::new()),
            line_strings: Some(Vec::<LineString3<F1>>::new()),
            rejected_edges: None,
            ignored_edges: None,
        }
    }

    /// Creates a Centerline container with a set of segments
    pub fn with_segments(segments: Vec<boostvoronoi::Line<I1>>) -> Self {
        Self {
            diagram: VD::VoronoiDiagram::default(),
            segments,
            lines: Some(Vec::<Line3<F1>>::new()),
            line_strings: Some(Vec::<LineString3<F1>>::new()),
            rejected_edges: None,
            ignored_edges: None,
        }
    }

    pub fn build_voronoi(&mut self) -> Result<(), CenterlineError> {
        let mut vb = Builder::new();
        vb.with_segments(self.segments.iter())?;
        self.diagram = vb.construct()?;
        self.reject_edges();
        #[cfg(feature = "console_debug")]
        println!(
            "build_voronoi()-> Rejected edges:{:?} {}",
            self.rejected_edges.as_ref(),
            &self.rejected_edges.as_ref().unwrap().bit(0)
        );
        Ok(())
    }

    pub fn calculate_centerline(
        &mut self,
        dot_limit: F1,
        discrete_limit: F1,
    ) -> Result<(), CenterlineError> {
        self.normalized_dot_test(dot_limit);
        self.traverse_edges(discrete_limit);
        Ok(())
    }

    /// returns a copy of the ignored edges bit field
    pub fn ignored_edges(&self) -> Option<yabf::Yabf> {
        self.ignored_edges.to_owned()
    }

    /// returns a copy of the rejected edges bit field
    pub fn rejected_edges(&self) -> Option<yabf::Yabf> {
        self.rejected_edges.to_owned()
    }

    pub fn retrieve_point(&self, cell_id: VD::VoronoiCellIndex) -> boostvoronoi::Point<I1> {
        let (index, category) = self.diagram.get_cell(cell_id).get().source_index_2();
        match category {
            VD::SourceCategory::SinglePoint => panic!("No points in the input data"),
            VD::SourceCategory::SegmentStart => self.segments[index].start,
            VD::SourceCategory::Segment | VD::SourceCategory::SegmentEnd => {
                self.segments[index].end
            }
        }
    }

    pub fn retrieve_segment(&self, cell_id: VD::VoronoiCellIndex) -> boostvoronoi::Line<I1> {
        let cell = self.diagram.get_cell(cell_id).get();
        self.segments[cell.source_index()]
    }

    pub fn diagram(&self) -> &VD::VoronoiDiagram<I1, F1, I2, F2> {
        &self.diagram
    }

    /// Color exterior edges also rejects some secondary edges
    fn reject_edges(&mut self) {
        let mut rejected_edges = yabf::Yabf::default();
        // ensure capacity of bit field by setting last bit +1 to true
        rejected_edges.set_bit(self.diagram().edges().len(), true);

        for it in self.diagram.edges().iter() {
            let edge = it.get();
            let edge_id = edge.get_id();
            let edge_sid = Some(edge_id);
            if edge.is_secondary() {
                rejected_edges.set_bit(edge_id.0, true);
                self.diagram
                    .edge_or_color(edge_sid, ColorFlag::SECONDARY.bits);
                let twin_sid = self.diagram.edge_get_twin(edge_sid);
                self.diagram
                    .edge_or_color(twin_sid, ColorFlag::SECONDARY.bits);
                if let Some(twin_id) = twin_sid {
                    rejected_edges.set_bit(twin_id.0, true);
                }
            }
            if !self.diagram.edge_is_finite(edge_sid).unwrap() {
                self.recursively_reject_edge(edge_sid, ColorFlag::EXTERNAL, &mut rejected_edges);

                self.diagram
                    .edge_or_color(edge_sid, ColorFlag::INFINITE.bits);
                rejected_edges.set_bit(edge_id.0, true);
            }
        }

        self.rejected_edges = Some(rejected_edges);
    }

    /// Reject edges that does not pass the dot limit test.
    /// It iterates over all cells, looking for vertices that are identical to the
    /// input segment endpoints.
    /// It then look at edges connected to that vertex and test if the dot product
    /// between the normalized segment vector and normalized edge vector exceeds
    /// a predefined value.
    /// TODO: there must be a quicker way to get this information from the voronoi diagram
    /// maybe mark each vertex identical to input points..
    fn normalized_dot_test(&mut self, dot_limit: F1) {
        let mut ignored_edges = self.rejected_edges.clone().take().unwrap();

        for (c_id, cell) in self.diagram.cells().iter().enumerate() {
            let cell_c = cell.get();
            assert_eq!(c_id, cell_c.get_id());
            if !cell_c.contains_segment() {
                continue;
            }
            let segment = self.retrieve_segment(VD::VoronoiCellIndex(c_id));
            let point0 = cgmath::Point2 {
                x: Self::i2f(segment.start.x),
                y: Self::i2f(segment.start.y),
            };
            let point1 = cgmath::Point2 {
                x: Self::i2f(segment.end.x),
                y: Self::i2f(segment.end.y),
            };

            if let Some(incident_e) = cell_c.get_incident_edge() {
                //println!("incident_e {:?}", incident_e);
                let mut e = Some(incident_e);
                loop {
                    e = self.diagram.get_edge(e.unwrap()).get().next();

                    if !Self::is_edge_rejected(e, &ignored_edges) {
                        // all infinite edges should be rejected at this point, so
                        // all edges should contain a vertex0 and vertex1

                        let vertex0 = self.diagram.edge_get_vertex0(e);
                        if let Some(vertex0) = self.diagram.vertex_get(vertex0) {
                            let vertex0 = vertex0.get();
                            let vertex0 = cgmath::Point2 {
                                x: vertex0.x(),
                                y: vertex0.y(),
                            };
                            let vertex1 = self.diagram.edge_get_vertex1(e);
                            if let Some(vertex1) = self.diagram.vertex_get(vertex1) {
                                let vertex1 = vertex1.get();
                                let vertex1 = cgmath::Point2 {
                                    x: vertex1.x(),
                                    y: vertex1.y(),
                                };
                                let _ = self.normalized_dot_test_6(
                                    dot_limit,
                                    &mut ignored_edges,
                                    e,
                                    &vertex0,
                                    &vertex1,
                                    &point0,
                                    &point1,
                                ) || self.normalized_dot_test_6(
                                    dot_limit,
                                    &mut ignored_edges,
                                    e,
                                    &vertex0,
                                    &vertex1,
                                    &point1,
                                    &point0,
                                ) || self.normalized_dot_test_6(
                                    dot_limit,
                                    &mut ignored_edges,
                                    e,
                                    &vertex1,
                                    &vertex0,
                                    &point0,
                                    &point1,
                                ) || self.normalized_dot_test_6(
                                    dot_limit,
                                    &mut ignored_edges,
                                    e,
                                    &vertex1,
                                    &vertex0,
                                    &point1,
                                    &point0,
                                );
                            }
                        }
                    }
                    if e.is_none() || e.unwrap() == incident_e {
                        break;
                    }
                }
            }
        }
        self.ignored_edges = Some(ignored_edges);
    }

    /// set the edge as rejected if it fails the dot test
    #[allow(clippy::too_many_arguments)]
    fn normalized_dot_test_6(
        &self,
        dot_limit: F1,
        ignored_edges: &mut yabf::Yabf,
        edge: Option<VD::VoronoiEdgeIndex>,
        vertex0: &cgmath::Point2<F1>,
        vertex1: &cgmath::Point2<F1>,
        s_point_0: &cgmath::Point2<F1>,
        s_point_1: &cgmath::Point2<F1>,
    ) -> bool {
        if approx::ulps_eq!(vertex0.x, s_point_0.x) && approx::ulps_eq!(vertex0.y, s_point_0.y) {
            // todo better to compare to the square of the dot product, fewer operations.
            let segment_v = (s_point_1 - s_point_0).normalize();
            let vertex_v = (vertex1 - vertex0).normalize();
            let dot_product = segment_v.dot(vertex_v).abs();
            if dot_product < dot_limit {
                if let Some(twin) = self.diagram.edge_get_twin(edge) {
                    ignored_edges.set_bit(twin.0, true);
                }
                if let Some(edge) = edge {
                    ignored_edges.set_bit(edge.0, true);
                }
                return true;
            }
        }
        false
    }

    /// check if an edge is rejected based on the ignored_edges map
    #[inline(always)]
    fn is_edge_rejected(edge_id: Option<VD::VoronoiEdgeIndex>, ignored_edges: &yabf::Yabf) -> bool {
        if let Some(edge_id_u) = edge_id {
            ignored_edges.bit(edge_id_u.0)
        } else {
            true
        }
    }

    /// Set the color value of the edge
    #[inline(always)]
    fn reject_edge(
        &self,
        edge_id: Option<VD::VoronoiEdgeIndex>,
        color: ColorFlag,
        ignored_edges: &mut yabf::Yabf,
    ) {
        self.diagram.edge_or_color(edge_id, color.bits);
        if let Some(edge_id) = edge_id {
            ignored_edges.set_bit(edge_id.0, true);
        }
    }

    /*
    #[allow(dead_code)]
    #[inline(always)]
    fn is_vertex_rejected(&self, vertex_id: Option<VD::VoronoiVertexIndex>) -> bool {
        self.diagram
            .vertex_get_color(vertex_id)
            .map_or(true, |x| x != 0)
    }*/

    #[inline(always)]
    fn reject_vertex(&self, vertex_id: Option<VD::VoronoiVertexIndex>, color: ColorFlag) {
        self.diagram.vertex_or_color(vertex_id, color.bits);
    }

    /// Recursively marks this edge and all other edges connecting to it as rejected.
    /// Recursion stops when connecting to input geometry.
    fn recursively_reject_edge(
        &self,
        edge_id: Option<VD::VoronoiEdgeIndex>,
        color: ColorFlag,
        ignored_edges: &mut yabf::Yabf,
    ) {
        if edge_id.is_none() || Self::is_edge_rejected(edge_id, ignored_edges) {
            return;
        }
        self.reject_edge(edge_id, color, ignored_edges);
        self.reject_edge(self.diagram.edge_get_twin(edge_id), color, ignored_edges);
        let v = self.diagram.edge_get_vertex1(edge_id);
        if v.is_none() || !self.diagram.get_edge(edge_id.unwrap()).get().is_primary() {
            // break recursion if vertex1 is not found or it the edge is primary
            return;
        }
        self.reject_vertex(v, color);
        let mut e = self.diagram.vertex_get_incident_edge(v);
        let v_incident_edge = e;
        while e.is_some() {
            self.recursively_reject_edge(e, color, ignored_edges);
            e = self.diagram.edge_rot_next(e);
            if e == v_incident_edge {
                break;
            }
        }
    }

    /// move across each edge and sample the lines and arcs
    fn traverse_edges(&mut self, maxdist: F1) {
        // de-couple self and containers
        let mut lines = self.lines.take().unwrap();
        let mut linestrings = self.line_strings.take().unwrap();

        // operate on a copy of self.ignored_edges
        let mut ignored_edges = self.ignored_edges.clone().take().unwrap();
        let mut used_edges = ignored_edges.clone();

        linestrings.clear();
        lines.clear();

        for it in self.diagram.edges().iter().enumerate() {
            let edge_id = VD::VoronoiEdgeIndex(it.0);
            assert_eq!(edge_id.0, it.0);

            // could not use that an iter().filter() because of the BC
            if used_edges.bit(edge_id.0) {
                continue;
            }

            self.traverse_edge(
                edge_id,
                &mut ignored_edges,
                &mut used_edges,
                &mut lines,
                &mut linestrings,
                maxdist,
            );
        }

        #[cfg(feature = "console_debug")]
        {
            println!("Got {} single lines", lines.len());
            println!("Got {} linestrings", linestrings.len());
            println!(
                "self.ignored_edges {:?}",
                self.ignored_edges.clone().unwrap()
            );
            println!("     ignored_edges {:?}", ignored_edges);
            println!("        used_edges {:?}", used_edges);
        }
        // put the containers back
        self.lines = Some(lines);
        self.line_strings = Some(linestrings);
    }

    /// Mark an edge and it's twin as used/rejected
    #[inline(always)]
    fn mark_edge_and_twin_as_used(
        &self,
        edge_id: VD::VoronoiEdgeIndex,
        used_edges: &mut yabf::Yabf,
    ) {
        used_edges.set_bit(edge_id.0, true);
        #[cfg(feature = "console_debug")]
        print!("marking {}", edge_id.0);
        if let Some(twin) = self.diagram.edge_get_twin(Some(edge_id)) {
            #[cfg(feature = "console_debug")]
            print!(" & {}", twin.0);
            if used_edges.bit(twin.0) {
                print!(" TWIN was already used!!!!!");
            }
            used_edges.set_bit(twin.0, true);
        }
    }

    /// move across each adjacent edge and sample the lines and arcs
    fn traverse_edge(
        &self,
        seed_edge: VD::VoronoiEdgeIndex,
        ignored_edges: &mut yabf::Yabf,
        used_edges: &mut yabf::Yabf,
        lines: &mut Vec<Line3<F1>>,
        linestrings: &mut Vec<LineString3<F1>>,
        maxdist: F1,
    ) {
        #[cfg(feature = "console_debug")]
        println!();
        #[cfg(feature = "console_debug")]
        println!("->traverse_edge({})", seed_edge.0);
        #[cfg(feature = "console_debug")]
        let mut mockup = Vec::<Vec<VD::VoronoiEdgeIndex>>::default();

        let count = self
            .diagram
            .edge_rot_next_iterator(Some(seed_edge))
            .filter(|x| !ignored_edges.bit(x.0))
            .count();
        if count == 1 {
            let mut start_points = VecDeque::<VD::VoronoiEdgeIndex>::default();
            let mut current_edge_set = Vec::<VD::VoronoiEdgeIndex>::new();
            start_points.push_front(seed_edge);

            while !start_points.is_empty() {
                #[cfg(feature = "console_debug")]
                println!();
                let edge = start_points.pop_front().unwrap();
                #[cfg(feature = "console_debug")]
                {
                    let v0 = self.diagram().edge_get_vertex0(Some(edge));
                    let v0 = self.diagram().vertex_get(v0).unwrap().get();

                    let v1 = self.diagram().edge_get_vertex1(Some(edge));
                    let v1 = self.diagram().vertex_get(v1).unwrap().get();

                    print!(
                        "now looking at {} i:{} u:{} ({:5.3},{:.3})->({:.3},{:.3})",
                        edge.0,
                        ignored_edges.bit(edge.0),
                        used_edges.bit(edge.0),
                        v0.x(),
                        v0.y(),
                        v1.x(),
                        v1.y(),
                    );
                }

                if ignored_edges.bit(edge.0) {
                    // Should never happen
                    panic!();
                }
                if used_edges.bit(edge.0) {
                    #[cfg(feature = "console_debug")]
                    print!(" skip");
                    // edge was already processed, continue
                    continue;
                }
                #[cfg(feature = "console_debug")]
                println!();

                current_edge_set.push(edge);
                self.mark_edge_and_twin_as_used(edge, used_edges);

                let mut next_edge_o = self.diagram.edge_get_next(Some(edge));
                while next_edge_o.is_some() {
                    #[cfg(feature = "console_debug")]
                    print!("Inner loop next_edge={} ", next_edge_o.unwrap().0);

                    // it does not matter if next_edge is rejected/valid, it will be fixed by the iterator
                    let next_edges: Vec<VD::VoronoiEdgeIndex> = self
                        .diagram
                        .edge_rot_next_iterator(next_edge_o)
                        .filter(|x| !ignored_edges.bit(x.0))
                        .collect();

                    #[cfg(feature = "console_debug")]
                    {
                        print!("candidates[");

                        for ne in next_edges.iter() {
                            if used_edges.bit(ne.0) {
                                print!("!");
                            }
                            print!("{},", ne.0);
                        }
                        println!("]");
                    }
                    match next_edges.len() {
                        1 | 2 => {
                            let next_edges: Vec<VD::VoronoiEdgeIndex> = next_edges
                                .into_iter()
                                .filter(|x| !used_edges.bit(x.0))
                                .collect();
                            if next_edges.len() == 1 {
                                // continue walking the edge line
                                let e = next_edges.first().unwrap().to_owned();
                                current_edge_set.push(e);
                                self.mark_edge_and_twin_as_used(e, used_edges);

                                next_edge_o = self.diagram.edge_get_next(Some(e));
                            } else {
                                // terminating the line string, pushing candidates
                                self.convert_edges_to_lines(
                                    &current_edge_set,
                                    lines,
                                    linestrings,
                                    maxdist,
                                );
                                #[cfg(feature = "console_debug")]
                                mockup.push(current_edge_set.clone());
                                current_edge_set.clear();
                                next_edge_o = None;
                                if !next_edges.is_empty() {
                                    #[cfg(feature = "console_debug")]
                                    print!("1|2 Pushing new start points: [");
                                    for e in next_edges.iter() {
                                        if !ignored_edges.bit(e.0) && !used_edges.bit(e.0) {
                                            #[cfg(feature = "console_debug")]
                                            print!("{},", e.0);
                                            start_points.push_back(*e);
                                        }
                                    }
                                }
                                #[cfg(feature = "console_debug")]
                                {
                                    println!("]");
                                    println!("1|2 Starting new set");
                                }
                            }
                            continue;
                        }
                        _ => {
                            // to many or too few intersections found, end this linestring and push the new candidates to the queue
                            self.convert_edges_to_lines(
                                &current_edge_set,
                                lines,
                                linestrings,
                                maxdist,
                            );
                            if !next_edges.is_empty() {
                                #[cfg(feature = "console_debug")]
                                print!("0|_ Pushing new start points: [");
                                for e in next_edges.iter() {
                                    if !ignored_edges.bit(e.0) && !used_edges.bit(e.0) {
                                        #[cfg(feature = "console_debug")]
                                        print!("{},", e.0);
                                        start_points.push_back(*e);
                                    }
                                }
                                #[cfg(feature = "console_debug")]
                                println!("]");
                            }
                            #[cfg(feature = "console_debug")]
                            mockup.push(current_edge_set.clone());
                            current_edge_set.clear();
                            next_edge_o = None;
                            #[cfg(feature = "console_debug")]
                            println!("0|_ Starting new set");

                            continue;
                        }
                    }
                }
            }
            #[cfg(feature = "console_debug")]
            {
                println!("mockups");
                for m in mockup.iter() {
                    println!("mockup {:?}", m.iter().map(|x| x.0).collect::<Vec<usize>>());
                }
            }
        } else {
            #[cfg(feature = "console_debug")]
            println!(
                "<-traverse_edge({}) ignoring start edge, count={} {:?}",
                seed_edge.0,
                count,
                self.diagram
                    .edge_rot_next_iterator(Some(seed_edge))
                    .filter(|x| !ignored_edges.bit(x.0 as u64))
                    .map(|x| x.0)
                    .collect::<Vec<usize>>()
            );
        }
    }

    fn convert_edges_to_lines(
        &self,
        edges: &[VD::VoronoiEdgeIndex],
        lines: &mut Vec<Line3<F1>>,
        linestrings: &mut Vec<LineString3<F1>>,
        maxdist: F1,
    ) {
        #[cfg(feature = "console_debug")]
        println!();
        #[cfg(feature = "console_debug")]
        println!(
            "Converting {:?} to lines",
            edges.iter().map(|x| x.0).collect::<Vec<usize>>()
        );
        match edges.len() {
            0 => panic!(),
            1 => {
                let edge_id = edges.first().unwrap();
                let edge = self.diagram.get_edge(*edge_id).get();
                match self.convert_edge_to_shape(&edge) {
                    cgmath_3d::Shape3d::Line(l) => lines.push(l),
                    cgmath_3d::Shape3d::ParabolicArc(a) => {
                        linestrings.push(a.discretise_3d(maxdist));
                    }
                    cgmath_3d::Shape3d::Linestring(_s) => {
                        panic!();
                    }
                }
            }
            _ => {
                let mut ls = LineString3::<F1>::default();
                for edge_id in edges.iter() {
                    let edge = self.diagram.get_edge(*edge_id).get();
                    match self.convert_edge_to_shape(&edge) {
                        cgmath_3d::Shape3d::Line(l) => {
                            //println!("->got {:?}", l);
                            ls.push(l.start);
                            ls.push(l.end);
                            //println!("<-got");
                        }
                        cgmath_3d::Shape3d::ParabolicArc(a) => {
                            //println!("->got {:?}", a);
                            ls.append(a.discretise_3d(maxdist));
                            //println!("<-got");
                        }
                        // should not happen
                        cgmath_3d::Shape3d::Linestring(_s) => panic!(),
                    }
                }
                linestrings.push(ls);
            }
        }
        //println!("Converted {:?} to lines", edges);
    }

    fn convert_edge_to_shape(
        &self,
        edge: &VD::VoronoiEdge<I1, F1, I2, F2>,
    ) -> cgmath_3d::Shape3d<F1> {
        //println!("Converting {:?} to line", edge.get_id());
        let edge_id = Some(edge.get_id());
        let edge_twin_id = self.diagram.edge_get_twin(edge_id);

        // Edge is finite so we know that vertex0 and vertex1 is_some()
        let vertex0 = self.diagram.vertex_get(edge.vertex0()).unwrap().get();

        let vertex1 = self.diagram.edge_get_vertex1(edge_id);
        let vertex1 = self.diagram.vertex_get(vertex1).unwrap().get();

        let start_point = cgmath::Point2 {
            x: vertex0.x(),
            y: vertex0.y(),
        };
        let end_point = cgmath::Point2 {
            x: vertex1.x(),
            y: vertex1.y(),
        };
        let cell_id = self.diagram.edge_get_cell(edge_id).unwrap();
        let cell = self.diagram.get_cell(cell_id).get();
        let twin_cell_id = self.diagram.edge_get_cell(edge_twin_id).unwrap();

        let cell_point = if cell.contains_point() {
            self.retrieve_point(cell_id)
        } else {
            self.retrieve_point(twin_cell_id)
        };
        let segment = if cell.contains_point() {
            self.retrieve_segment(twin_cell_id)
        } else {
            self.retrieve_segment(cell_id)
        };

        let segment_start_point = cgmath::Point2 {
            x: Self::i2f(segment.start.x),
            y: Self::i2f(segment.start.y),
        };
        let segment_end_point = cgmath::Point2 {
            x: Self::i2f(segment.end.x),
            y: Self::i2f(segment.end.y),
        };
        let cell_point = cgmath::Point2 {
            x: Self::i2f(cell_point.x),
            y: Self::i2f(cell_point.y),
        };
        let distance_to_start = linestring::cgmath_2d::distance_to_line_squared(
            &segment_start_point,
            &segment_end_point,
            &start_point,
        )
        .sqrt();
        if distance_to_start.is_nan() {
            println!(
                "start_point:{:?}, segment_start_point:{:?}, segment_end_point:{:?} => NaN",
                start_point, segment_start_point, segment_end_point
            );
        }
        let distance_to_end = linestring::cgmath_2d::distance_to_line_squared(
            &segment_start_point,
            &segment_end_point,
            &end_point,
        )
        .sqrt();
        if distance_to_end.is_nan() {
            println!(
                "end_point:{:?}, segment_start_point:{:?}, segment_end_point:{:?} => NaN",
                end_point, segment_start_point, segment_end_point
            );
        }
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
            #[cfg(feature = "console_debug")]
            println!("Converted {:?} to {:?}", edge.get_id().0, arc);
            cgmath_3d::Shape3d::ParabolicArc(arc)
        } else {
            let line = Line3 {
                start: cgmath::Point3 {
                    x: start_point.x,
                    y: start_point.y,
                    z: distance_to_start,
                },
                end: cgmath::Point3 {
                    x: end_point.x,
                    y: end_point.y,
                    z: distance_to_end,
                },
            };
            #[cfg(feature = "console_debug")]
            println!("Converted {:?} to {:?}", edge.get_id().0, line);
            cgmath_3d::Shape3d::Line(line)
        }
    }

    #[inline(always)]
    pub fn i2f(input: I1) -> F1 {
        num::cast::<I1, F1>(input).unwrap()
    }
}

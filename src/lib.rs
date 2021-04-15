#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![deny(unused_results)]
#![deny(unused_imports)]
#![deny(unused_imports)]
#![feature(hash_drain_filter)]
//#![feature(test)]

use boostvoronoi::builder as VB;
use boostvoronoi::diagram as VD;
use boostvoronoi::sync_diagram as VSD;
use boostvoronoi::{InputType, OutputType};

//use num_traits::Float;
use cgmath::InnerSpace;
use linestring::cgmath_2d;
#[allow(unused_imports)]
use linestring::cgmath_2d::convex_hull;
use linestring::cgmath_3d;
use linestring::cgmath_3d::{Line3, LineString3, LineStringSet3};
use std::collections::VecDeque;
use std::ops::Neg;
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

    #[error(transparent)]
    LinestringError(#[from] linestring::LinestringError),
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
    id: usize,            // index into the point3 list
    edges: Vec<usize>,    // list of edges this vertex is part of
    shape: Option<usize>, // shape id
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

#[allow(clippy::type_complexity)]
/// remove internal edges from a wavefront-obj object
pub fn remove_internal_edges(
    obj: obj::raw::RawObj,
) -> Result<(fnv::FnvHashSet<(usize, usize)>, Vec<cgmath::Point3<f32>>), CenterlineError> {
    for p in obj.points.iter() {
        // Ignore all points
        println!("Ignored point:{:?}", p);
    }
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
    let _ = all_edges.drain_filter(|x| internal_edges.contains(x));
    // all_edges should now contain the outline and none of the internal edges.
    let vertices: Vec<cgmath::Point3<f32>> = obj
        .positions
        .into_iter()
        .map(|x| cgmath::Point3 {
            x: x.0,
            y: x.1,
            z: x.2,
        })
        .collect();

    Ok((all_edges, vertices))
}

/// Group input edges into connected shapes
pub fn divide_into_shapes<T>(
    edge_set: fnv::FnvHashSet<(usize, usize)>,
    points: Vec<cgmath::Point3<T>>,
) -> Result<Vec<LineStringSet3<T>>, CenterlineError>
where
    T: cgmath::BaseFloat + Sync,
{
    //println!("All edges: {:?}", all_edges);
    // put all edges into a hashmap of Vertices, this will make it possible to
    // arrange them in the order they are connected
    let mut vertices = fnv::FnvHashMap::<usize, Vertices>::default();
    for e in edge_set.iter() {
        let id = e.0;
        let other = e.1;
        if let Some(v) = vertices.get_mut(&id) {
            v.edges.push(other);
        } else {
            let _ = vertices.insert(
                id,
                Vertices {
                    id,
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
                    edges: vec![other],
                    shape: None,
                },
            );
        }
    }
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
    // Spit all detected connected vertices into separate sets.
    // i.e. every vertex with the same color goes into the same set.
    let mut shape_iter = 0..usize::MAX;
    let mut shape_separation = Vec::<fnv::FnvHashMap<usize, Vertices>>::new();
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
    // now we have a list of groups of vertices, each group are connected by edges.
    // Create lists of linestrings3 by walking the edges of each vertex set.
    let mut rv = Vec::<LineStringSet3<T>>::with_capacity(shape_separation.len());

    for rvi in shape_separation.iter() {
        if rvi.is_empty() {
            continue;
        }

        let mut rvs = cgmath_3d::LineStringSet3::<T>::with_capacity(shape_separation.len());
        let mut als = cgmath_3d::LineString3::<T>::with_capacity(rvi.len());

        let started_with: usize = rvi.iter().next().unwrap().1.id;
        let mut prev: usize;
        let mut current: usize = started_with;
        let mut next: usize = started_with;
        let mut first_loop = true;

        loop {
            prev = current;
            current = next;
            if let Some(current_vertex) = rvi.get(&current) {
                als.push(points[current]);

                //assert_eq!(newV.edges.len(),2);
                next = *current_vertex.edges.iter().find(|x| **x != prev).unwrap();
            } else {
                break;
            }
            // allow the start point to be added twice (in case of a loop)
            if !first_loop && current == started_with {
                break;
            }
            first_loop = false;
        }
        if als.points().last() != als.points().first() {
            println!(
                "Linestring is not connected ! {:?} {:?}",
                als.points().first(),
                als.points().last()
            );
            println!("Linestring is not connected ! {:?}", als.points());
        }
        rvs.push(als);
        rv.push(rvs);
    }
    Ok(rv)
}

/// Center line calculation object.
/// It: * calculates the segmented voronoi diagram.
///     * Filter out voronoi edges based on the angle to input geometry.
///     * Collects connected edges into line strings and line segments.
///     * Performs line simplification on those line strings.
pub struct Centerline<I1, F1>
where
    I1: InputType + Neg<Output = I1>,
    F1: cgmath::BaseFloat + Sync + OutputType + Neg<Output = F1>,
{
    /// the input data to the voronoi diagram
    pub segments: Vec<boostvoronoi::Line<I1>>,
    /// the voronoi diagram itself
    pub diagram: VSD::SyncVoronoiDiagram<I1, F1>,
    /// the individual two-point edges
    pub lines: Option<Vec<Line3<F1>>>,
    /// concatenated connected edges
    pub line_strings: Option<Vec<LineString3<F1>>>,

    /// bit field defining edges rejected by EXTERNAL or INFINITE
    rejected_edges: Option<yabf::Yabf>,
    /// bit field defining edges rejected by 'rejected_edges' + dot test
    ignored_edges: Option<yabf::Yabf>,

    #[cfg(feature = "console_debug")]
    pub debug_edges: Option<fnv::FnvHashMap<usize, [F1; 4]>>,
}

impl<I1, F1> Centerline<I1, F1>
where
    I1: InputType + Neg<Output = I1>,
    F1: cgmath::BaseFloat + Sync + OutputType + Neg<Output = F1>,
{
    /// Creates a Centerline container with a set of segments
    pub fn default() -> Self {
        Self {
            diagram: VSD::SyncVoronoiDiagram::default(),
            segments: Vec::<boostvoronoi::Line<I1>>::default(),
            lines: Some(Vec::<Line3<F1>>::new()),
            line_strings: Some(Vec::<LineString3<F1>>::new()),
            rejected_edges: None,
            ignored_edges: None,
            #[cfg(feature = "console_debug")]
            debug_edges: None,
        }
    }

    /// Creates a Centerline container with a set of segments
    pub fn with_segments(segments: Vec<boostvoronoi::Line<I1>>) -> Self {
        Self {
            diagram: VSD::SyncVoronoiDiagram::default(),
            segments,
            lines: Some(Vec::<Line3<F1>>::new()),
            line_strings: Some(Vec::<LineString3<F1>>::new()),
            rejected_edges: None,
            ignored_edges: None,
            #[cfg(feature = "console_debug")]
            debug_edges: None,
        }
    }

    /// builds the voronoi diagram and filter out infinite edges and other 'outside' geometry
    pub fn build_voronoi(&mut self) -> Result<(), CenterlineError> {
        let mut vb = VB::Builder::default();
        #[cfg(feature = "console_debug")]
        {
            print!("build_voronoi()-> input segments:[");
            for s in self.segments.iter() {
                print!("[{},{},{},{}],", s.start.x, s.start.y, s.end.x, s.end.y);
            }
            println!("];");
        }
        vb.with_segments(self.segments.iter())?;
        self.diagram = vb.construct()?.into();
        self.reject_external_edges()?;
        #[cfg(feature = "console_debug")]
        println!(
            "build_voronoi()-> Rejected edges:{:?} {}",
            self.rejected_edges.as_ref(),
            &self.rejected_edges.as_ref().unwrap().bit(0)
        );
        Ok(())
    }

    /// perform the angle-to-geometry test and filter out some edges.
    /// Collect the rest of the edges into connected line-strings and line segments.
    pub fn calculate_centerline(
        &mut self,
        dot_limit: F1,
        discrete_limit: F1,
        ignored_regions: Option<
            &Vec<(
                linestring::cgmath_2d::Aabb2<F1>,
                linestring::cgmath_2d::LineString2<F1>,
            )>,
        >,
    ) -> Result<(), CenterlineError> {
        self.normalized_dot_test(dot_limit)?;
        if let Some(ignored_regions) = ignored_regions {
            self.traverse_edges(discrete_limit, ignored_regions)?;
        } else {
            let ignored_regions = Vec::<(
                linestring::cgmath_2d::Aabb2<F1>,
                linestring::cgmath_2d::LineString2<F1>,
            )>::with_capacity(0);
            self.traverse_edges(discrete_limit, &ignored_regions)?;
        }
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

    pub fn retrieve_point(
        &self,
        cell_id: VD::VoronoiCellIndex,
    ) -> Result<boostvoronoi::Point<I1>, CenterlineError> {
        let (index, category) = self.diagram.cell_get(cell_id)?.source_index_2();
        match category {
            VD::SourceCategory::SinglePoint => panic!("No points in the input data"),
            VD::SourceCategory::SegmentStart => Ok(self.segments[index].start),
            VD::SourceCategory::Segment | VD::SourceCategory::SegmentEnd => {
                Ok(self.segments[index].end)
            }
        }
    }

    pub fn retrieve_segment(
        &self,
        cell_id: VD::VoronoiCellIndex,
    ) -> Result<boostvoronoi::Line<I1>, CenterlineError> {
        let cell = self.diagram.cell_get(cell_id)?;
        Ok(self.segments[cell.source_index()])
    }

    /// returns a reference to the internal voronoi diagram
    pub fn diagram(&self) -> &VSD::SyncVoronoiDiagram<I1, F1> {
        &self.diagram
    }

    /// Mark infinite edges and their adjacent edges as EXTERNAL.
    fn reject_external_edges(&mut self) -> Result<(), CenterlineError> {
        let mut rejected_edges = yabf::Yabf::default();
        // ensure capacity of bit field by setting last bit +1 to true
        rejected_edges.set_bit(self.diagram().edges().len(), true);

        for edge in self.diagram.edges().iter() {
            let edge_id = edge.get_id();
            if edge.is_secondary() {
                rejected_edges.set_bit(edge_id.0, true);
                //self.diagram
                //    .edge_or_color(edge_id, ColorFlag::SECONDARY.bits)?;
                let twin_id = self.diagram.edge_get_twin_err(edge_id)?;
                //self.diagram
                //    .edge_or_color(twin_id, ColorFlag::SECONDARY.bits);
                rejected_edges.set_bit(twin_id.0, true);
            }
            if !self.diagram.edge_is_finite(edge_id)? {
                self.recursively_mark_connected_edges(edge_id, &mut rejected_edges, false)?;

                //self.diagram
                //    .edge_or_color(edge_id, ColorFlag::INFINITE.bits);
                rejected_edges.set_bit(edge_id.0, true);
            }
        }

        self.rejected_edges = Some(rejected_edges);
        Ok(())
    }

    /// Reject edges that does not pass the dot limit test.
    /// It iterates over all cells, looking for vertices that are identical to the
    /// input segment endpoints.
    /// It then look at edges connected to that vertex and test if the dot product
    /// between the normalized segment vector and normalized edge vector exceeds
    /// a predefined value.
    /// TODO: there must be a quicker way to get this information from the voronoi diagram
    /// maybe mark each vertex identical to input points..
    fn normalized_dot_test(&mut self, dot_limit: F1) -> Result<(), CenterlineError> {
        let mut ignored_edges = self.rejected_edges.clone().take().unwrap();

        for cell in self.diagram.cells().iter() {
            let cell_id = VD::VoronoiCellIndex(cell.get_id());

            if !cell.contains_segment() {
                continue;
            }
            let segment = self.retrieve_segment(cell_id)?;
            let point0 = cgmath::Point2 {
                x: Self::i2f(segment.start.x),
                y: Self::i2f(segment.start.y),
            };
            let point1 = cgmath::Point2 {
                x: Self::i2f(segment.end.x),
                y: Self::i2f(segment.end.y),
            };

            if let Some(incident_e) = cell.get_incident_edge() {
                //println!("incident_e {:?}", incident_e);
                let mut e: Option<VD::VoronoiEdgeIndex> = Some(incident_e);
                loop {
                    e = self.diagram.edge_get(e.unwrap())?.next();

                    if let Some(e) = e {
                        if !ignored_edges.bit(e.0) {
                            // all infinite edges should be rejected at this point, so
                            // all edges should contain a vertex0 and vertex1

                            let vertex0 = self.diagram.edge_get_vertex0(e)?;
                            let vertex0 = vertex0.map(|x| self.diagram.vertex_get(x));

                            if let Some(Ok(vertex0)) = vertex0 {
                                let vertex0 = cgmath::Point2 {
                                    x: vertex0.x(),
                                    y: vertex0.y(),
                                };
                                let vertex1 = self.diagram.edge_get_vertex1(e)?;
                                let vertex1 = vertex1.map(|x| self.diagram.vertex_get(x));
                                if let Some(Ok(vertex1)) = vertex1 {
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
                                    )? || self.normalized_dot_test_6(
                                        dot_limit,
                                        &mut ignored_edges,
                                        e,
                                        &vertex0,
                                        &vertex1,
                                        &point1,
                                        &point0,
                                    )? || self.normalized_dot_test_6(
                                        dot_limit,
                                        &mut ignored_edges,
                                        e,
                                        &vertex1,
                                        &vertex0,
                                        &point0,
                                        &point1,
                                    )? || self.normalized_dot_test_6(
                                        dot_limit,
                                        &mut ignored_edges,
                                        e,
                                        &vertex1,
                                        &vertex0,
                                        &point1,
                                        &point0,
                                    )?;
                                }
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
        Ok(())
    }

    /// set the edge as rejected if it fails the dot test
    #[allow(clippy::too_many_arguments)]
    fn normalized_dot_test_6(
        &self,
        dot_limit: F1,
        ignored_edges: &mut yabf::Yabf,
        edge_id: VD::VoronoiEdgeIndex,
        vertex0: &cgmath::Point2<F1>,
        vertex1: &cgmath::Point2<F1>,
        s_point_0: &cgmath::Point2<F1>,
        s_point_1: &cgmath::Point2<F1>,
    ) -> Result<bool, CenterlineError> {
        if approx::ulps_eq!(vertex0.x, s_point_0.x) && approx::ulps_eq!(vertex0.y, s_point_0.y) {
            // todo better to compare to the square of the dot product, fewer operations.
            let segment_v = (s_point_1 - s_point_0).normalize();
            let vertex_v = (vertex1 - vertex0).normalize();
            if segment_v.dot(vertex_v).abs() < dot_limit {
                if let Some(twin) = self.diagram.edge_get(edge_id)?.twin() {
                    ignored_edges.set_bit(twin.0, true);
                }
                ignored_edges.set_bit(edge_id.0, true);
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Recursively marks this edge and all other edges connecting to it via vertex1.
    /// Recursion stops when connecting to input geometry.
    /// if 'initial' is set to true it will search both ways, (edge and the twin edge)
    fn recursively_mark_connected_edges(
        &self,
        edge_id: VD::VoronoiEdgeIndex,
        marked_edges: &mut yabf::Yabf,
        initial: bool,
    ) -> Result<(), CenterlineError> {
        if marked_edges.bit(edge_id.0) {
            return Ok(());
        }

        let v1 = self.diagram.edge_get_vertex1(edge_id)?;
        if self.diagram.edge_get_vertex0(edge_id)?.is_some() && v1.is_none() {
            // this edge leads to nowhere, break recursion
            marked_edges.set_bit(edge_id.0, true);
            return Ok(());
        }
        marked_edges.set_bit(edge_id.0, true);

        if initial {
            self.recursively_mark_connected_edges(
                self.diagram.edge_get_twin_err(edge_id)?,
                marked_edges,
                false,
            )?;
        } else {
            marked_edges.set_bit(self.diagram.edge_get_twin_err(edge_id)?.0, true);
        }

        if v1.is_none() || !self.diagram.edge_get(edge_id)?.is_primary() {
            // break recursion if vertex1 is not found or if the edge is not primary
            return Ok(());
        }
        // v1 is always Some from this point on
        if let Some(v1) = v1 {
            let v1 = self.diagram.vertex_get(v1)?;
            if v1.is_site_point() {
                // break recursion on site points
                return Ok(());
            }
            //self.reject_vertex(v1, color);
            let mut e = v1.get_incident_edge();
            let v_incident_edge = e;
            while let Some(this_edge) = e {
                self.recursively_mark_connected_edges(this_edge, marked_edges, false)?;
                e = self.diagram.edge_rot_next(this_edge)?;
                if e == v_incident_edge {
                    break;
                }
            }
        }
        Ok(())
    }

    /// returns true if *all* of the 'edges' are contained inside one of the 'ignored_regions'
    fn edges_are_inside_ignored_region(
        &self,
        edges: &yabf::Yabf,
        ignored_regions: &[(
            linestring::cgmath_2d::Aabb2<F1>,
            linestring::cgmath_2d::LineString2<F1>,
        )],
    ) -> Result<bool, CenterlineError> {
        let is_inside_region = |edge: VD::VoronoiEdgeIndex,
                                region: &(
            linestring::cgmath_2d::Aabb2<F1>,
            linestring::cgmath_2d::LineString2<F1>,
        )|
         -> Result<bool, CenterlineError> {
            let v0 = self.diagram.edge_get_vertex0(edge)?.unwrap();
            let v0 = self.diagram.vertex_get(v0).unwrap();
            let v0 = cgmath::Point2 {
                x: v0.x(),
                y: v0.y(),
            };

            let v1 = self.diagram.edge_get_vertex0(edge)?.unwrap();
            let v1 = self.diagram.vertex_get(v1).unwrap();
            let v1 = cgmath::Point2 {
                x: v1.x(),
                y: v1.y(),
            };
            Ok(region.0.contains_point_inclusive(&v0)
                && region.0.contains_point_inclusive(&v1)
                && convex_hull::ConvexHull::contains_point_inclusive(&region.1, &v0)
                && convex_hull::ConvexHull::contains_point_inclusive(&region.1, &v1))
        };

        'outer: for region in ignored_regions.iter().enumerate() {
            for edge in edges.into_iter() {
                if !is_inside_region(VD::VoronoiEdgeIndex(edge), region.1)? {
                    //println!("edge: {:?} is not inside region {}, skipping", edge, region.0);
                    continue 'outer;
                }
            }
            //println!("edges were all inside region {}", region.0);
            return Ok(true);
        }
        Ok(false)
    }

    /// move across each edge and sample the lines and arcs
    fn traverse_edges(
        &mut self,
        maxdist: F1,
        ignored_regions: &[(
            linestring::cgmath_2d::Aabb2<F1>,
            linestring::cgmath_2d::LineString2<F1>,
        )],
    ) -> Result<(), CenterlineError> {
        // de-couple self and containers
        let mut lines = self.lines.take().unwrap();
        let mut linestrings = self.line_strings.take().unwrap();

        let mut ignored_edges = self.ignored_edges.take().unwrap();

        #[cfg(feature = "console_debug")]
        let mut edge_lines = fnv::FnvHashMap::<usize, [F1; 4]>::default();

        linestrings.clear();
        lines.clear();

        if !ignored_regions.is_empty() {
            // find the groups of connected edges in this shape
            let mut searched_edges_v = Vec::<yabf::Yabf>::new();
            let mut searched_edges_s = ignored_edges.clone();
            for it in self.diagram.edges().iter().enumerate() {
                // can not use iter().filter() because of the borrow checker
                if searched_edges_s.bit(it.0) {
                    continue;
                }
                let mut edges = yabf::Yabf::with_capacity(self.diagram.edges.len());
                self.recursively_mark_connected_edges(
                    VD::VoronoiEdgeIndex(it.0),
                    &mut edges,
                    true,
                )?;
                searched_edges_s |= &edges;
                searched_edges_v.push(edges);
            }

            for edges in searched_edges_v.iter() {
                if self.edges_are_inside_ignored_region(edges, ignored_regions)? {
                    //println!("edges: are inside ignored region {:?}", edges);
                    ignored_edges |= &edges;
                    continue;
                } else {
                    //println!("edges: are NOT inside ignored regions {:?}", edges);
                }
            }
            // ignored_edges are now filled with the rejected edges
        }

        let mut used_edges = ignored_edges.clone();

        for it in self.diagram.edges().iter().enumerate() {
            // can not use iter().filter() because of the borrow checker
            if used_edges.bit(it.0) {
                continue;
            }
            let edge_id = VD::VoronoiEdgeIndex(it.0);

            self.traverse_edge(
                edge_id,
                false,
                &ignored_edges,
                &mut used_edges,
                &mut lines,
                &mut linestrings,
                maxdist,
            )?;
        }

        // loop over each edge again, make sure they were all used or properly ignored.
        for it in self.diagram.edges().iter().enumerate() {
            // can not use iter().filter() because of the borrow checker
            if used_edges.bit(it.0) {
                continue;
            }
            let edge_id = VD::VoronoiEdgeIndex(it.0);
            #[cfg(feature = "console_debug")]
            println!(
                "Did not use all edges, forcing the use of edge:{}",
                edge_id.0
            );

            self.traverse_edge(
                edge_id,
                true,
                &ignored_edges,
                &mut used_edges,
                &mut lines,
                &mut linestrings,
                maxdist,
            )?;
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
        #[cfg(feature = "console_debug")]
        {
            self.debug_edges = Some(edge_lines);
        }
        Ok(())
    }

    /// Mark an edge and it's twin as used/rejected
    #[inline(always)]
    fn mark_edge_and_twin_as_used(
        &self,
        edge_id: VD::VoronoiEdgeIndex,
        used_edges: &mut yabf::Yabf,
    ) -> Result<(), CenterlineError> {
        used_edges.set_bit(edge_id.0, true);
        #[cfg(feature = "console_debug")]
        print!("marking {}", edge_id.0);
        if let Some(twin) = self.diagram.edge_get(edge_id)?.twin() {
            #[cfg(feature = "console_debug")]
            print!(" & {}", twin.0);
            if used_edges.bit(twin.0) {
                eprintln!(" TWIN was already used!!!!! edge id:{}", twin.0);
            }
            used_edges.set_bit(twin.0, true);
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    /// move across each adjacent edge and sample the lines and arcs
    /// If force_seed_edge is set to false the method tries to
    /// start at edges with only one connection (using seed_edge as a search start point).
    /// Edge loops will not be processed in this mode.
    /// If force_seed_edge is set to true, the seed_edge will be used as a starting point.
    fn traverse_edge(
        &self,
        seed_edge: VD::VoronoiEdgeIndex,
        force_seed_edge: bool,
        ignored_edges: &yabf::Yabf,
        used_edges: &mut yabf::Yabf,
        lines: &mut Vec<Line3<F1>>,
        linestrings: &mut Vec<LineString3<F1>>,
        maxdist: F1,
    ) -> Result<(), CenterlineError> {
        #[cfg(feature = "console_debug")]
        println!();
        #[cfg(feature = "console_debug")]
        println!("->traverse_edge({})", seed_edge.0);
        #[cfg(feature = "console_debug")]
        let mut mockup = Vec::<Vec<VD::VoronoiEdgeIndex>>::default();

        let found_edge = force_seed_edge
            || self
                .diagram
                .edge_rot_next_iterator(Some(seed_edge))
                .filter(|x| !ignored_edges.bit(x.0))
                .take(2) // we do not need more than 2 for the test
                .count()
                == 1;
        if found_edge {
            let mut start_points = VecDeque::<VD::VoronoiEdgeIndex>::default();
            let mut current_edge_set = Vec::<VD::VoronoiEdgeIndex>::new();
            start_points.push_front(seed_edge);
            while !start_points.is_empty() {
                #[cfg(feature = "console_debug")]
                println!();
                let edge = start_points.pop_front().unwrap();

                if ignored_edges.bit(edge.0) {
                    // Should never happen
                    return Err(CenterlineError::InternalError {
                        txt: format!(
                            "should never happen: edge {} already in ignore list.",
                            edge.0
                        ),
                    });
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
                self.mark_edge_and_twin_as_used(edge, used_edges)?;

                let mut next_edge_o = self.diagram.edge_get(edge)?.next();
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
                                self.mark_edge_and_twin_as_used(e, used_edges)?;

                                next_edge_o = self.diagram.edge_get(e)?.next();
                            } else {
                                // terminating the line string, pushing candidates
                                let _ = self.convert_edges_to_lines(
                                    &current_edge_set,
                                    lines,
                                    linestrings,
                                    maxdist,
                                )?;
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
                            let _ = self.convert_edges_to_lines(
                                &current_edge_set,
                                lines,
                                linestrings,
                                maxdist,
                            )?;
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
            /*
            #[cfg(feature = "console_debug")]
            for m in mockup.iter() {
                println!("mockup {:?}", m.iter().map(|x| x.0).collect::<Vec<usize>>());

                let mut i1 = m.iter();
                for e2 in m.iter().skip(1) {
                    let e1 = i1.next().unwrap();
                    assert_eq!(
                        self.diagram.edge_get_vertex1(Some(*e1)),
                        self.diagram.edge_get_vertex0(Some(*e2))
                    );
                }
            }*/
        } else {
            #[cfg(feature = "console_debug")]
            println!(
                "<-traverse_edge({}) ignoring start edge, count={} {:?}",
                seed_edge.0,
                count,
                self.diagram
                    .edge_rot_next_iterator(Some(seed_edge))
                    .filter(|x| !ignored_edges.bit(x.0))
                    .map(|x| x.0)
                    .collect::<Vec<usize>>()
            );
        }
        Ok(())
    }

    fn convert_edges_to_lines(
        &self,
        edges: &[VD::VoronoiEdgeIndex],
        lines: &mut Vec<Line3<F1>>,
        linestrings: &mut Vec<LineString3<F1>>,
        maxdist: F1,
    ) -> Result<(), CenterlineError> {
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
                let edge = self.diagram.edge_get(*edge_id)?;
                match self.convert_edge_to_shape(&edge) {
                    Ok(cgmath_3d::Shape3d::Line(l)) => lines.push(l),
                    Ok(cgmath_3d::Shape3d::ParabolicArc(a)) => {
                        linestrings.push(a.discretise_3d(maxdist));
                    }
                    Ok(cgmath_3d::Shape3d::Linestring(_s)) => {
                        panic!();
                    }
                    Err(_) => {
                        println!("Error :{:?}", edge);
                    }
                }
            }
            _ => {
                let mut ls = LineString3::<F1>::default();
                for edge_id in edges.iter() {
                    let edge = self.diagram.edge_get(*edge_id)?;
                    match self.convert_edge_to_shape(&edge) {
                        Ok(cgmath_3d::Shape3d::Line(l)) => {
                            //println!("->got {:?}", l);
                            ls.push(l.start);
                            ls.push(l.end);
                            //println!("<-got");
                        }
                        Ok(cgmath_3d::Shape3d::ParabolicArc(a)) => {
                            //println!("->got {:?}", a);
                            ls.append(a.discretise_3d(maxdist));
                            //println!("<-got");
                        }
                        // should not happen
                        Ok(cgmath_3d::Shape3d::Linestring(_s)) => panic!(),
                        Err(_) => panic!(),
                    }
                }
                linestrings.push(ls);
            }
        }
        //println!("Converted {:?} to lines", edges);
        Ok(())
    }

    fn convert_edge_to_shape(
        &self,
        edge: &VD::VoronoiEdge<I1, F1>,
    ) -> Result<cgmath_3d::Shape3d<F1>, CenterlineError> {
        let edge_id = edge.get_id();
        let edge_twin_id = self.diagram.edge_get_twin_err(edge_id)?;

        // Edge is finite so we know that vertex0 and vertex1 is_some()
        let vertex0 = self.diagram.vertex_get(edge.vertex0().unwrap())?;

        let vertex1 = self.diagram.edge_get_vertex1(edge_id)?.unwrap();
        let vertex1 = self.diagram.vertex_get(vertex1)?;

        #[cfg(feature = "console_debug")]
        println!(
            "Converting e:{:?} to line v0:{} v1:{}",
            edge.get_id().0,
            vertex0.get_id().0,
            vertex1.get_id().0,
        );

        let start_point = cgmath::Point2 {
            x: vertex0.x(),
            y: vertex0.y(),
        };
        let end_point = cgmath::Point2 {
            x: vertex1.x(),
            y: vertex1.y(),
        };
        let cell_id = self.diagram.edge_get(edge_id)?.cell().unwrap();
        let cell = self.diagram.cell_get(cell_id)?;
        let twin_cell_id = self.diagram.edge_get(edge_twin_id)?.cell().unwrap();

        let cell_point = if cell.contains_point() {
            #[cfg(feature = "console_debug")]
            println!("cell c:{}", cell_id.0);
            self.retrieve_point(cell_id)?
        } else {
            #[cfg(feature = "console_debug")]
            println!("twin cell c:{}", twin_cell_id.0);
            self.retrieve_point(twin_cell_id)?
        };
        let segment = if cell.contains_point() {
            #[cfg(feature = "console_debug")]
            println!("twin segment c:{}", twin_cell_id.0);
            self.retrieve_segment(twin_cell_id)?
        } else {
            #[cfg(feature = "console_debug")]
            println!("segment c:{}", cell_id.0);
            self.retrieve_segment(cell_id)?
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
        #[cfg(feature = "console_debug")]
        {
            println!("sp:[{},{}]", start_point.x, start_point.y);
            println!("ep:[{},{}]", end_point.x, end_point.y);
            println!(
                "cp:[{},{}] sg:[{},{},{},{}]",
                cell_point.x,
                cell_point.y,
                segment_start_point.x,
                segment_start_point.y,
                segment_end_point.x,
                segment_end_point.y
            );
        }

        if edge.is_curved() {
            let arc = cgmath_2d::VoronoiParabolicArc::new(
                cgmath_2d::Line2 {
                    start: segment_start_point,
                    end: segment_end_point,
                },
                cell_point,
                start_point,
                end_point,
            );
            #[cfg(feature = "console_debug")]
            println!("Converted {:?} to {:?}", edge.get_id().0, arc);
            Ok(cgmath_3d::Shape3d::ParabolicArc(arc))
        } else {
            let distance_to_start = {
                if vertex0.is_site_point() {
                    F1::zero()
                } else if cell.contains_point() {
                    let cell_point = self.retrieve_point(cell_id)?;
                    let cell_point = cgmath::Point2 {
                        x: Self::i2f(cell_point.x),
                        y: Self::i2f(cell_point.y),
                    };
                    linestring::cgmath_2d::distance_to_point_squared(&cell_point, &start_point)
                        .sqrt()
                } else {
                    let segment = self.retrieve_segment(cell_id)?;
                    let segment_start_point = cgmath::Point2 {
                        x: Self::i2f(segment.start.x),
                        y: Self::i2f(segment.start.y),
                    };
                    let segment_end_point = cgmath::Point2 {
                        x: Self::i2f(segment.end.x),
                        y: Self::i2f(segment.end.y),
                    };
                    linestring::cgmath_2d::distance_to_line_squared_safe(
                        &segment_start_point,
                        &segment_end_point,
                        &start_point,
                    )
                    .sqrt()
                }
            };
            let distance_to_end = {
                if vertex1.is_site_point() {
                    F1::zero()
                } else {
                    let cell_id = self
                        .diagram
                        .edge_get(vertex1.get_incident_edge().unwrap())?
                        .cell()
                        .unwrap();
                    let cell = self.diagram.cell_get(cell_id)?;
                    if cell.contains_point() {
                        let cell_point = self.retrieve_point(cell_id)?;
                        let cell_point = cgmath::Point2 {
                            x: Self::i2f(cell_point.x),
                            y: Self::i2f(cell_point.y),
                        };
                        linestring::cgmath_2d::distance_to_point_squared(&cell_point, &end_point)
                            .sqrt()
                    } else {
                        let segment = self.retrieve_segment(cell_id)?;
                        let segment_start_point = cgmath::Point2 {
                            x: Self::i2f(segment.start.x),
                            y: Self::i2f(segment.start.y),
                        };
                        let segment_end_point = cgmath::Point2 {
                            x: Self::i2f(segment.end.x),
                            y: Self::i2f(segment.end.y),
                        };
                        linestring::cgmath_2d::distance_to_line_squared_safe(
                            &segment_start_point,
                            &segment_end_point,
                            &end_point,
                        )
                        .sqrt()
                    }
                }
            };
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
            Ok(cgmath_3d::Shape3d::Line(line))
        }
    }

    #[inline(always)]
    pub fn i2f(input: I1) -> F1 {
        num::cast::<I1, F1>(input).unwrap()
    }
}

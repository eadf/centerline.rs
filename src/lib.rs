#![deny(
    rust_2018_compatibility,
    rust_2018_idioms,
    nonstandard_style,
    unused,
    future_incompatible,
    non_camel_case_types,
    unused_parens,
    non_upper_case_globals,
    unused_qualifications,
    unused_results,
    unused_imports,
    unused_variables
)]
#![cfg_attr(feature = "hash_drain_filter", feature(hash_drain_filter))]
#![cfg_attr(feature = "map_first_last", feature(map_first_last))]

use boostvoronoi::builder as VB;
use boostvoronoi::diagram as VD;
use boostvoronoi::sync_diagram as VSD;
use boostvoronoi::{InputType, OutputType};
use cgmath::InnerSpace;
use cgmath::SquareMatrix;
use cgmath::Transform;
use linestring::linestring_2d;
use linestring::linestring_2d::convex_hull;
use linestring::linestring_3d;
use linestring::linestring_3d::{Line3, LineString3, LineStringSet3};
use ordered_float::OrderedFloat;
use rayon::iter::IntoParallelIterator;
use rayon::iter::ParallelIterator;
use std::collections::VecDeque;
use std::line;
use std::ops::Neg;
use thiserror::Error;

#[macro_use]
extern crate bitflags;

#[derive(Error, Debug)]
pub enum CenterlineError {
    #[error("Something is wrong with the internal logic")]
    InternalError(String),

    #[error("Something is wrong with the input data")]
    CouldNotCalculateInverseMatrix,

    #[error("Your line-strings are self-intersecting.")]
    SelfIntersectingData,

    #[error("The input data is not 2D")]
    InputNotPLane,

    #[error("Invalid data")]
    InvalidData(String),

    #[error(transparent)]
    BvError(#[from] boostvoronoi::BvError),

    #[error("Error from .obj file handling")]
    ObjError(String),

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
    id: usize,                      // index into the point3 list
    connected_vertices: Vec<usize>, // list of other vertices this vertex is connected to
    shape: Option<usize>,           // shape id
}

/// paints every connected vertex with the
fn paint_every_connected_vertex(
    vertices: &mut ahash::AHashMap<usize, Vertices>,
    already_painted: &mut yabf::Yabf,
    vertex_id: usize,
    color: usize,
) -> Result<(), CenterlineError> {
    let mut queue = VecDeque::<usize>::new();
    queue.push_back(vertex_id);

    while !queue.is_empty() {
        // unwrap is safe here, we just checked that there are item in the queue
        let current_vertex = queue.pop_front().unwrap();
        if already_painted.bit(current_vertex) {
            continue;
        }

        if let Some(vertex_obj) = vertices.get_mut(&current_vertex) {
            if vertex_obj.shape.is_none() {
                vertex_obj.shape = Some(color);
                already_painted.set_bit(current_vertex, true);
            } else {
                // already painted for some reason
                continue;
            }
            for v in vertex_obj.connected_vertices.iter() {
                if !already_painted.bit(*v) {
                    queue.push_back(*v);
                }
            }
        } else {
            return Err(CenterlineError::InternalError(format!(
                "Vertex with id:{} dissapeared. {}:{}",
                current_vertex,
                file!(),
                line!()
            )));
        };
    }
    Ok(())
}

#[cfg(feature = "impl-wavefront")]
#[allow(clippy::type_complexity)]
/// Remove internal edges from a wavefront-obj object
/// This requires the feature "impl-wavefront" to be active.
pub fn remove_internal_edges(
    obj: obj::raw::RawObj,
) -> Result<(ahash::AHashSet<(usize, usize)>, Vec<cgmath::Point3<f32>>), CenterlineError> {
    for p in obj.points.iter() {
        // Ignore all points
        println!("Ignored point:{:?}", p);
    }
    let mut all_edges = ahash::AHashSet::<(usize, usize)>::default();
    let mut internal_edges = ahash::AHashSet::<(usize, usize)>::default();

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
    #[cfg(feature = "hash_drain_filter")]
    {
        let _ = all_edges.drain_filter(|x| internal_edges.contains(x));
    }

    #[cfg(not(feature = "hash_drain_filter"))]
    {
        // inefficient version of drain_filter for +stable
        let kept_edges = all_edges
            .into_iter()
            .filter(|x| !internal_edges.contains(&x))
            .collect();
        all_edges = kept_edges;
    }

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
    edge_set: ahash::AHashSet<(usize, usize)>,
    points: Vec<cgmath::Point3<T>>,
) -> Result<Vec<LineStringSet3<T>>, CenterlineError>
where
    T: cgmath::BaseFloat + Sync + Send,
{
    //println!("All edges: {:?}", all_edges);
    // put all edges into a hashmap of Vertices, this will make it possible to
    // arrange them in the order they are connected
    let mut vertices = ahash::AHashMap::<usize, Vertices>::default();
    for e in edge_set.iter() {
        let id = e.0;
        let other = e.1;
        vertices
            .entry(id)
            .or_insert_with_key(|key| Vertices {
                id: *key,
                connected_vertices: Vec::<usize>::new(),
                shape: None,
            })
            .connected_vertices
            .push(other);

        let id = e.1;
        let other = e.0;
        vertices
            .entry(id)
            .or_insert_with_key(|key| Vertices {
                id: *key,
                connected_vertices: Vec::<usize>::new(),
                shape: None,
            })
            .connected_vertices
            .push(other);
    }
    //println!("Vertices: {:?}", vertices.iter().map(|x|x.1.id).collect::<Vec<usize>>());
    // Do a search on one vertex, paint all connected vertices with the same number.
    let mut unique_shape_id_generator = 0..usize::MAX;

    let mut already_painted = yabf::Yabf::with_capacity(vertices.len());
    for vertex_id in 0..vertices.len() {
        if already_painted.bit(vertex_id) {
            continue;
        }

        // found an un-painted vertex
        paint_every_connected_vertex(
            &mut vertices,
            &mut already_painted,
            vertex_id,
            unique_shape_id_generator.next().unwrap(),
        )?;
    }
    let highest_shape_id_plus_one = unique_shape_id_generator.next().unwrap();
    if highest_shape_id_plus_one == 0 {
        return Err(CenterlineError::InternalError(format!(
            "Could not find any shapes to separate. {}:{}",
            file!(),
            line!()
        )));
    }

    // Spit all detected connected vertices into separate sets.
    // i.e. every vertex with the same color goes into the same set.
    let mut shape_separation = Vec::<ahash::AHashMap<usize, Vertices>>::new();
    for current_shape in 0..highest_shape_id_plus_one {
        if vertices.is_empty() {
            println!("vertices:{:?}", vertices);
            println!("current_shape:{}", current_shape);
            println!("shape_separation:{:?}", shape_separation);

            return Err(CenterlineError::InternalError(format!(
                "Could not separate all shapes, ran out of vertices. {}:{}",
                file!(),
                line!()
            )));
        }
        #[cfg(feature = "hash_drain_filter")]
        {
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
        #[cfg(not(feature = "hash_drain_filter"))]
        {
            // inefficient version of drain_filter for +stable
            let mut drained = ahash::AHashMap::<usize, Vertices>::default();
            let mut new_vertices = ahash::AHashMap::<usize, Vertices>::default();
            for (x0, x1) in vertices.into_iter() {
                if x1.shape.map_or(false, |shape| shape == current_shape) {
                    let _ = drained.insert(x0, x1);
                } else {
                    let _ = new_vertices.insert(x0, x1);
                };
            }
            vertices = new_vertices;
            shape_separation.push(drained);
        }
    }
    drop(vertices);
    // now we have a list of groups of vertices, each group are connected by edges.

    let shape_separation = shape_separation;

    // Create lists of linestrings3 by walking the edges of each vertex set.
    shape_separation
        .into_par_iter()
        .map(|rvi| -> Result<LineStringSet3<T>, CenterlineError> {
            if rvi.is_empty() {
                return Err(CenterlineError::InternalError(
                    format!("rvi.is_empty() Seems like the shape separation failed. {}:{}", file!(),line!()),
                ));
            }
            let mut loops = 0;

            let mut rvs = linestring_3d::LineStringSet3::<T>::with_capacity(rvi.len());
            let mut als = linestring_3d::LineString3::<T>::with_capacity(rvi.len());

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
                    next = *current_vertex.connected_vertices.iter().find(|x| **x != prev).ok_or_else(||
                        CenterlineError::InvalidData(
                            "Could not find next vertex. All lines must form unconnected loops".to_string(),
                        ),
                    )?;
                } else {
                    break;
                }
                // allow the start point to be added twice (in case of a loop)
                if !first_loop && current == started_with {
                    break;
                }
                first_loop = false;
                loops += 1;
                if loops > rvi.len() + 1 {
                    return Err(CenterlineError::InvalidData(
                        "It seems like one (or more) of the line strings does not form an unconnected loop."
                            .to_string(),
                    ));
                }
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
            Ok(rvs)
        })
        .collect()
}

#[inline(always)]
/// Calculate an affine transform that will center, flip plane to XY, and scale the arbitrary shape
/// so that it will fill the screen. For good measure the scale is then multiplied by 256 so the
/// points makes half decent input data to boost voronoi (integer input only)
/// 'desired_voronoi_dimension' is the maximum length of the voronoi input data aabb
/// boost_voronoi uses integers as input so float vertices have to be scaled up substantially to
/// maintain numerical precision
pub fn get_transform<F>(
    total_aabb: &linestring_3d::Aabb3<F>,
    desired_voronoi_dimension: F,
) -> Result<
    (
        linestring_3d::Plane,
        cgmath::Matrix4<F>,
        linestring::linestring_2d::Aabb2<F>,
    ),
    CenterlineError,
>
where
    F: cgmath::BaseFloat + Sync,
{
    get_transform_relaxed(
        total_aabb,
        desired_voronoi_dimension,
        F::default_epsilon(),
        F::default_max_ulps(),
    )
}

/// Calculate an affine transform that will center, flip plane to XY, and scale the arbitrary shape
/// so that it will fill the screen. For good measure the scale is then multiplied by 256 so the
/// points makes half decent input data to boost voronoi (integer input only)
/// 'desired_voronoi_dimension' is the maximum length of the voronoi input data aabb
/// boost_voronoi uses integers as input so float vertices have to be scaled up substantially to
/// maintain numerical precision
pub fn get_transform_relaxed<F>(
    total_aabb: &linestring_3d::Aabb3<F>,
    desired_voronoi_dimension: F,
    epsilon: F,
    max_ulps: u32,
) -> Result<
    (
        linestring_3d::Plane,
        cgmath::Matrix4<F>,
        linestring::linestring_2d::Aabb2<F>,
    ),
    CenterlineError,
>
where
    F: cgmath::BaseFloat + Sync,
{
    let plane = if let Some(plane) =
        linestring_3d::Plane::get_plane_relaxed(total_aabb, epsilon, max_ulps)
    {
        plane
    } else {
        return Err(CenterlineError::InputNotPLane);
    };

    println!(
        "get_transform_relaxed desired_voronoi_dimension:{:?}",
        desired_voronoi_dimension
    );

    let low = total_aabb.get_low().unwrap();
    let high = total_aabb.get_high().unwrap();
    let delta = high - low;
    let center = cgmath::point3(
        (high.x + low.x) / F::from(2.0).unwrap(),
        (high.y + low.y) / F::from(2.0).unwrap(),
        (high.z + low.z) / F::from(2.0).unwrap(),
    );
    println!(
        "Input data AABB: Center:({:?}, {:?}, {:?})",
        center.x, center.y, center.z,
    );
    println!(
        "                   high:({:?}, {:?}, {:?})",
        high.x, high.y, high.z,
    );
    println!(
        "                    low:({:?}, {:?}, {:?})",
        low.x, low.y, low.z,
    );
    println!(
        "                  delta:({:?}, {:?}, {:?})",
        delta.x, delta.y, delta.z,
    );

    let scale_transform = {
        let scale = desired_voronoi_dimension
            / std::cmp::max(
                std::cmp::max(OrderedFloat(delta.x), OrderedFloat(delta.y)),
                OrderedFloat(delta.z),
            )
            .into_inner();

        cgmath::Matrix4::from_scale(scale)
    };

    let center = scale_transform.transform_point(center);
    let center_transform: cgmath::Matrix4<F> =
        cgmath::Matrix4::from_translation(cgmath::Vector3::new(-center.x, -center.y, -center.z));

    let plane_transform: cgmath::Matrix4<F> = {
        let x = cgmath::Vector4::<F>::new(F::one(), F::zero(), F::zero(), F::zero());
        let y = cgmath::Vector4::<F>::new(F::zero(), F::one(), F::zero(), F::zero());
        let z = cgmath::Vector4::<F>::new(F::zero(), F::zero(), F::one(), F::zero());
        let w = cgmath::Vector4::<F>::new(F::zero(), F::zero(), F::zero(), F::one());

        match plane {
            linestring_3d::Plane::XY => cgmath::Matrix4::from_cols(x, y, z, w),
            linestring_3d::Plane::XZ => cgmath::Matrix4::from_cols(x, z, y, w),
            linestring_3d::Plane::ZY => cgmath::Matrix4::from_cols(z, y, x, w),
        }
    };

    let total_transform = plane_transform * center_transform * scale_transform;

    let high0 = total_aabb.get_high().unwrap();
    let low0 = total_aabb.get_low().unwrap();

    let low0 = total_transform.transform_point(low0);
    let high0 = total_transform.transform_point(high0);
    let delta0 = high0 - low0;
    let center0 = cgmath::point3(
        (high0.x + low0.x) / F::from(2.0).unwrap(),
        (high0.y + low0.y) / F::from(2.0).unwrap(),
        (high0.z + low0.z) / F::from(2.0).unwrap(),
    );
    #[cfg(feature = "console_debug")]
    let center0 = total_transform.transform_point(center0);

    #[cfg(feature = "console_debug")]
    println!(
        "Voronoi input AABB: Center {:?} low:{:?}, high:{:?}",
        center0, low0, high0
    );
    let mut voronoi_input_aabb =
        linestring::linestring_2d::Aabb2::new(&cgmath::Point2::new(low0.x, low0.y));
    voronoi_input_aabb.update_point(&cgmath::Point2::new(high0.x, high0.y));

    println!(
        "Voronoi input AABB: Center:({:?}, {:?}, {:?})",
        center0.x, center0.y, center0.z,
    );
    println!(
        "                   high:({:?}, {:?}, {:?})",
        high0.x, high0.y, high0.z,
    );
    println!(
        "                    low:({:?}, {:?}, {:?})",
        low0.x, low0.y, low0.z,
    );
    println!(
        "                  delta:({:?}, {:?}, {:?})",
        delta0.x, delta0.y, delta0.z,
    );

    let inverse_total = total_transform.invert();
    if inverse_total.is_none() {
        return Err(CenterlineError::CouldNotCalculateInverseMatrix);
    }
    //let inverse_total = inverse_total.unwrap();

    //let low0 = inverse_total.transform_point(low0);
    //let high0 = inverse_total.transform_point(high0);
    //let center0 = inverse_total.transform_point(center0);
    //println!("I Center {:?} low:{:?}, high:{:?}", center0, low0, high0);

    Ok((plane, total_transform, voronoi_input_aabb))
}

/// try to consolidate shapes. If one AABB and convex hull (a) totally engulfs another shape (b)
/// we put shape (b) inside (a)
pub fn consolidate_shapes<F>(
    mut raw_data: Vec<linestring::linestring_2d::LineStringSet2<F>>,
) -> Result<Vec<linestring::linestring_2d::LineStringSet2<F>>, CenterlineError>
where
    F: cgmath::BaseFloat + Sync,
{
    //for shape in raw_data.iter().enumerate() {
    //    println!("Shape #{} aabb:{:?}", shape.0, shape.1.get_aabb());
    //}
    'outer_loop: loop {
        // redo *every* test until nothing else can be done
        for i in 0..raw_data.len() {
            for j in i + 1..raw_data.len() {
                //println!("testing #{} vs #{}", i, j);
                if raw_data[i].get_aabb().contains_aabb(raw_data[j].get_aabb())
                    && linestring::linestring_2d::convex_hull::ConvexHull::contains(
                        raw_data[i].get_convex_hull().as_ref().unwrap(),
                        raw_data[j].get_convex_hull().as_ref().unwrap(),
                        F::default_epsilon() * F::from(2.0).unwrap(),
                        F::default_max_ulps() * 2,
                    )
                {
                    //println!("#{} contains #{}", i, j);
                    // move stuff from j to i via a temp because of borrow checker
                    let mut stolen_line_j = linestring::linestring_2d::LineStringSet2::steal_from(
                        raw_data.get_mut(j).unwrap(),
                    );
                    let line_i = raw_data.get_mut(i).unwrap();
                    line_i.take_from_internal(&mut stolen_line_j)?;
                    let _ = raw_data.remove(j);
                    continue 'outer_loop;
                } else if raw_data[j].get_aabb().contains_aabb(raw_data[i].get_aabb())
                    && linestring::linestring_2d::convex_hull::ConvexHull::contains(
                        raw_data[j].get_convex_hull().as_ref().unwrap(),
                        raw_data[i].get_convex_hull().as_ref().unwrap(),
                        F::default_epsilon() * F::from(2.0).unwrap(),
                        F::default_max_ulps() * 2,
                    )
                {
                    //println!("#{} contains #{}", j, i);
                    // move stuff from i to j via a temp because of borrow checker
                    let mut stolen_line_i = linestring::linestring_2d::LineStringSet2::steal_from(
                        raw_data.get_mut(i).unwrap(),
                    );
                    let line_j = raw_data.get_mut(j).unwrap();
                    line_j.take_from_internal(&mut stolen_line_i)?;
                    let _ = raw_data.remove(i);
                    continue 'outer_loop;
                }
            }
        }
        break 'outer_loop;
    }
    Ok(raw_data)
}

/// Center line calculation object.
/// It: * calculates the segmented voronoi diagram.
///     * Filter out voronoi edges based on the angle to input geometry.
///     * Collects connected edges into line strings and line segments.
///     * Performs line simplification on those line strings.
pub struct Centerline<I, F>
where
    I: InputType + Neg<Output = I>,
    F: cgmath::BaseFloat + Sync + OutputType + Neg<Output = F>,
{
    /// the input data to the voronoi diagram
    pub segments: Vec<boostvoronoi::geometry::Line<I>>,
    /// the voronoi diagram itself
    pub diagram: VSD::SyncDiagram<I, F>,
    /// the individual two-point edges
    pub lines: Option<Vec<Line3<F>>>,
    /// concatenated connected edges
    pub line_strings: Option<Vec<LineString3<F>>>,

    /// bit field defining edges rejected by EXTERNAL or INFINITE
    rejected_edges: Option<yabf::Yabf>,
    /// bit field defining edges rejected by 'rejected_edges' + dot test
    ignored_edges: Option<yabf::Yabf>,

    #[cfg(feature = "console_debug")]
    pub debug_edges: Option<ahash::AHashMap<usize, [F; 4]>>,
}

impl<I, F> Centerline<I, F>
where
    I: InputType + Neg<Output = I>,
    F: cgmath::BaseFloat + Sync + OutputType + Neg<Output = F>,
{
    /// Creates a Centerline container with a set of segments
    pub fn default() -> Self {
        Self {
            diagram: VSD::SyncDiagram::default(),
            segments: Vec::<boostvoronoi::geometry::Line<I>>::default(),
            lines: Some(Vec::<Line3<F>>::new()),
            line_strings: Some(Vec::<LineString3<F>>::new()),
            rejected_edges: None,
            ignored_edges: None,
            #[cfg(feature = "console_debug")]
            debug_edges: None,
        }
    }

    /// Creates a Centerline container with a set of segments
    pub fn with_segments(segments: Vec<boostvoronoi::geometry::Line<I>>) -> Self {
        Self {
            diagram: VSD::SyncDiagram::default(),
            segments,
            lines: Some(Vec::<Line3<F>>::new()),
            line_strings: Some(Vec::<LineString3<F>>::new()),
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
        self.diagram = vb.build()?.into();
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
        cos_angle: F,
        discrete_limit: F,
        ignored_regions: Option<
            &Vec<(
                linestring::linestring_2d::Aabb2<F>,
                linestring::linestring_2d::LineString2<F>,
            )>,
        >,
    ) -> Result<(), CenterlineError> {
        self.angle_test(cos_angle)?;
        if let Some(ignored_regions) = ignored_regions {
            self.traverse_edges(discrete_limit, ignored_regions)?;
        } else {
            let ignored_regions = Vec::<(
                linestring::linestring_2d::Aabb2<F>,
                linestring::linestring_2d::LineString2<F>,
            )>::with_capacity(0);
            self.traverse_edges(discrete_limit, &ignored_regions)?;
        }
        Ok(())
    }

    /// Collects lines and linestrings from the centerline.
    /// This version of calculate_centerline() tries to keep as many edges as possible.
    /// The intention is to use the data for mesh generation.
    /// TODO: make this return a true mesh
    pub fn calculate_centerline_mesh(
        &mut self,
        discrete_limit: F,
        ignored_regions: Option<
            &Vec<(
                linestring::linestring_2d::Aabb2<F>,
                linestring::linestring_2d::LineString2<F>,
            )>,
        >,
    ) -> Result<(), CenterlineError> {
        self.ignored_edges = self.rejected_edges.clone();

        if let Some(ignored_regions) = ignored_regions {
            self.traverse_cells(discrete_limit, ignored_regions)?;
        } else {
            let ignored_regions = Vec::<(
                linestring::linestring_2d::Aabb2<F>,
                linestring::linestring_2d::LineString2<F>,
            )>::with_capacity(0);
            self.traverse_cells(discrete_limit, &ignored_regions)?;
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
        cell_id: VD::CellIndex,
    ) -> Result<boostvoronoi::geometry::Point<I>, CenterlineError> {
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
        cell_id: VD::CellIndex,
    ) -> Result<boostvoronoi::geometry::Line<I>, CenterlineError> {
        let cell = self.diagram.cell_get(cell_id)?;
        Ok(self.segments[cell.source_index()])
    }

    /// returns a reference to the internal voronoi diagram
    pub fn diagram(&self) -> &VSD::SyncDiagram<I, F> {
        &self.diagram
    }

    /// Mark infinite edges and their adjacent edges as EXTERNAL.
    fn reject_external_edges(&mut self) -> Result<(), CenterlineError> {
        let mut rejected_edges = yabf::Yabf::default();
        // ensure capacity of bit field by setting last bit +1 to true
        rejected_edges.set_bit(self.diagram().edges().len(), true);

        for edge in self.diagram.edges().iter() {
            let edge_id = edge.id();
            if edge.is_secondary() {
                rejected_edges.set_bit(edge_id.0, true);
                //self.diagram
                //    .edge_or_color(edge_id, ColorFlag::SECONDARY.bits)?;
                let twin_id = self.diagram.edge_get_twin(edge_id)?;
                //self.diagram
                //    .edge_or_color(twin_id, ColorFlag::SECONDARY.bits);
                rejected_edges.set_bit(twin_id.0, true);
            }
            if !self.diagram.edge_is_finite(edge_id)? {
                self.mark_connected_edges(edge_id, &mut rejected_edges, true)?;
                rejected_edges.set_bit(edge_id.0, true);
            }
        }

        self.rejected_edges = Some(rejected_edges);
        Ok(())
    }

    /// Reject edges that does not pass the angle test.
    /// It iterates over all cells, looking for vertices that are identical to the
    /// input segment endpoints.
    /// It then look at edges connected to that vertex and test if the dot product
    /// between the normalized segment vector and normalized edge vector exceeds
    /// a predefined value.
    /// TODO: there must be a quicker way to get this information from the voronoi diagram
    /// maybe mark each vertex identical to input points..
    fn angle_test(&mut self, cos_angle: F) -> Result<(), CenterlineError> {
        let mut ignored_edges = self.rejected_edges.clone().take().unwrap();

        for cell in self.diagram.cells().iter() {
            let cell_id = cell.id();

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
                let mut e = incident_e;
                loop {
                    e = self.diagram.edge_get_next(e)?;

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
                                let _ = self.angle_test_6(
                                    cos_angle,
                                    &mut ignored_edges,
                                    e,
                                    &vertex0,
                                    &vertex1,
                                    &point0,
                                    &point1,
                                )? || self.angle_test_6(
                                    cos_angle,
                                    &mut ignored_edges,
                                    e,
                                    &vertex0,
                                    &vertex1,
                                    &point1,
                                    &point0,
                                )? || self.angle_test_6(
                                    cos_angle,
                                    &mut ignored_edges,
                                    e,
                                    &vertex1,
                                    &vertex0,
                                    &point0,
                                    &point1,
                                )? || self.angle_test_6(
                                    cos_angle,
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

                    if e == incident_e {
                        break;
                    }
                }
            }
        }
        self.ignored_edges = Some(ignored_edges);
        Ok(())
    }

    /// set the edge as rejected if it fails the dot product test
    #[allow(clippy::too_many_arguments)]
    fn angle_test_6(
        &self,
        cos_angle: F,
        ignored_edges: &mut yabf::Yabf,
        edge_id: VD::EdgeIndex,
        vertex0: &cgmath::Point2<F>,
        vertex1: &cgmath::Point2<F>,
        s_point_0: &cgmath::Point2<F>,
        s_point_1: &cgmath::Point2<F>,
    ) -> Result<bool, CenterlineError> {
        if cgmath::ulps_eq!(vertex0.x, s_point_0.x) && cgmath::ulps_eq!(vertex0.y, s_point_0.y) {
            // todo better to compare to the square of the dot product, fewer operations.
            let segment_v = (s_point_1 - s_point_0).normalize();
            let vertex_v = (vertex1 - vertex0).normalize();
            if segment_v.dot(vertex_v).abs() < cos_angle {
                let twin = self.diagram.edge_get_twin(edge_id)?;
                ignored_edges.set_bit(twin.0, true);
                ignored_edges.set_bit(edge_id.0, true);
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Marks this edge and all other edges connecting to it via vertex1.
    /// Line iteration stops when connecting to input geometry.
    /// if 'initial' is set to true it will search both ways, edge and the twin edge, but only
    /// for the first edge.
    fn mark_connected_edges(
        &self,
        edge_id: VD::EdgeIndex,
        marked_edges: &mut yabf::Yabf,
        initial: bool,
    ) -> Result<(), CenterlineError> {
        if marked_edges.bit(edge_id.0) {
            return Ok(());
        }

        let mut initial = initial;
        let mut queue = VecDeque::<VD::EdgeIndex>::new();
        queue.push_back(edge_id);

        'outer: while !queue.is_empty() {
            // unwrap is safe since we just checked !queue.is_empty()
            let edge_id = queue.pop_front().unwrap();
            if marked_edges.bit(edge_id.0) {
                initial = false;
                continue 'outer;
            }

            let v1 = self.diagram.edge_get_vertex1(edge_id)?;
            if self.diagram.edge_get_vertex0(edge_id)?.is_some() && v1.is_none() {
                // this edge leads to nowhere, stop following line
                marked_edges.set_bit(edge_id.0, true);
                initial = false;
                continue 'outer;
            }
            marked_edges.set_bit(edge_id.0, true);

            #[allow(unused_assignments)]
            if initial {
                initial = false;
                queue.push_back(self.diagram.edge_get_twin(edge_id)?);
            } else {
                marked_edges.set_bit(self.diagram.edge_get_twin(edge_id)?.0, true);
            }

            if v1.is_none() || !self.diagram.edge_get(edge_id)?.is_primary() {
                // stop traversing this line if vertex1 is not found or if the edge is not primary
                initial = false;
                continue 'outer;
            }
            // v1 is always Some from this point on
            if let Some(v1) = v1 {
                let v1 = self.diagram.vertex_get(v1)?;
                if v1.is_site_point() {
                    // break line iteration on site points
                    initial = false;
                    continue 'outer;
                }
                //self.reject_vertex(v1, color);
                let mut this_edge = v1.get_incident_edge()?;
                let v_incident_edge = this_edge;
                loop {
                    if !marked_edges.bit(this_edge.0) {
                        queue.push_back(this_edge);
                    }
                    this_edge = self.diagram.edge_rot_next(this_edge)?;
                    if this_edge == v_incident_edge {
                        break;
                    }
                }
            }
            initial = false;
        }
        Ok(())
    }

    /// returns true if *all* of the 'edges' are contained inside one of the 'ignored_regions'
    fn edges_are_inside_ignored_region(
        &self,
        edges: &yabf::Yabf,
        ignored_regions: &[(
            linestring::linestring_2d::Aabb2<F>,
            linestring::linestring_2d::LineString2<F>,
        )],
    ) -> Result<bool, CenterlineError> {
        let is_inside_region = |edge: VD::EdgeIndex,
                                region: &(
            linestring::linestring_2d::Aabb2<F>,
            linestring::linestring_2d::LineString2<F>,
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
                if !is_inside_region(VD::EdgeIndex(edge), region.1)? {
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
        maxdist: F,
        ignored_regions: &[(
            linestring::linestring_2d::Aabb2<F>,
            linestring::linestring_2d::LineString2<F>,
        )],
    ) -> Result<(), CenterlineError> {
        // de-couple self and containers
        let mut lines = self.lines.take().ok_or_else(|| {
            CenterlineError::InternalError(format!(
                "traverse_edges(): could not take lines. {}:{}",
                file!(),
                line!()
            ))
        })?;
        let mut linestrings = self.line_strings.take().ok_or_else(|| {
            CenterlineError::InternalError(format!(
                "traverse_edges(): could not take linestrings. {}:{}",
                file!(),
                line!()
            ))
        })?;

        let mut ignored_edges = self
            .ignored_edges
            .take()
            .unwrap_or_else(|| yabf::Yabf::with_capacity(0));

        #[cfg(feature = "console_debug")]
        let mut edge_lines = ahash::AHashMap::<usize, [F; 4]>::default();

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
                let mut edges = yabf::Yabf::with_capacity(self.diagram.edges().len());
                self.mark_connected_edges(VD::EdgeIndex(it.0), &mut edges, true)?;
                searched_edges_s |= &edges;
                searched_edges_v.push(edges);
            }

            for edges in searched_edges_v.iter() {
                if self.edges_are_inside_ignored_region(edges, ignored_regions)? {
                    //println!("edges: are inside ignored region {:?}", edges);
                    ignored_edges |= edges;
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
            let edge_id = VD::EdgeIndex(it.0);

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
            let edge_id = VD::EdgeIndex(it.0);
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

    /// move across each cell and sample the lines and arcs
    fn traverse_cells(
        &mut self,
        maxdist: F,
        ignored_regions: &[(
            linestring::linestring_2d::Aabb2<F>,
            linestring::linestring_2d::LineString2<F>,
        )],
    ) -> Result<(), CenterlineError> {
        // de-couple self and containers
        let mut lines = self.lines.take().ok_or_else(|| {
            CenterlineError::InternalError(format!(
                "traverse_edges(): could not take lines. {}:{}",
                file!(),
                line!()
            ))
        })?;
        let mut linestrings = self.line_strings.take().ok_or_else(|| {
            CenterlineError::InternalError(format!(
                "traverse_edges(): could not take linestrings. {}:{}",
                file!(),
                line!()
            ))
        })?;

        let mut ignored_edges = self
            .ignored_edges
            .take()
            .unwrap_or_else(|| yabf::Yabf::with_capacity(0));

        #[cfg(feature = "console_debug")]
        let mut edge_lines = ahash::AHashMap::<usize, [F; 4]>::default();

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
                let mut edges = yabf::Yabf::with_capacity(self.diagram.edges().len());
                self.mark_connected_edges(VD::EdgeIndex(it.0), &mut edges, true)?;
                searched_edges_s |= &edges;
                searched_edges_v.push(edges);
            }

            for edges in searched_edges_v.iter() {
                if self.edges_are_inside_ignored_region(edges, ignored_regions)? {
                    //println!("edges: are inside ignored region {:?}", edges);
                    ignored_edges |= edges;
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
            let edge_id = VD::EdgeIndex(it.0);

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
            let edge_id = VD::EdgeIndex(it.0);
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
        edge_id: VD::EdgeIndex,
        used_edges: &mut yabf::Yabf,
    ) -> Result<(), CenterlineError> {
        used_edges.set_bit(edge_id.0, true);
        #[cfg(feature = "console_debug")]
        print!("marking {}", edge_id.0);
        {
            let twin = self.diagram.edge_get_twin(edge_id)?;
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
        seed_edge: VD::EdgeIndex,
        force_seed_edge: bool,
        ignored_edges: &yabf::Yabf,
        used_edges: &mut yabf::Yabf,
        lines: &mut Vec<Line3<F>>,
        linestrings: &mut Vec<LineString3<F>>,
        maxdist: F,
    ) -> Result<(), CenterlineError> {
        #[cfg(feature = "console_debug")]
        println!();
        #[cfg(feature = "console_debug")]
        println!("->traverse_edge({})", seed_edge.0);

        #[cfg(feature = "console_debug")]
        let mut mockup = Vec::<Vec<VD::EdgeIndex>>::default();

        let found_edge = force_seed_edge
            || self
                .diagram
                .edge_rot_next_iterator(seed_edge)
                .filter(|x| !ignored_edges.bit(x.0))
                .take(2) // we do not need more than 2 for the test
                .count()
                == 1;
        if found_edge {
            let mut start_points = VecDeque::<VD::EdgeIndex>::default();
            let mut current_edge_set = Vec::<VD::EdgeIndex>::new();
            start_points.push_front(seed_edge);
            while !start_points.is_empty() {
                #[cfg(feature = "console_debug")]
                println!();
                let edge = start_points.pop_front().unwrap();

                if ignored_edges.bit(edge.0) {
                    // Should never happen
                    return Err(CenterlineError::InternalError(format!(
                        "should never happen: edge {} already in ignore list. {}:{}",
                        edge.0,
                        file!(),
                        line!()
                    )));
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

                let mut next_edge = self.diagram.edge_get(edge)?.next()?;
                loop {
                    #[cfg(feature = "console_debug")]
                    print!("Inner loop next_edge={} ", next_edge.unwrap().0);

                    // it does not matter if next_edge is rejected/valid, it will be fixed by the iterator
                    let next_edges: Vec<VD::EdgeIndex> = self
                        .diagram
                        .edge_rot_next_iterator(next_edge)
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
                            let next_edges: Vec<VD::EdgeIndex> = next_edges
                                .into_iter()
                                .filter(|x| !used_edges.bit(x.0))
                                .collect();
                            if next_edges.len() == 1 {
                                // continue walking the edge line
                                let e = next_edges.first().unwrap().to_owned();
                                current_edge_set.push(e);
                                self.mark_edge_and_twin_as_used(e, used_edges)?;

                                next_edge = self.diagram.edge_get(e)?.next()?;
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
                                break;
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

                            #[cfg(feature = "console_debug")]
                            println!("0|_ Starting new set");

                            break;
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
        edges: &[VD::EdgeIndex],
        lines: &mut Vec<Line3<F>>,
        linestrings: &mut Vec<LineString3<F>>,
        maxdist: F,
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
                match self.convert_edge_to_shape(edge) {
                    Ok(linestring_3d::Shape3d::Line(l)) => lines.push(l),
                    Ok(linestring_3d::Shape3d::ParabolicArc(a)) => {
                        linestrings.push(a.discretise_3d(maxdist));
                    }
                    Ok(linestring_3d::Shape3d::Linestring(_s)) => {
                        panic!();
                    }
                    Err(_) => {
                        println!("Error :{:?}", edge);
                    }
                }
            }
            _ => {
                let mut ls = LineString3::<F>::default();
                for edge_id in edges.iter() {
                    let edge = self.diagram.edge_get(*edge_id)?;
                    match self.convert_edge_to_shape(edge)? {
                        linestring_3d::Shape3d::Line(l) => {
                            //println!("->got {:?}", l);
                            ls.push(l.start);
                            ls.push(l.end);
                            //println!("<-got");
                        }
                        linestring_3d::Shape3d::ParabolicArc(a) => {
                            //println!("->got {:?}", a);
                            ls.append(a.discretise_3d(maxdist));
                            //println!("<-got");
                        }
                        // should not happen
                        linestring_3d::Shape3d::Linestring(_s) => {
                            return Err(CenterlineError::InternalError(format!(
                                "convert_edges_to_lines() got an unexpected linestring. {}:{}",
                                file!(),
                                line!()
                            )))
                        }
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
        edge: &VD::Edge<I, F>,
    ) -> Result<linestring_3d::Shape3d<F>, CenterlineError> {
        let edge_id = edge.id();
        let edge_twin_id = self.diagram.edge_get_twin(edge_id)?;

        // Edge is finite so we know that vertex0 and vertex1 is_some()
        let vertex0 = self.diagram.vertex_get(edge.vertex0().ok_or_else(|| {
            CenterlineError::InternalError(format!(
                "Could not find vertex 0. {}:{}",
                file!(),
                line!()
            ))
        })?)?;

        let vertex1 = self.diagram.edge_get_vertex1(edge_id)?.ok_or_else(|| {
            CenterlineError::InternalError(format!(
                "Could not find vertex 1. {}:{}",
                file!(),
                line!()
            ))
        })?;
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
            let arc = linestring_2d::VoronoiParabolicArc::new(
                linestring_2d::Line2 {
                    start: segment_start_point,
                    end: segment_end_point,
                },
                cell_point,
                start_point,
                end_point,
            );
            #[cfg(feature = "console_debug")]
            println!("Converted {:?} to {:?}", edge.get_id().0, arc);
            Ok(linestring_3d::Shape3d::ParabolicArc(arc))
        } else {
            let distance_to_start = {
                if vertex0.is_site_point() {
                    F::zero()
                } else if cell.contains_point() {
                    let cell_point = self.retrieve_point(cell_id)?;
                    let cell_point = cgmath::Point2 {
                        x: Self::i2f(cell_point.x),
                        y: Self::i2f(cell_point.y),
                    };
                    -linestring::linestring_2d::distance_to_point_squared(&cell_point, &start_point)
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
                    -linestring::linestring_2d::distance_to_line_squared_safe(
                        &segment_start_point,
                        &segment_end_point,
                        &start_point,
                    )
                    .sqrt()
                }
            };
            let distance_to_end = {
                if vertex1.is_site_point() {
                    F::zero()
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
                        -linestring::linestring_2d::distance_to_point_squared(
                            &cell_point,
                            &end_point,
                        )
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
                        -linestring::linestring_2d::distance_to_line_squared_safe(
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
            Ok(linestring_3d::Shape3d::Line(line))
        }
    }

    #[inline(always)]
    pub fn i2f(input: I) -> F {
        num::cast::<I, F>(input).unwrap()
    }
}

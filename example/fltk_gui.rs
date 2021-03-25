/*
Centerline detection library.

Copyright (C) 2021 eadf https://github.com/eadf

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.

Also add information on how to contact you by electronic and paper mail.

If the program does terminal interaction, make it output a short notice like
this when it starts in an interactive mode:

intersection2d Copyright (C) 2021 eadf

This program comes with ABSOLUTELY NO WARRANTY; for details type `show w'.

This is free software, and you are welcome to redistribute it under certain
conditions; type `show c' for details.

The hypothetical commands `show w' and `show c' should show the appropriate
parts of the General Public License. Of course, your program's commands might
be different; for a GUI interface, you would use an "about box".

You should also get your employer (if you work as a programmer) or school,
if any, to sign a "copyright disclaimer" for the program, if necessary. For
more information on this, and how to apply and follow the GNU GPL, see <https://www.gnu.org/licenses/>.

The GNU General Public License does not permit incorporating your program
into proprietary programs. If your program is a subroutine library, you may
consider it more useful to permit linking proprietary applications with the
library. If this is what you want to do, use the GNU Lesser General Public
License instead of this License. But first, please read <https://www.gnu.org/
licenses /why-not-lgpl.html>.
*/
#![feature(hash_drain_filter)]

use centerline::Centerline;
use centerline::CenterlineError;
use cgmath::Transform;
use cgmath::{Point2, SquareMatrix};

use fltk::app::redraw;
use fltk::valuator::HorNiceSlider;
use fltk::*;
use fltk::{app, draw::*, frame::*};

use cgmath;
use linestring::cgmath_2d::{Aabb2, LineStringSet2, SimpleAffine};
use linestring::cgmath_3d;
use linestring::cgmath_3d::LineString3;
use obj;
use ordered_float::OrderedFloat;
//use rayon::prelude::*;
#[allow(unused_imports)]
use itertools::Itertools;
use std::cell::{RefCell, RefMut};
use std::fs::File;
use std::io::BufReader;
use std::rc::Rc;

// frame size
const HF: i32 = 590;
const WF: i32 = 790;

// window size
#[allow(dead_code)]
const H: i32 = 650;
const W: i32 = 800;

type T = f32;

#[derive(Debug, Clone, Copy)]
pub enum GuiMessage {
    SliderPreChanged(f64),
    SliderPostChanged(f64),
    SliderDotChanged(T),
}

/// Data containing an individual shape, this will be processed by a single thread.
struct Shape {
    // the input data after it has been transformed to our coordinate system
    // but not simplified.
    raw_data: LineStringSet2<T>,

    // centerline.segments is the simplified version of 'raw_data', also input to boost voronoi
    centerline: Option<Centerline<i32, T>>,

    simplified_centerline: Option<Vec<LineString3<T>>>,
}

#[derive(Debug, Clone, Copy)]
struct Configuration {
    window_center: (i32, i32),
    input_distance: f64,
    input_distance_dirty: bool,
    centerline_distance: T,
    centerline_scaled_distance: T,
    centerline_distance_dirty: bool,
    normalized_dot: T,
    normalized_dot_dirty: bool,
    last_message: Option<GuiMessage>,
}

struct SharedData {
    shapes: Option<Vec<Shape>>,
    configuration: Configuration,
    affine: SimpleAffine<T>,
}

fn main() -> Result<(), CenterlineError> {
    let app = app::App::default();
    let mut wind = window::Window::default()
        .with_size(W, HF + 150)
        .center_screen()
        .with_label("Centerline finder");

    let mut frame = Frame::new(5, 5, WF, HF, "");
    frame.set_color(Color::Black);
    frame.set_frame(FrameType::DownBox);

    let mut slider_pre =
        HorNiceSlider::new(5, 5 + HF, WF, 25, "Input data simplification distance");
    slider_pre.set_value(1.0);
    slider_pre.set_frame(FrameType::PlasticUpBox);
    slider_pre.set_color(Color::White);

    let mut slider_dot = HorNiceSlider::new(5, 5 + HF + 50, WF, 25, "normalized dot product limit");
    slider_dot.set_value(0.38);
    slider_dot.set_frame(FrameType::PlasticUpBox);
    slider_dot.set_color(Color::White);

    let mut slider_post = HorNiceSlider::new(
        5,
        5 + HF + 100,
        WF,
        25,
        "Centerline data simplification distance",
    );
    slider_post.set_value(1.0);
    slider_post.set_frame(FrameType::PlasticUpBox);
    slider_post.set_color(Color::White);

    wind.set_color(Color::White);
    wind.end();
    wind.show();

    let shared_data_rc = Rc::new(RefCell::new(SharedData {
        shapes: None,
        configuration: Configuration {
            window_center: (WF / 2, HF / 2),
            centerline_distance: 0.0,
            centerline_scaled_distance: 256.0,
            centerline_distance_dirty: true,
            normalized_dot: 0.38,
            normalized_dot_dirty: true,
            input_distance: 0.0,
            input_distance_dirty: true,
            last_message: None,
        },
        affine: SimpleAffine::default(),
    }));

    let (sender, receiver) = app::channel::<GuiMessage>();
    sender.send(GuiMessage::SliderPreChanged(50.0));
    slider_pre.set_callback2(move |s| {
        let value = s.value() as f64 * 100.0;
        s.set_label(
            ("               Input data simplification distance:".to_string()
                + &value.to_string()
                + (&"               ".to_string()))
                .as_str(),
        );
        sender.send(GuiMessage::SliderPreChanged(value));
    });

    slider_dot.set_callback2(move |s| {
        let value = s.value() as T;
        s.set_label(
            ("               Normalized dot limit:".to_string()
                + &value.to_string()
                + (&"               ".to_string()))
                .as_str(),
        );
        sender.send(GuiMessage::SliderDotChanged(value));
    });

    slider_post.set_callback2(move |s| {
        let value = s.value() as f64 * 100.0;
        s.set_label(
            ("               Centerline simplification distance:".to_string()
                + &value.to_string()
                + (&"               ".to_string()))
                .as_str(),
        );
        sender.send(GuiMessage::SliderPostChanged(value));
    });

    add_data(Rc::clone(&shared_data_rc))?;

    let shared_data_c = Rc::clone(&shared_data_rc);
    // This is called whenever the window is drawn and redrawn (in the event loop)
    wind.draw(move || {
        let draw_fn = |line: Result<[T; 4], _>, cross: bool| {
            if let Ok(line) = line {
                let (x1, y1, x2, y2) = (
                    line[0] as i32,
                    line[1] as i32,
                    line[2] as i32,
                    line[3] as i32,
                );
                draw::draw_line(x1, y1, x2, y2);
                if cross {
                    draw::draw_line(x1 - 2, y1 - 2, x1 + 2, y1 + 2);
                    draw::draw_line(x1 + 2, y1 - 2, x1 - 2, y1 + 2);
                }
            }
        };

        let mut data_bm: RefMut<_> = shared_data_c.borrow_mut();

        set_draw_color(Color::White);
        draw_rectf(5, 5, WF, HF);
        set_line_style(LineStyle::Solid, 1);

        let opt_shapes = data_bm.shapes.take();
        if let Some(vec_shapes) = opt_shapes {
            let cross = match data_bm.configuration.last_message {
                Some(GuiMessage::SliderPreChanged(_)) => true,
                _ => false,
            };
            for shape in vec_shapes.iter() {
                if let Some(ref centerline) = shape.centerline {
                    draw::set_line_style(LineStyle::Solid, 2);
                    draw::set_draw_color(Color::Red);
                    for a_line in centerline.segments.iter() {
                        draw_fn(
                            data_bm.affine.transform_ab_a([
                                a_line.start.x as T,
                                a_line.start.y as T,
                                a_line.end.x as T,
                                a_line.end.y as T,
                            ]),
                            cross,
                        );
                    }
                    draw::set_line_style(LineStyle::Solid, 1);
                    draw::set_draw_color(Color::Black);
                    for a_line in centerline.lines.iter().flatten() {
                        draw_fn(
                            data_bm.affine.transform_ab_a([
                                a_line.start.x as T,
                                a_line.start.y as T,
                                a_line.end.x as T,
                                a_line.end.y as T,
                            ]),
                            false,
                        )
                    }
                    draw::set_draw_color(Color::Blue);
                    let cross = match data_bm.configuration.last_message {
                        Some(GuiMessage::SliderPostChanged(_)) => true,
                        _=> false
                    };
                    for a_linestring in shape.simplified_centerline.iter().flatten() {
                        //println!("an_arc start_point:{:?} end_point:{:?} cell_point:{:?} segment:{:?}", an_arc.start_point, an_arc.end_point, an_arc.cell_point, an_arc.segment);
                        //let lines = an_arc.discretise_2d(1000.0);
                        //println!("an_arc.len()={:?}", lines.points().len() );
                        for a_line in a_linestring.as_lines().iter() {

                            draw_fn(
                                data_bm.affine.transform_ab_a([
                                    a_line.start.x as T,
                                    a_line.start.y as T,
                                    a_line.end.x as T,
                                    a_line.end.y as T,
                                ]),
                                cross,
                            );
                        }
                    }
                    /*#[cfg(feature = "console_debug")]
                    if let Some(ref centerline) = shape.centerline {
                        draw::set_draw_color(Color::Black);
                        if let Some(ref debug_edges) = centerline.debug_edges {
                            for e in debug_edges
                                .iter()
                                .map(|x| x.0)
                                .sorted_unstable()
                                .map(|x| debug_edges.get_key_value(x))
                            {
                                let (e, l) = e.unwrap();
                                let x1 =
                                    ((l[0] as i32) >> 8) + data_bm.configuration.window_center.0;
                                let y1 =
                                    ((l[1] as i32) >> 8) + data_bm.configuration.window_center.1;
                                let x2 =
                                    ((l[2] as i32) >> 8) + data_bm.configuration.window_center.0;
                                let y2 =
                                    ((l[3] as i32) >> 8) + data_bm.configuration.window_center.1;
                                draw::draw_line(x1, y1, x2, y2);
                                //let mid_x = (x1 + x2) / 2;
                                //let mid_y = (y1 + y2) / 2;
                                //draw_text(e.to_string().as_str(), x1, y1);
                                //println!("drawing edge {}:({},{}),({},{})", e, x1, y1, x2, y2);
                            }
                        }
                    }*/
                }
            }
            data_bm.shapes = Some(vec_shapes);
        }
    });

    let shared_data_c = Rc::clone(&shared_data_rc);
    let mut mouse_drag: Option<(i32, i32)> = None;
    wind.handle(move |ev| match ev {
        fltk::enums::Event::MouseWheel => {
            let event = &app::event_coords();
            let mut shared_data_bm = shared_data_c.borrow_mut();
            let event_dy = app::event_dy();
            let reverse_middle = shared_data_bm
                .affine
                .transform_ba(&cgmath::Point2::from([event.0 as T, event.1 as T]));
            if reverse_middle.is_err() {
                println!("{:?}", reverse_middle.err().unwrap());
                return false;
            }
            let reverse_middle = reverse_middle.unwrap();
            if event_dy != 0 {
                shared_data_bm.affine.scale *= 1.01_f32.powf(event_dy as T);
            }
            let new_middle = shared_data_bm.affine.transform_ab(&cgmath::Point2::from([
                reverse_middle[0] as T,
                reverse_middle[1] as T,
            ]));
            if new_middle.is_err() {
                println!("{:?}", new_middle.err().unwrap());
                return false;
            }
            let new_middle = new_middle.unwrap();
            // When zooming we want the center of screen remain at the same relative position.
            shared_data_bm.affine.b_offset[0] += (event.0 as T) - new_middle[0];
            shared_data_bm.affine.b_offset[1] += (event.1 as T) - new_middle[1];

            //println!("mouse wheel at dy:{:?} scale:{:?}", event_dy, shared_data_bm.visualizer.affine.scale);
            redraw();
            true
        }
        fltk::enums::Event::Drag => {
            let event = &app::event_coords();
            if mouse_drag.is_none() {
                mouse_drag = Some(*event);
            } else {
                let md = mouse_drag.unwrap();
                let mut shared_data_bm = shared_data_c.borrow_mut();
                shared_data_bm.affine.b_offset[0] += (event.0 - md.0) as T;
                shared_data_bm.affine.b_offset[1] += (event.1 - md.1) as T;
                mouse_drag = Some(*event);
                redraw();
            }
            true
        }
        fltk::enums::Event::Released => {
            if mouse_drag.is_some() {
                mouse_drag = None;
            }
            true
        }
        _ => false,
    });

    let shared_data_c = Rc::clone(&shared_data_rc);
    while app.wait() {
        if let Some(msg) = receiver.recv() {
            let mut shared_data_bm: RefMut<_> = shared_data_c.borrow_mut();
            match msg {
                GuiMessage::SliderPreChanged(value) => {
                    shared_data_bm.configuration.input_distance = value;
                    shared_data_bm.configuration.input_distance_dirty = true;
                }
                GuiMessage::SliderDotChanged(value) => {
                    shared_data_bm.configuration.normalized_dot = value;
                    shared_data_bm.configuration.normalized_dot_dirty = true;
                }
                GuiMessage::SliderPostChanged(value) => {
                    shared_data_bm.configuration.centerline_distance = value as T;
                    shared_data_bm.configuration.centerline_distance_dirty = true;
                }
            }
            shared_data_bm.configuration.last_message = Some(msg);
            re_calculate(shared_data_bm);
            redraw();
        }
    }
    Ok(())
}

/// Re-calculate the center-line and all of the other parameters
/// Todo: rayon the whole chain per shape
fn re_calculate(mut shared_data_bm: RefMut<SharedData>) {
    #[cfg(feature = "console_debug")]
    {
        println!("***********************");
        println!("re_calculate()");
    }
    let shapes = shared_data_bm.shapes.take();
    let configuration = shared_data_bm.configuration.clone();
    if let Some(mut shapes) = shapes {
        shapes = shapes
            .into_iter() //into_par_iter()
            .filter_map(|x| threaded_re_calculate_error_handler(x, configuration))
            .collect();
        shared_data_bm.shapes = Some(shapes);
    }
    shared_data_bm.configuration.normalized_dot_dirty = false;
    shared_data_bm.configuration.centerline_distance_dirty = false;
    shared_data_bm.configuration.input_distance_dirty = false;
    redraw();
}

fn threaded_re_calculate_error_handler(
    shape: Shape,
    configuration: Configuration,
) -> Option<Shape> {
    let rv = threaded_re_calculate(shape, configuration);
    match rv {
        Err(e) => {
            println!("Error {:?}", e);
            None
        }
        Ok(shape) => Some(shape),
    }
}

/// This is what a single thread is supposed to do in sequence
fn threaded_re_calculate(
    mut shape: Shape,
    configuration: Configuration,
) -> Result<Shape, CenterlineError> {
    if shape.centerline.is_none() {
        shape.centerline = Some(Centerline::<i32, T>::default());
    }
    let input_changed = if configuration.input_distance_dirty {
        recalculate_voronoi_input(&mut shape, configuration)?;
        true
    } else {
        false
    };
    let diagram_changed = if shape.centerline.is_none() || input_changed {
        recalculate_voronoi_diagram(&mut shape, configuration)?;
        true
    } else {
        false
    };
    let centerline_changed =
        if shape.centerline.is_none() || diagram_changed || configuration.normalized_dot_dirty {
            recalculate_centerline(&mut shape, configuration)?;
            true
        } else {
            false
        };
    let _centerline_changed = if shape.simplified_centerline.is_none()
        || centerline_changed
        || configuration.centerline_distance_dirty
    {
        simplify_centerline(&mut shape, configuration)?;
        true
    } else {
        false
    };

    Ok(shape)
}

#[allow(unused_variables)]
/// Re-calculate voronoi diagram
fn recalculate_voronoi_diagram(
    shape: &mut Shape,
    configuration: Configuration,
) -> Result<(), CenterlineError> {
    #[cfg(feature = "console_debug")]
    println!("recalculate_voronoi_diagram()");
    if let Some(mut centerline_rw) = shape.centerline.take() {
        centerline_rw.build_voronoi()?;
        shape.centerline = Some(centerline_rw);
    }
    Ok(())
}

/// Re-calculate centerline
fn recalculate_centerline(
    shape: &mut Shape,
    configuration: Configuration,
) -> Result<(), CenterlineError> {
    #[cfg(feature = "console_debug")]
    println!("recalculate_centerline()");
    if let Some(ref mut centerline) = shape.centerline {
        centerline.calculate_centerline(
            configuration.normalized_dot,
            configuration.centerline_scaled_distance,
        )?;
        //println!("centerline.lines.len() = {:?}", centerline.lines.len());
    } else {
        dbg!(shape.centerline.is_none());
        println!("centerline was none");
    }
    Ok(())
}

/// simplify centerline
fn simplify_centerline(
    shape: &mut Shape,
    configuration: Configuration,
) -> Result<(), CenterlineError> {
    #[cfg(feature = "console_debug")]
    println!("simplify_centerline()");
    let mut simplified_centerline =
        if let Some(simplified_centerline) = shape.simplified_centerline.take() {
            simplified_centerline
        } else {
            Vec::<LineString3<T>>::new()
        };
    simplified_centerline.clear();
    if let Some(ref centerline) = shape.centerline {
        if let Some(ref line_strings) = centerline.line_strings {
            for ls in line_strings.iter() {
                simplified_centerline
                    .push(ls.simplify(configuration.centerline_distance * 256_f32.sqrt()));
            }
        }
    }
    shape.simplified_centerline = Some(simplified_centerline);
    Ok(())
}

/// Re-calculate voronoi input geometry
/// Todo: self intersection test -> fail
fn recalculate_voronoi_input(
    shape: &mut Shape,
    configuration: Configuration,
) -> Result<(), CenterlineError> {
    #[cfg(feature = "console_debug")]
    println!("recalculate_voronoi_input()");
    if let Some(mut centerline) = shape.centerline.take() {
        centerline.segments.clear();
        for lines in shape.raw_data.set().iter() {
            //let mut set = Vec::<boostvoronoi::Line<i64>>::with_capacity(lines.len());
            if configuration.input_distance > 0.0 {
                #[cfg(feature = "console_debug")]
                let before = lines.len();
                let s_lines = lines.simplify((configuration.input_distance as T) * 256_f32.sqrt());
                #[cfg(feature = "console_debug")]
                println!(
                    "reduced by {} points of {} = {}",
                    before - s_lines.len(),
                    before,
                    s_lines.len()
                );
                for lineseq in s_lines.as_lines().iter() {
                    centerline.segments.push(boostvoronoi::Line::new(
                        boostvoronoi::Point {
                            x: lineseq.start.x as i32,
                            y: lineseq.start.y as i32,
                        },
                        boostvoronoi::Point {
                            x: lineseq.end.x as i32,
                            y: lineseq.end.y as i32,
                        },
                    ));
                }
            } else {
                #[cfg(feature = "console_debug")]
                println!("no reduction");
                for lineseq in lines.as_lines().iter() {
                    centerline.segments.push(boostvoronoi::Line::new(
                        boostvoronoi::Point {
                            x: lineseq.start.x as i32,
                            y: lineseq.start.y as i32,
                        },
                        boostvoronoi::Point {
                            x: lineseq.end.x as i32,
                            y: lineseq.end.y as i32,
                        },
                    ))
                }
            };
            #[cfg(feature = "console_debug")]
            println!("1set.len() {}", centerline.segments.len());
        }
        shape.centerline = Some(centerline);
    }
    Ok(())
}

/// Add data to the input lines.
fn add_data(data: Rc<RefCell<SharedData>>) -> Result<(), CenterlineError> {
    let mut data_bm = data.borrow_mut();

    let obj_set = {
        let input = BufReader::new(File::open("example/rust.obj").unwrap());
        obj::raw::parse_obj(input)?
    };

    let lines = centerline::remove_internal_edges(obj_set)?;
    let mut total_aabb = cgmath_3d::Aabb3::default();
    for l in lines.iter() {
        total_aabb.update_aabb(l.get_aabb());
    }
    #[cfg(feature = "console_debug")]
    println!("total aabb b4:{:?}", total_aabb);

    let (_plane, transform, voronoi_input_aabb) = get_transform(&total_aabb)?;
    // transform each linestring to 2d
    let mut raw_data: Vec<LineStringSet2<T>> = lines
        .iter()
        .map(|x| x.transform(&transform).copy_to_2d(cgmath_3d::Plane::XY))
        .collect();
    {
        let mut screen_aabb = Aabb2::new(&Point2::new(W as T, H as T));
        screen_aabb.update_point(&Point2::new(0.0, 0.0));
        data_bm.affine = SimpleAffine::new(&voronoi_input_aabb, &screen_aabb)?;
    }

    #[cfg(feature = "console_debug")]
    println!("Started with {} shapes", raw_data.len());

    // try to consolidate shapes. If one AABB (a) totally engulfs another shape (b)
    // we put shape (b) inside (a)
    'outer_loop: loop {
        // redo *every* test if something is changed until nothing else can be done
        for i in 0..raw_data.len() {
            for j in i + 1..raw_data.len() {
                if raw_data[i].get_aabb().contains_aabb(raw_data[j].get_aabb()) {
                    // move stuff from j to i via a temp
                    let mut line_j_steal = LineStringSet2::default();
                    {
                        let line_j = raw_data.get_mut(j).unwrap();
                        line_j_steal.take_from(line_j);
                    }
                    let line_i = raw_data.get_mut(i).unwrap();

                    line_i.take_from(&mut line_j_steal);
                    let _ = raw_data.remove(j);
                    continue 'outer_loop;
                } else if raw_data[j].get_aabb().contains_aabb(raw_data[i].get_aabb()) {
                    // move stuff from i to j via a temp
                    let mut line_i_steal = LineStringSet2::default();
                    {
                        let line_i = raw_data.get_mut(i).unwrap();
                        line_i_steal.take_from(line_i);
                    }
                    let line_j = raw_data.get_mut(j).unwrap();

                    line_j.take_from(&mut line_i_steal);
                    let _ = raw_data.remove(i);
                    continue 'outer_loop;
                }
            }
        }
        break 'outer_loop;
    }
    #[cfg(feature = "console_debug")]
    println!("Reduced to {} shapes", raw_data.len());
    data_bm.shapes = Some(
        raw_data
            .into_iter()
            .map(|x| Shape {
                raw_data: x,
                centerline: None,
                simplified_centerline: None,
            })
            .collect(),
    );

    Ok(())
}

/// Calculate an affine transform that will center, flip plane to XY, and scale the arbitrary shape
/// so that it will fill the screen. For good measure the scale is then multiplied by 256 so the
/// points makes half decent input data to boost voronoi (integer input only)
fn get_transform(
    total_aabb: &cgmath_3d::Aabb3<T>,
) -> Result<(cgmath_3d::Plane, cgmath::Matrix4<T>, Aabb2<T>), CenterlineError> {
    let plane = if let Some(plane) = cgmath_3d::Plane::get_plane(total_aabb) {
        plane
    } else {
        return Err(CenterlineError::InputNotPLane);
    };

    //println!("Total aabb {:?} Plane={:?}", total_aabb, plane);

    let low = total_aabb.get_low().unwrap();
    let high = total_aabb.get_high().unwrap();
    let center = cgmath::point3(
        (high.x + low.x) / 2.0,
        (high.y + low.y) / 2.0,
        (high.z + low.z) / 2.0,
    );
    println!(
        "Obj file AABB: Center {:?} low:{:?}, high:{:?}",
        center, low, high
    );

    let scale_transform = {
        let scale = 256.0 * (HF.min(WF) as T - 10.0)
            / std::cmp::max(
                std::cmp::max(OrderedFloat(high.x - low.x), OrderedFloat(high.y - low.y)),
                OrderedFloat(high.z - low.z),
            )
            .into_inner();

        cgmath::Matrix4::from_scale(scale)
    };

    let center = scale_transform.transform_point(center);
    let center_transform =
        cgmath::Matrix4::from_translation(cgmath::Vector3::new(-center.x, -center.y, -center.z));

    let plane_transform = {
        let x = cgmath::Vector4::new(1., 0., 0., 0.);
        let y = cgmath::Vector4::new(0., 1., 0., 0.);
        let z = cgmath::Vector4::new(0., 0., 1., 0.);
        let w = cgmath::Vector4::new(0., 0., 0., 1.);

        match plane {
            cgmath_3d::Plane::XY => cgmath::Matrix4::from_cols(x, y, z, w),
            cgmath_3d::Plane::XZ => cgmath::Matrix4::from_cols(x, z, y, w),
            cgmath_3d::Plane::ZY => cgmath::Matrix4::from_cols(z, y, x, w),
        }
    };

    let total_transform = plane_transform * center_transform * scale_transform;

    let low0 = total_aabb.get_low().unwrap();
    let high0 = total_aabb.get_high().unwrap();
    #[cfg(feature = "console_debug")]
    let center0 = cgmath::point3(
        (high0.x + low0.x) / 2.0,
        (high0.y + low0.y) / 2.0,
        (high0.z + low0.z) / 2.0,
    );

    let low0 = total_transform.transform_point(low0);
    let high0 = total_transform.transform_point(high0);
    #[cfg(feature = "console_debug")]
    let center0 = total_transform.transform_point(center0);
    let mut voronoi_input_aabb = Aabb2::new(&Point2::new(low0.x, low.y));
    voronoi_input_aabb.update_point(&Point2::new(high0.x, high0.y));

    #[cfg(feature = "console_debug")]
    println!(
        "Voronoi input AABB: Center {:?} low:{:?}, high:{:?}",
        center0, low0, high0
    );
    println!("Voronoi input AABB: {:?}", voronoi_input_aabb);

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

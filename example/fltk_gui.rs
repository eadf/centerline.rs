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

use centerline::CenterlineError;
use cgmath::SquareMatrix;
use cgmath::Transform;
use fltk::app::redraw;
use fltk::valuator::HorNiceSlider;
use fltk::*;
use fltk::{app, button::*, draw::*, frame::*, window::*};

use linestring::cgmath_2d;
use linestring::cgmath_2d::LineStringSet2;
use linestring::cgmath_3d;
//use linestring::cgmath_3d::LineStringSet3;
use cgmath;
use obj;
use ordered_float::OrderedFloat;
use std::cell::{RefCell, RefMut};
use std::fs::File;
use std::io::BufReader;
use std::rc::Rc;

// frame size
const HF: i32 = 590;
const WF: i32 = 790;

// window size
const H: i32 = 650;
const W: i32 = 800;

#[derive(Debug, Clone, Copy)]
pub enum GuiMessage {
    SliderPreChanged(f64),
    SliderPostChanged(f64),
    SliderDotChanged(f64),
}

struct SharedData {
    // the input data after it has been transformed to our coordinate system
    // but not simplified.
    raw_data: Vec<LineStringSet2<f32>>,
    // the simplified data, also input to boost voronoi
    // This should never be None (unless during iteration when the value is take():en)
    voronoi_input: Option<Vec<Vec<boostvoronoi::Line<i64>>>>,
    // the unmodified voronoi output
    //voronoi_output: Vec<something>,
    window_center: (i32, i32),
    input_distance: f64,
    input_distance_dirty: bool,
    centerline_distance: f64,
    centerline_distance_dirty: bool,
    normalized_dot: f64,
    normalized_dot_dirty: bool,
    lastMessage: Option<GuiMessage>,
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
    slider_pre.set_value(0.0);
    slider_pre.set_frame(FrameType::PlasticUpBox);
    slider_pre.set_color(Color::White);

    let mut slider_dot = HorNiceSlider::new(5, 5 + HF + 50, WF, 25, "normalized dot product limit");
    slider_dot.set_value(0.0);
    slider_dot.set_frame(FrameType::PlasticUpBox);
    slider_dot.set_color(Color::White);

    let mut slider_post = HorNiceSlider::new(
        5,
        5 + HF + 100,
        WF,
        25,
        "Centerline data simplification distance",
    );
    slider_post.set_value(0.0);
    slider_post.set_frame(FrameType::PlasticUpBox);
    slider_post.set_color(Color::White);

    wind.set_color(Color::White);
    wind.end();
    wind.show();
    let shared_data_rc = Rc::from(RefCell::from(SharedData {
        raw_data: Vec::default(),
        voronoi_input: Some(Vec::default()),
        window_center: (WF / 2, HF / 2),
        centerline_distance: 0.0,
        centerline_distance_dirty: true,
        normalized_dot: 0.0,
        normalized_dot_dirty: true,
        input_distance: 0.0,
        input_distance_dirty: true,
        lastMessage: None,
    }));

    let (sender, receiver) = app::channel::<GuiMessage>();
    sender.send(GuiMessage::SliderPreChanged(0.0));
    slider_pre.set_callback2(move |s| {
        let value = s.value() as f64 * 50.0;
        s.set_label(
            ("               Input data simplification distance:".to_string()
                + &value.to_string()
                + (&"               ".to_string()))
                .as_str(),
        );
        sender.send(GuiMessage::SliderPreChanged(value));
    });

    slider_dot.set_callback2(move |s| {
        let value = s.value() as f64 * 400.0;
        s.set_label(
            ("               Normalized dot limit:".to_string()
                + &value.to_string()
                + (&"               ".to_string()))
                .as_str(),
        );
        sender.send(GuiMessage::SliderDotChanged(value));
    });

    slider_post.set_callback2(move |s| {
        let value = s.value() as f64 * 50.0;
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
        let mut data_bm = shared_data_c.borrow_mut();

        set_draw_color(Color::White);
        draw_rectf(5, 5, WF, HF);
        set_line_style(LineStyle::Solid, 1);
        draw::set_draw_color(Color::Blue);

        let voronoi_input = data_bm.voronoi_input.take().unwrap();
        for set in voronoi_input.iter() {
            println!("2set.len() {}", set.len());
            for a_line in set.iter() {
                // The scaling transform is already multiplied by 1024
                let x1 = (a_line.start.x >> 10) as i32 + data_bm.window_center.0;
                let y1 = (a_line.start.y >> 10) as i32 + data_bm.window_center.1;
                let x2 = (a_line.end.x >> 10) as i32 + data_bm.window_center.0;
                let y2 = (a_line.end.y >> 10) as i32 + data_bm.window_center.1;
                draw::draw_line(x1, y1, x2, y2);
                match data_bm.lastMessage {
                    Some(GuiMessage::SliderPreChanged(_)) => {
                        draw::draw_line(x1 - 2, y1 - 2, x1 + 2, y1 + 2);
                        draw::draw_line(x1 + 2, y1 - 2, x1 - 2, y1 + 2);
                    }
                    _ => (),
                }
            }
        }
        data_bm.voronoi_input = Some(voronoi_input);
    });

    let shared_data_c = Rc::clone(&shared_data_rc);
    wind.handle(move |ev| match ev {
        enums::Event::Released => {
            let event = &app::event_coords();
            println!("mouse at {:?}", event);
            true
        }
        _ => false,
    });

    let shared_data_c = Rc::clone(&shared_data_rc);
    while app.wait() {
        if let Some(msg) = receiver.recv() {
            let mut shared_data_bm = shared_data_c.borrow_mut();
            match msg {
                GuiMessage::SliderPreChanged(value) => {
                    shared_data_bm.input_distance = value;
                    shared_data_bm.input_distance_dirty = true;
                }
                GuiMessage::SliderDotChanged(value) => {
                    shared_data_bm.normalized_dot = value;
                    shared_data_bm.normalized_dot_dirty = true;
                }
                GuiMessage::SliderPostChanged(value) => {
                    shared_data_bm.centerline_distance = value;
                    shared_data_bm.centerline_distance_dirty = true;
                }
            }
            shared_data_bm.lastMessage = Some(msg);
            re_calculate(&mut shared_data_bm);
            redraw();
        }
    }
    Ok(())
}

/// Re-calculate the center-line and all of the other parameters
/// Todo: rayon the whole chain per shape
fn re_calculate(shared_data_bm: &mut RefMut<SharedData>) {
    let input_changed = if shared_data_bm.input_distance_dirty{
        recalculate_voronoi_input(shared_data_bm);
        true
    } else {false};
    let diagram_changed = if input_changed || shared_data_bm.normalized_dot_dirty {
        recalculate_voronoi_diagram(shared_data_bm);
        true
    } else {false};
    let _centerline_changed = if diagram_changed || shared_data_bm.centerline_distance_dirty {
        recalculate_centerline(shared_data_bm);
        true
    } else {false};
    shared_data_bm.normalized_dot_dirty = false;
    shared_data_bm.centerline_distance_dirty = false;
    shared_data_bm.input_distance_dirty = false;
    redraw();
}

/// Re-calculate voronoi diagram
fn recalculate_voronoi_diagram(shared_data_bm: &mut RefMut<SharedData>)  {
}

/// Re-calculate centerline
fn recalculate_centerline(shared_data_bm: &mut RefMut<SharedData>) {
}

/// Re-calculate voronoi input geometry
/// Todo: self intersection test -> fail
fn recalculate_voronoi_input(shared_data_bm: &mut RefMut<SharedData>) {

    let mut voronoi_input = shared_data_bm.voronoi_input.take().unwrap();
    voronoi_input.clear();

    for line_sets in shared_data_bm.raw_data.iter() {
        for lines in line_sets.set().iter() {
            let mut set = Vec::<boostvoronoi::Line<i64>>::with_capacity(lines.len());
            if shared_data_bm.input_distance > 0.0 {
                let before = lines.len();
                let s_lines =
                    lines.simplify((shared_data_bm.input_distance as f32) * 1024_f32.sqrt());
                println!(
                    "reduced by {} points of {} = {}",
                    before - s_lines.len(),
                    before,
                    s_lines.len()
                );
                for lineseq in s_lines.as_lines().iter() {
                    set.push(boostvoronoi::Line::new(
                        boostvoronoi::Point {
                            x: lineseq.start.x as i64,
                            y: lineseq.start.y as i64,
                        },
                        boostvoronoi::Point {
                            x: lineseq.end.x as i64,
                            y: lineseq.end.y as i64,
                        },
                    ));
                }
            } else {
                println!("no reduction");
                for lineseq in lines.as_lines().iter() {
                    set.push(boostvoronoi::Line::new(
                        boostvoronoi::Point {
                            x: lineseq.start.x as i64,
                            y: lineseq.start.y as i64,
                        },
                        boostvoronoi::Point {
                            x: lineseq.end.x as i64,
                            y: lineseq.end.y as i64,
                        },
                    ))
                }
            };
            println!("1set.len() {}", set.len());
            voronoi_input.push(set);
        }
    }
    shared_data_bm.voronoi_input = Some(voronoi_input);
}

/// Add data to the input lines.
fn add_data(data: Rc<RefCell<SharedData>>) -> Result<(), CenterlineError> {
    let mut data_bm = data.borrow_mut();

    let obj_set = {
        let input = BufReader::new(File::open("example/rust.obj").unwrap());
        obj::raw::parse_obj(input)?
    };

    let mut lines = centerline::remove_internal_edges(obj_set)?;
    let mut total_aabb = cgmath_3d::Aabb3::default();
    for l in lines.iter() {
        total_aabb.update_aabb(l.get_aabb());
    }
    println!("total aabb b4:{:?}", total_aabb);

    let (_plane, transform) = get_transform(&total_aabb)?;
    // transform each linestring to 2d
    let mut lines: Vec<LineStringSet2<f32>> = lines
        .iter()
        .map(|x| x.transform(&transform).copy_to_2d(cgmath_3d::Plane::XY))
        .collect();
    data_bm.raw_data = lines;

    println!("Started with {} shapes", data_bm.raw_data.len());

    // try to consolidate shapes. If one AABB (a) totally engulfs another shape (b)
    // we put shape (b) inside (a)
    'outer_loop: loop {
        // redo *every* test if something is changed until nothing else can be done
        for i in 0..data_bm.raw_data.len() {
            for j in i + 1..data_bm.raw_data.len() {
                if data_bm.raw_data[i]
                    .get_aabb()
                    .contains_aabb(data_bm.raw_data[j].get_aabb())
                {
                    // move stuff from j to i via a temp
                    let mut line_j_steal = LineStringSet2::default();
                    {
                        let line_j = data_bm.raw_data.get_mut(j).unwrap();
                        line_j_steal.take_from(line_j);
                    }
                    let line_i = data_bm.raw_data.get_mut(i).unwrap();

                    line_i.take_from(&mut line_j_steal);
                    let _ = data_bm.raw_data.remove(j);
                    continue 'outer_loop;
                } else if data_bm.raw_data[j]
                    .get_aabb()
                    .contains_aabb(data_bm.raw_data[i].get_aabb())
                {
                    // move stuff from i to j via a temp
                    let mut line_i_steal = LineStringSet2::default();
                    {
                        let line_i = data_bm.raw_data.get_mut(i).unwrap();
                        line_i_steal.take_from(line_i);
                    }
                    let line_j = data_bm.raw_data.get_mut(j).unwrap();

                    line_j.take_from(&mut line_i_steal);
                    let _ = data_bm.raw_data.remove(i);
                    continue 'outer_loop;
                }
            }
        }
        break 'outer_loop;
    }

    println!("Reduced to {} shapes", data_bm.raw_data.len());

    let mut voronoi_input = data_bm.voronoi_input.take();
    if let Some(mut voronoi_input) = voronoi_input {
        voronoi_input.clear();
        data_bm.voronoi_input = Some(voronoi_input);
    } else {
        data_bm.voronoi_input = voronoi_input;
    }

    //data_bm.input_distance_dirty = true;

    //data_b.lines
    //data_b.lines.append(&mut to_lines(&_l));
    /*
    let results = AlgorithmData::<f64>::default()
        .with_ignore_end_point_intersections(true)?
        .with_stop_at_first_intersection(true)?
        .with_ref_lines(data_b.lines.iter())?
        .compute()?;

    for (p, l) in results.iter() {
        println!("Intersection @{:?} Involved lines:{:?}", p, l);
        return Err(CenterlineError::SelfIntersectingData);
    }*/
    Ok(())
}

/// Calculate an affine transform that will center, flip plane to XY, and scale the arbitrary shape
/// so that it will fill the screen. For good measure the scale is then multiplied by 1024 so the
/// points makes half decent input data to boostvoronoi (integer input only)
fn get_transform(
    total_aabb: &cgmath_3d::Aabb3<f32>,
) -> Result<(cgmath_3d::Plane, cgmath::Matrix4<f32>), CenterlineError> {
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
    println!("0 Center {:?} low:{:?}, high:{:?}", center, low, high);

    let scale_transform = {
        let scale = 1024.0 * 700.0
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
    let center0 = cgmath::point3(
        (high0.x + low0.x) / 2.0,
        (high0.y + low0.y) / 2.0,
        (high0.z + low0.z) / 2.0,
    );

    let low0 = total_transform.transform_point(low0);
    let high0 = total_transform.transform_point(high0);
    let center0 = total_transform.transform_point(center0);
    println!("T Center {:?} low:{:?}, high:{:?}", center0, low0, high0);

    let inverse_total = total_transform.invert();
    if inverse_total.is_none() {
        return Err(CenterlineError::CouldNotCalculateInverseMatrix);
    }
    let inverse_total = inverse_total.unwrap();

    let low0 = inverse_total.transform_point(low0);
    let high0 = inverse_total.transform_point(high0);
    let center0 = inverse_total.transform_point(center0);
    println!("I Center {:?} low:{:?}, high:{:?}", center0, low0, high0);

    Ok((plane, total_transform))
}

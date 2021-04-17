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
use cgmath::{Matrix4, One};
use cgmath::{Point2, SquareMatrix};

use fltk::app::redraw;
use fltk::group::Pack;
use fltk::valuator::HorNiceSlider;
use fltk::*;
use fltk::{app, draw::*, frame::*};

use cgmath;
use fltk::button::RoundButton;
use fltk::dialog::FileDialogType;
use fltk::menu::MenuButton;
#[allow(unused_imports)]
use itertools::Itertools;
use linestring::cgmath_2d::{Aabb2, Line2, LineString2, LineStringSet2, SimpleAffine};
use linestring::cgmath_3d;
use linestring::cgmath_3d::LineString3;
use num::traits::FloatConst;
use obj;
use rayon::prelude::*;
use std::cell::{RefCell, RefMut};
use std::fs::File;
use std::io::BufReader;
use std::rc::Rc;

#[macro_use]
extern crate bitflags;

// frame size
const HF: i32 = 590;
const WF: i32 = 790;

// window size
#[allow(dead_code)]
const H: i32 = 650;
const W: i32 = 800;

// The float type used by voronoi diagram
type F = f32;

// The integer type used by voronoi diagram
type I = i32;

#[derive(Debug, Clone, Copy)]
pub enum GuiMessage {
    SliderPreChanged(F),
    SliderPostChanged(F),
    SliderDotChanged(F),
    Filter(DrawFilterFlag),
    MenuChoiceLoad,
    MenuChoiceSaveOutline,
    MenuChoiceSaveCenterLine,
}

bitflags! {
    pub struct DrawFilterFlag: u32 {
        /// Edges considered to be outside all closed input geometry
        const THREAD_GROUP_HULL     = 0b000000000000001;
        const THREAD_GROUP_AABB     = 0b000000000000010;
        const INTERNAL_GEOMETRY     = 0b000000000000100;
        const REMOVE_INTERNAL_EDGES = 0b000000000001000;
        const DRAW_ALL              = 0b111111111111111;
    }
}

/// Data containing an individual shape, this will be processed by a single thread.
struct Shape {
    // the input data after it has been transformed to our coordinate system
    // but not simplified.
    raw_data: LineStringSet2<F>,

    // centerline.segments is the simplified version of 'raw_data', also input to boost voronoi
    centerline: Option<Centerline<I, F>>,

    simplified_centerline: Option<Vec<LineString3<F>>>,
}

#[derive(Debug, Clone, Copy)]
struct Configuration {
    window_center: (i32, i32),
    input_distance: F,
    input_distance_dirty: bool,
    centerline_distance: F,
    centerline_scaled_distance: F,
    centerline_distance_dirty: bool,
    normalized_dot: F,
    normalized_dot_dirty: bool,
    last_message: Option<GuiMessage>,
    draw_flag: DrawFilterFlag,
    inverse_transform: Matrix4<F>,
}

struct SharedData {
    shapes: Option<Vec<Shape>>,
    configuration: Configuration,
    affine: SimpleAffine<F>,
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

    let mut pack = Pack::new(5, 5 + HF, (W - 10) / 2, H - 10, "");
    pack.set_spacing(27);

    let mut slider_pre = HorNiceSlider::default()
        .with_size(100, 25)
        .with_label("Input data simplification distance");
    slider_pre.set_value(1.0);
    slider_pre.set_frame(FrameType::PlasticUpBox);
    slider_pre.set_color(Color::White);

    let mut slider_dot = HorNiceSlider::default()
        .with_size(100, 25)
        .with_label("Angle: 50.0000°");
    slider_dot.set_value(0.55);
    slider_dot.set_frame(FrameType::PlasticUpBox);
    slider_dot.set_color(Color::White);

    let mut slider_post = HorNiceSlider::default()
        .with_size(100, 25)
        .with_label("Centerline data simplification distance");
    slider_post.set_value(1.0);
    slider_post.set_frame(FrameType::PlasticUpBox);
    slider_post.set_color(Color::White);

    pack.end();
    let mut pack = Pack::new(2 * 5 + (W - 10) / 2, 5 + HF, (W - 10) / 2 - 5, H - 10, "");
    pack.set_spacing(1);
    let mut menu_but = MenuButton::default().with_size(170, 25).with_label("Menu");
    menu_but.set_frame(FrameType::PlasticUpBox);

    let mut thread_group_aabb_button = RoundButton::default()
        .with_size(180, 25)
        .with_label("Thread group AABB");
    thread_group_aabb_button.toggle(false);
    thread_group_aabb_button.set_frame(FrameType::PlasticUpBox);

    let mut thread_group_hull_button = RoundButton::default()
        .with_size(180, 25)
        .with_label("Thread group convex hull");
    thread_group_hull_button.toggle(false);
    thread_group_hull_button.set_frame(FrameType::PlasticUpBox);

    let mut internal_geometry_button = RoundButton::default()
        .with_size(180, 25)
        .with_label("Internal geometry convex hull");
    internal_geometry_button.toggle(false);
    internal_geometry_button.set_frame(FrameType::PlasticUpBox);

    let mut internal_edges_button = RoundButton::default()
        .with_size(180, 25)
        .with_label("Remove internal edges");
    internal_edges_button.toggle(true);
    internal_edges_button.set_frame(FrameType::PlasticUpBox);

    pack.end();
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
            draw_flag: DrawFilterFlag::DRAW_ALL
                ^ DrawFilterFlag::THREAD_GROUP_AABB
                ^ DrawFilterFlag::THREAD_GROUP_HULL
                ^ DrawFilterFlag::INTERNAL_GEOMETRY,
            inverse_transform: Matrix4::<F>::one(),
        },
        affine: SimpleAffine::default(),
    }));

    let (sender, receiver) = app::channel::<GuiMessage>();
    sender.send(GuiMessage::SliderPreChanged(50.0));
    slider_pre.set_callback2(move |s| {
        let value = s.value() as f32 * 100.0;
        s.set_label(&format!(
            "   Input data simplification distance: {:.4}       ",
            value
        ));
        sender.send(GuiMessage::SliderPreChanged(value));
    });

    slider_dot.set_callback2(move |s| {
        let value = s.value() * 90.0;
        s.set_label(&format!("   Angle: {:.4}°      ", value));
        let value = (f64::PI() * value / 180.0).cos();
        sender.send(GuiMessage::SliderDotChanged(value as F));
    });

    slider_post.set_callback2(move |s| {
        let value = s.value() as f32 * 100.0;
        s.set_label(&format!(
            "   Centerline simplification distance: {:.4}       ",
            value
        ));
        sender.send(GuiMessage::SliderPostChanged(value));
    });

    internal_geometry_button.emit(
        sender,
        GuiMessage::Filter(DrawFilterFlag::INTERNAL_GEOMETRY),
    );
    internal_edges_button.emit(
        sender,
        GuiMessage::Filter(DrawFilterFlag::REMOVE_INTERNAL_EDGES),
    );
    thread_group_aabb_button.emit(
        sender,
        GuiMessage::Filter(DrawFilterFlag::THREAD_GROUP_AABB),
    );
    thread_group_hull_button.emit(
        sender,
        GuiMessage::Filter(DrawFilterFlag::THREAD_GROUP_HULL),
    );
    menu_but.add_emit(
        "Load from file",
        Shortcut::None,
        menu::MenuFlag::Normal,
        sender,
        GuiMessage::MenuChoiceLoad,
    );
    menu_but.add_emit(
        "Save outline to file",
        Shortcut::None,
        menu::MenuFlag::Normal,
        sender,
        GuiMessage::MenuChoiceSaveOutline,
    );
    menu_but.add_emit(
        "Save centerline to file",
        Shortcut::None,
        menu::MenuFlag::Normal,
        sender,
        GuiMessage::MenuChoiceSaveCenterLine,
    );
    {
        let data = Rc::clone(&shared_data_rc);
        add_data_from_file(data, "example/logo.obj")?;
    }
    let shared_data_c = Rc::clone(&shared_data_rc);
    // This is called whenever the window is drawn and redrawn (in the event loop)
    wind.draw(move || {
        let draw_fn = |line: Result<[F; 4], _>, cross: bool| {
            if let Ok(line) = line {
                let (x1, y1, x2, y2) = (
                    line[0] as i32,
                    line[1] as i32,
                    line[2] as i32,
                    line[3] as i32,
                );
                draw::draw_line(x1, y1, x2, y2);
                if cross {
                    // draws a little X at the end points.
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
                    // Draw the segments of this shape
                    draw::set_line_style(LineStyle::Solid, 2);
                    draw::set_draw_color(Color::Red);
                    for a_line in centerline.segments.iter() {
                        draw_fn(
                            data_bm.affine.transform_ab_a([
                                a_line.start.x as F,
                                a_line.start.y as F,
                                a_line.end.x as F,
                                a_line.end.y as F,
                            ]),
                            cross,
                        );
                    }
                    // draw the AABB of each (root level) - raw input shape
                    if data_bm
                        .configuration
                        .draw_flag
                        .contains(DrawFilterFlag::THREAD_GROUP_AABB)
                    {
                        draw::set_line_style(LineStyle::Solid, 1);
                        draw::set_draw_color(Color::Dark1);
                        let aabb: LineString2<F> = shape.raw_data.get_aabb().clone().into();
                        for a_line in aabb.as_lines_iter() {
                            draw_fn(
                                data_bm.affine.transform_ab_a([
                                    a_line.start.x as F,
                                    a_line.start.y as F,
                                    a_line.end.x as F,
                                    a_line.end.y as F,
                                ]),
                                false,
                            );
                        }
                    }
                    // draw the convex hull of each (root level) - raw input shape
                    if data_bm
                        .configuration
                        .draw_flag
                        .contains(DrawFilterFlag::THREAD_GROUP_HULL)
                    {
                        draw::set_line_style(LineStyle::Solid, 1);
                        draw::set_draw_color(Color::Dark2);
                        if let Some(ref hull) = shape.raw_data.get_convex_hull() {
                            for a_line in hull.as_lines_iter() {
                                draw_fn(
                                    data_bm.affine.transform_ab_a([
                                        a_line.start.x as F,
                                        a_line.start.y as F,
                                        a_line.end.x as F,
                                        a_line.end.y as F,
                                    ]),
                                    false,
                                );
                            }
                        }
                    }
                    if data_bm
                        .configuration
                        .draw_flag
                        .contains(DrawFilterFlag::INTERNAL_GEOMETRY)
                    {
                        // draw the internal geometry 'holes'
                        if let Some(shape_internals) = shape.raw_data.get_internals() {
                            draw::set_draw_color(Color::Green);
                            for hull in shape_internals.iter() {
                                for a_line in hull.1.as_lines_iter() {
                                    draw_fn(
                                        data_bm.affine.transform_ab_a([
                                            a_line.start.x as F,
                                            a_line.start.y as F,
                                            a_line.end.x as F,
                                            a_line.end.y as F,
                                        ]),
                                        false,
                                    );
                                }
                            }
                        }
                    }

                    draw::set_line_style(LineStyle::Solid, 1);
                    draw::set_draw_color(Color::Black);
                    for a_line in centerline.lines.iter().flatten() {
                        draw_fn(
                            data_bm.affine.transform_ab_a([
                                a_line.start.x as F,
                                a_line.start.y as F,
                                a_line.end.x as F,
                                a_line.end.y as F,
                            ]),
                            false,
                        )
                    }
                    draw::set_draw_color(Color::Blue);
                    let cross = match data_bm.configuration.last_message {
                        Some(GuiMessage::SliderPostChanged(_)) => true,
                        _ => false,
                    };
                    for a_linestring in shape.simplified_centerline.iter().flatten() {
                        //println!("an_arc start_point:{:?} end_point:{:?} cell_point:{:?} segment:{:?}", an_arc.start_point, an_arc.end_point, an_arc.cell_point, an_arc.segment);
                        //let lines = an_arc.discretise_2d(1000.0);
                        //println!("an_arc.len()={:?}", lines.points().len() );
                        for a_line in a_linestring.as_lines_iter() {
                            draw_fn(
                                data_bm.affine.transform_ab_a([
                                    a_line.start.x as F,
                                    a_line.start.y as F,
                                    a_line.end.x as F,
                                    a_line.end.y as F,
                                ]),
                                cross,
                            );
                        }
                    }
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
            //println!("mouse wheel at x:{} y:{}", event.0, event.1);

            let mut shared_data_bm = shared_data_c.borrow_mut();
            let event_dy = app::event_dy();
            let reverse_middle = shared_data_bm
                .affine
                .transform_ba(&cgmath::Point2::from([event.0 as F, event.1 as F]));
            if reverse_middle.is_err() {
                println!("{:?}", reverse_middle.err().unwrap());
                return false;
            }
            let reverse_middle = reverse_middle.unwrap();
            if event_dy != 0 {
                let scale_mod = 1.01_f32.powf(event_dy as F);
                shared_data_bm.affine.scale[0] *= scale_mod;
                shared_data_bm.affine.scale[1] *= scale_mod;
            }
            let new_middle = shared_data_bm.affine.transform_ab(&cgmath::Point2::from([
                reverse_middle[0] as F,
                reverse_middle[1] as F,
            ]));
            if new_middle.is_err() {
                println!("{:?}", new_middle.err().unwrap());
                return false;
            }
            let new_middle = new_middle.unwrap();
            // When zooming we want the center of screen remain at the same relative position.
            shared_data_bm.affine.b_offset[0] += (event.0 as F) - new_middle[0];
            shared_data_bm.affine.b_offset[1] += (event.1 as F) - new_middle[1];

            redraw();
            true
        }
        fltk::enums::Event::Drag => {
            let event = &app::event_coords();
            //println!("mouse wheel at x:{} y:{}", event.0, event.1);

            if mouse_drag.is_none() {
                mouse_drag = Some(*event);
            } else {
                let md = mouse_drag.unwrap();
                let mut shared_data_bm = shared_data_c.borrow_mut();
                shared_data_bm.affine.b_offset[0] += (event.0 - md.0) as F;
                shared_data_bm.affine.b_offset[1] += (event.1 - md.1) as F;
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

    let shared_data_c1 = Rc::clone(&shared_data_rc);
    let shared_data_c2 = Rc::clone(&shared_data_rc);
    while app.wait() {
        if let Some(msg) = receiver.recv() {
            match msg {
                GuiMessage::SliderPreChanged(value) => {
                    let mut shared_data_bm = shared_data_c1.borrow_mut();
                    shared_data_bm.configuration.input_distance = value as F;
                    shared_data_bm.configuration.input_distance_dirty = true;
                }
                GuiMessage::SliderDotChanged(value) => {
                    let mut shared_data_bm = shared_data_c1.borrow_mut();
                    shared_data_bm.configuration.normalized_dot = value;
                    shared_data_bm.configuration.normalized_dot_dirty = true;
                }
                GuiMessage::SliderPostChanged(value) => {
                    let mut shared_data_bm = shared_data_c1.borrow_mut();
                    shared_data_bm.configuration.centerline_distance = value as F;
                    shared_data_bm.configuration.centerline_distance_dirty = true;
                }
                GuiMessage::Filter(flag) => {
                    let mut shared_data_bm = shared_data_c1.borrow_mut();
                    shared_data_bm.configuration.draw_flag ^= flag;
                    if flag.contains(DrawFilterFlag::REMOVE_INTERNAL_EDGES) {
                        shared_data_bm.configuration.normalized_dot_dirty = true;
                    }
                }
                GuiMessage::MenuChoiceLoad => {
                    let mut chooser = dialog::NativeFileChooser::new(FileDialogType::BrowseDir);

                    let _ = chooser.set_directory(std::path::Path::new("examples"));
                    let _ = chooser.set_title("select your input data");
                    chooser.set_filter("*.obj");
                    chooser.show();
                    if let Some(filename) = chooser.filenames().first() {
                        let shared_data_c = Rc::clone(&shared_data_rc);
                        if let Err(err) =
                            add_data_from_file(shared_data_c, filename.to_str().unwrap())
                        {
                            println!("Failed to read file: {:?}", err);
                        }
                    }
                }
                GuiMessage::MenuChoiceSaveOutline => {
                    let mut chooser =
                        dialog::NativeFileChooser::new(FileDialogType::BrowseSaveFile);

                    let _ = chooser.set_directory(std::path::Path::new("examples"));
                    let _ = chooser.set_title("select file to save outline to");
                    chooser.set_filter("*.obj");
                    chooser.show();
                    if let Some(filename) = chooser.filenames().first() {
                        let shared_data_c = Rc::clone(&shared_data_rc);
                        let shared_data_b = shared_data_c.borrow();
                        let mut data_to_save = Vec::<Vec<cgmath_3d::Line3<F>>>::new();
                        for s in shared_data_b.shapes.iter().flatten() {
                            if let Some(ref centerline) = s.centerline {
                                data_to_save.push(
                                    centerline
                                        .segments
                                        .iter()
                                        .map(|l| {
                                            Line2::<F>::from([
                                                l.start.x as F,
                                                l.start.y as F,
                                                l.end.x as F,
                                                l.end.y as F,
                                            ])
                                            .copy_to_3d(cgmath_3d::Plane::XY)
                                            .transform(
                                                &shared_data_b.configuration.inverse_transform,
                                            )
                                        })
                                        .collect(),
                                );
                            }
                        }
                        if let Err(err) = linestring::cgmath_3d::save_to_obj_file(
                            filename.to_str().unwrap(),
                            "outline",
                            data_to_save,
                        ) {
                            println!("Failed to write file: {:?}", err);
                        }
                    }
                }
                GuiMessage::MenuChoiceSaveCenterLine => {
                    let mut chooser =
                        dialog::NativeFileChooser::new(FileDialogType::BrowseSaveFile);

                    let _ = chooser.set_directory(std::path::Path::new("examples"));
                    let _ = chooser.set_title("select file to save outline to");
                    chooser.set_filter("*.obj");
                    chooser.show();
                    if let Some(filename) = chooser.filenames().first() {
                        let shared_data_c = Rc::clone(&shared_data_rc);
                        let shared_data_b = shared_data_c.borrow();
                        let mut data_to_save = Vec::<Vec<cgmath_3d::Line3<F>>>::new();
                        for s in shared_data_b.shapes.iter().flatten() {
                            for r in s.centerline.iter() {
                                for ls in r.line_strings.iter().flatten() {
                                    data_to_save.push(
                                        ls.transform(
                                            &shared_data_b.configuration.inverse_transform,
                                        )
                                        .as_lines(),
                                    );
                                }
                                for ls in r.lines.iter().flatten() {
                                    data_to_save.push(vec![ls.transform(
                                        &shared_data_b.configuration.inverse_transform,
                                    )]);
                                }
                            }
                        }
                        if let Err(err) = linestring::cgmath_3d::save_to_obj_file(
                            filename.to_str().unwrap(),
                            "centerline",
                            data_to_save,
                        ) {
                            println!("Failed to write file: {:?}", err);
                        }
                    }
                }
            }
            {
                let mut shared_data_bm = shared_data_c2.borrow_mut();
                shared_data_bm.configuration.last_message = Some(msg);
                re_calculate(shared_data_bm);
            }
            redraw();
        }
    }
    Ok(())
}

/// Re-calculate the center-line and all of the other parameters
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
            .into_par_iter() // .into_iter()
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

/// This is what a single thread does in sequence
fn threaded_re_calculate(
    mut shape: Shape,
    configuration: Configuration,
) -> Result<Shape, CenterlineError> {
    if shape.centerline.is_none() {
        shape.centerline = Some(Centerline::<i32, F>::default());
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
        if configuration
            .draw_flag
            .contains(DrawFilterFlag::REMOVE_INTERNAL_EDGES)
        {
            centerline.calculate_centerline(
                configuration.normalized_dot,
                configuration.centerline_scaled_distance,
                shape.raw_data.get_internals(),
            )?;
        } else {
            // ignore the internal
            centerline.calculate_centerline(
                configuration.normalized_dot,
                configuration.centerline_scaled_distance,
                None,
            )?;
        }
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
            Vec::<LineString3<F>>::new()
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

/// Re-calculate voronoi input geometry by running simplify
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
                let s_lines = lines.simplify((configuration.input_distance as F) * 256_f32.sqrt());
                #[cfg(feature = "console_debug")]
                println!(
                    "reduced by {} points of {} = {}",
                    before - s_lines.len(),
                    before,
                    s_lines.len()
                );
                for lineseq in s_lines.as_lines_iter() {
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
                for lineseq in lines.as_lines_iter() {
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
fn add_data_from_file(
    shared_data: Rc<RefCell<SharedData>>,
    filename: &str,
) -> Result<(), CenterlineError> {
    let mut shared_data_bm = shared_data.borrow_mut();

    let obj_set = {
        let input = BufReader::new(File::open(filename).unwrap());
        obj::raw::parse_obj(input)?
    };

    let lines = centerline::remove_internal_edges(obj_set)?;
    let lines = centerline::divide_into_shapes(lines.0, lines.1)?;

    let mut total_aabb = cgmath_3d::Aabb3::<f32>::default();
    for l in lines.iter() {
        total_aabb.update_aabb(l.get_aabb());
    }
    #[cfg(feature = "console_debug")]
    println!("total aabb b4:{:?}", total_aabb);

    let (_plane, transform, voronoi_input_aabb) =
        centerline::get_transform(&total_aabb, 256.0 * (HF.min(WF) as F - 10.0))?;
    println!("Read from file:'{}', plane was {:?}", filename, _plane);
    if let Some(inverse_transform) = transform.invert() {
        shared_data_bm.configuration.inverse_transform = inverse_transform;
    } else {
        return Err(CenterlineError::CouldNotCalculateInverseMatrix);
    }

    // transform each linestring to 2d
    let mut raw_data: Vec<LineStringSet2<F>> = lines
        .par_iter()
        .map(|x| x.transform(&transform).copy_to_2d(cgmath_3d::Plane::XY))
        .collect();
    {
        let truncate_float = |x: F| -> F { x as I as F };
        for r in raw_data.iter_mut() {
            r.operation(&truncate_float);
        }
    }

    // calculate the hull of each shape
    let raw_data: Vec<LineStringSet2<F>> = raw_data
        .into_par_iter()
        .map(|mut x| {
            x.calculate_convex_hull();
            x
        })
        .collect();

    {
        let mut screen_aabb = Aabb2::new(&Point2::new(W as F, H as F));
        screen_aabb.update_point(&Point2::new(0.0, 0.0));
        shared_data_bm.affine = SimpleAffine::new(&voronoi_input_aabb, &screen_aabb)?;
    }

    #[cfg(feature = "console_debug")]
    println!("Started with {} shapes", raw_data.len());

    let raw_data = centerline::consolidate_shapes(raw_data)?;

    #[cfg(feature = "console_debug")]
    println!("Reduced to {} shapes", raw_data.len());
    shared_data_bm.shapes = Some(
        raw_data
            .into_par_iter()
            .map(|x| Shape {
                raw_data: x,
                centerline: None,
                simplified_centerline: None,
            })
            .collect(),
    );
    shared_data_bm.configuration.input_distance_dirty = true;
    Ok(())
}

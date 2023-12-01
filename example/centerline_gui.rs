// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2021,2023 lacklustr@protonmail.com https://github.com/eadf

// This file is part of the centerline crate.

/*
Copyright (c) 2021,2023 lacklustr@protonmail.com https://github.com/eadf

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

or

Copyright 2021,2023 lacklustr@protonmail.com https://github.com/eadf

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

use centerline::Matrix4;
use centerline::{Centerline, HasMatrix4};
use centerline::{CenterlineError, LineStringSet2};

use fltk::app::redraw;
use fltk::enums::*;
use fltk::group::Pack;
use fltk::valuator::HorNiceSlider;
use fltk::{app, dialog, draw, frame::*, menu, window};

use boostvoronoi::{InputType, OutputType};
use fltk::app::MouseWheel;
use fltk::button::RoundButton;
use fltk::dialog::FileDialogType;
use fltk::menu::MenuButton;
use fltk::prelude::{GroupExt, MenuExt, ValuatorExt, WidgetBase, WidgetExt, WindowExt};
#[allow(unused_imports)]
use itertools::Itertools;
use linestring::linestring_2d::{Aabb2, Line2, LineString2, SimpleAffine};
use linestring::linestring_3d;
use linestring::linestring_3d::LineString3;
use rayon::prelude::*;
use std::cell::{RefCell, RefMut};
use std::fs::File;
use std::io::BufReader;
use std::rc::Rc;

// this requires the "obj-rs" feature to be active
use obj;
#[allow(unused_imports)]
use vector_traits::glam::{DVec3, Vec3};
use vector_traits::num_traits::{AsPrimitive, Float, FloatConst};
use vector_traits::{GenericScalar, GenericVector3, HasXY};

#[macro_use]
extern crate bitflags;

// frame size
const HF: i32 = 590;
const WF: i32 = 790;

// window size
#[allow(dead_code)]
const H: i32 = 650;
const W: i32 = 800;

#[derive(Debug, Clone, Copy)]
pub enum GuiMessage {
    SliderPreChanged(f64),
    SliderPostChanged(f64),
    SliderDotChanged(f64),
    Filter(DrawFilterFlag),
    MenuChoiceLoad,
    MenuChoiceSaveOutline,
    MenuChoiceSaveCenterLine,
}

bitflags! {
    #[derive(Debug, Clone, Copy)]
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
struct Shape<I: InputType, T: GenericVector3>
where
    T::Scalar: OutputType,
{
    // the input data after it has been transformed to our coordinate system
    // but not simplified.
    raw_data: LineStringSet2<<T as GenericVector3>::Vector2>,

    // centerline.segments is the simplified version of 'raw_data', also input to boost voronoi
    centerline: Option<Centerline<I, T>>,

    simplified_centerline: Option<Vec<Vec<T>>>,
}

#[derive(Debug, Clone)]
struct Configuration<T: GenericVector3 + HasMatrix4> {
    input_distance: T::Scalar,
    input_distance_dirty: bool,
    centerline_distance: T::Scalar,
    centerline_scaled_distance: T::Scalar,
    centerline_distance_dirty: bool,
    normalized_dot: T::Scalar,
    normalized_dot_dirty: bool,
    last_message: Option<GuiMessage>,
    draw_flag: DrawFilterFlag,
    inverse_transform: T::Matrix4Type,
}

struct SharedData<I: InputType, T: GenericVector3 + HasMatrix4>
where
    T::Scalar: OutputType,
{
    shapes: Option<Vec<Shape<I, T>>>,
    configuration: Configuration<T>,
    affine: SimpleAffine<<T as GenericVector3>::Vector2>,
}

fn main() -> Result<(), CenterlineError> {
    // define the types of execution by changing the generic parameters:
    typed_main::<i32, DVec3>() // voronoi input data is i32, voronoi output is f64
                               //typed_main::<i32, Vec3>() // voronoi input data is i32, voronoi output is f32
}

// i don't know why these types require 'static, (note that is the types, not references that need
// 'static). I think it has something to do with the gui callbacks.
fn typed_main<I: InputType + Send + 'static, T: GenericVector3 + HasMatrix4 + 'static>(
) -> Result<(), CenterlineError>
where
    T::Scalar: OutputType,
    I: AsPrimitive<T::Scalar>,
    T::Scalar: AsPrimitive<I> + AsPrimitive<i32>,
    i32: AsPrimitive<T::Scalar>,
    f32: AsPrimitive<T::Scalar>,
    f64: AsPrimitive<T::Scalar>,
{
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

    slider_pre.set_tooltip(
        "This slider defines how much the line simplification should remove from the input lines",
    );
    slider_pre.set_value(0.5);
    slider_pre.set_frame(FrameType::PlasticUpBox);
    slider_pre.set_color(Color::White);

    let mut slider_dot = HorNiceSlider::default()
        .with_size(100, 25)
        .with_label("Angle: 50.0000°");
    slider_dot.set_tooltip(
        "This slider defines the angle predicate of the edge removal process.\
         Voronoi edges that are touching the geometry will be removed if the angle between the edge \
         and the input geometry exceeds this value",
    );

    slider_dot.set_value(0.55);
    slider_dot.set_frame(FrameType::PlasticUpBox);
    slider_dot.set_color(Color::White);

    let mut slider_post = HorNiceSlider::default()
        .with_size(100, 25)
        .with_label("Centerline data simplification distance");
    slider_post.set_tooltip(
        "This slider defines how much the line simplification should remove from the output lines",
    );
    slider_post.set_value(0.5);
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
    internal_edges_button.set_tooltip(
        "If this button is enabled the inner geometry \
         (think the inside of the letter O) will be hidden.",
    );
    internal_edges_button.toggle(true);
    internal_edges_button.set_frame(FrameType::PlasticUpBox);

    pack.end();
    wind.set_color(Color::White);
    wind.end();
    wind.show();

    let shared_data_rc = Rc::new(RefCell::new(SharedData::<I, T> {
        shapes: None,
        configuration: Configuration {
            centerline_distance: 0.0.into(),
            centerline_scaled_distance: 256.0.into(),
            centerline_distance_dirty: true,
            normalized_dot: 0.38.into(),
            normalized_dot_dirty: true,
            input_distance: 0.0.into(),
            input_distance_dirty: true,
            last_message: None,
            draw_flag: DrawFilterFlag::DRAW_ALL
                ^ DrawFilterFlag::THREAD_GROUP_AABB
                ^ DrawFilterFlag::THREAD_GROUP_HULL
                ^ DrawFilterFlag::INTERNAL_GEOMETRY,
            inverse_transform: T::identity(),
        },
        affine: SimpleAffine::default(),
    }));

    let (sender, receiver) = app::channel::<GuiMessage>();
    sender.send(GuiMessage::SliderPreChanged(50.0));

    slider_pre.set_callback(move |s| {
        let value = s.value() * 100.0;
        s.set_label(&format!(
            "   Input data simplification distance: {:.4}       ",
            value
        ));
        sender.send(GuiMessage::SliderPreChanged(value));
    });

    slider_dot.set_callback(move |s| {
        let value = s.value() * 90.0;
        s.set_label(&format!("   Angle: {:.4}°      ", value));
        let value = (f64::PI() * value / 180.0).cos();
        sender.send(GuiMessage::SliderDotChanged(value));
    });

    slider_post.set_callback(move |s| {
        let value = s.value() * 100.0;
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
    wind.draw(move |_| {
        let draw_fn = |line: Result<[T::Scalar; 4], _>, cross: bool| {
            if let Ok(line) = line {
                let (x1, y1, x2, y2): (i32, i32, i32, i32) =
                    (line[0].as_(), line[1].as_(), line[2].as_(), line[3].as_());
                draw::draw_line(x1, y1, x2, y2);
                if cross {
                    // draws a little X at the end points.
                    draw::draw_line(x1 - 2, y1 - 2, x1 + 2, y1 + 2);
                    draw::draw_line(x1 + 2, y1 - 2, x1 - 2, y1 + 2);
                }
            }
        };

        let mut data_bm: RefMut<_> = shared_data_c.borrow_mut();

        draw::set_draw_color(Color::White);
        draw::draw_rectf(5, 5, WF, HF);
        draw::set_line_style(draw::LineStyle::Solid, 1);

        let opt_shapes = data_bm.shapes.take();
        if let Some(vec_shapes) = opt_shapes {
            let cross = match data_bm.configuration.last_message {
                Some(GuiMessage::SliderPreChanged(_)) => true,
                _ => false,
            };
            for shape in vec_shapes.iter() {
                if let Some(ref centerline) = shape.centerline {
                    // Draw the segments of this shape
                    draw::set_line_style(draw::LineStyle::Solid, 2);
                    draw::set_draw_color(Color::Red);
                    for a_line in centerline.segments.iter() {
                        draw_fn(
                            data_bm.affine.transform_ab_a([
                                a_line.start.x.as_(),
                                a_line.start.y.as_(),
                                a_line.end.x.as_(),
                                a_line.end.y.as_(),
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
                        draw::set_line_style(draw::LineStyle::Solid, 1);
                        draw::set_draw_color(Color::Dark1);
                        let aabb: Vec<<T as GenericVector3>::Vector2> =
                            shape.raw_data.get_aabb().clone().into();
                        for a_line in aabb.window_iter() {
                            draw_fn(
                                data_bm.affine.transform_ab_a([
                                    a_line.start.x() as T::Scalar,
                                    a_line.start.y() as T::Scalar,
                                    a_line.end.x() as T::Scalar,
                                    a_line.end.y() as T::Scalar,
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
                        draw::set_line_style(draw::LineStyle::Solid, 1);
                        draw::set_draw_color(Color::Dark2);
                        if let Some(ref hull) = shape.raw_data.get_convex_hull() {
                            for a_line in hull.window_iter() {
                                draw_fn(
                                    data_bm.affine.transform_ab_a([
                                        a_line.start.x(),
                                        a_line.start.y(),
                                        a_line.end.x(),
                                        a_line.end.y(),
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
                                for a_line in hull.1.window_iter() {
                                    draw_fn(
                                        data_bm.affine.transform_ab_a([
                                            a_line.start.x(),
                                            a_line.start.y(),
                                            a_line.end.x(),
                                            a_line.end.y(),
                                        ]),
                                        false,
                                    );
                                }
                            }
                        }
                    }

                    draw::set_line_style(draw::LineStyle::Solid, 1);
                    draw::set_draw_color(Color::Black);
                    for a_line in centerline.lines.iter().flatten() {
                        draw_fn(
                            data_bm.affine.transform_ab_a([
                                a_line.start.x(),
                                a_line.start.y(),
                                a_line.end.x(),
                                a_line.end.y(),
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
                        for a_line in a_linestring.window_iter() {
                            draw_fn(
                                data_bm.affine.transform_ab_a([
                                    a_line.start.x(),
                                    a_line.start.y(),
                                    a_line.end.x(),
                                    a_line.end.y(),
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

    wind.handle(move |_, ev| match ev {
        fltk::enums::Event::MouseWheel => {
            let event = &app::event_coords();
            //println!("mouse wheel at x:{} y:{}", event.0, event.1);

            let mut shared_data_bm = shared_data_c.borrow_mut();
            let event_dy = match app::event_dy() {
                MouseWheel::Up => 3,
                MouseWheel::Down => -3,
                _ => 0,
            };
            let reverse_middle = shared_data_bm
                .affine
                .transform_ba(T::Vector2::new_2d(event.0.as_(), event.1.as_()));
            if reverse_middle.is_err() {
                println!("{:?}", reverse_middle.err().unwrap());
                return false;
            }
            let reverse_middle = reverse_middle.unwrap();
            if event_dy != 0 {
                let scale_mod: T::Scalar = 1.01_f32.powf(event_dy as f32).as_();
                shared_data_bm.affine.scale[0] *= scale_mod;
                shared_data_bm.affine.scale[1] *= scale_mod;
            }
            let new_middle = shared_data_bm
                .affine
                .transform_ab(T::Vector2::new_2d(reverse_middle[0], reverse_middle[1]));
            if new_middle.is_err() {
                println!("{:?}", new_middle.err().unwrap());
                return false;
            }
            let new_middle = new_middle.unwrap();
            // When zooming we want the center of screen remain at the same relative position.
            shared_data_bm.affine.b_offset[0] += event.0.as_() - new_middle[0];
            shared_data_bm.affine.b_offset[1] += event.1.as_() - new_middle[1];

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
                shared_data_bm.affine.b_offset[0] += (event.0 - md.0).as_();
                shared_data_bm.affine.b_offset[1] += (event.1 - md.1).as_();
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
                    shared_data_bm.configuration.input_distance = value.as_();
                    shared_data_bm.configuration.input_distance_dirty = true;
                }
                GuiMessage::SliderDotChanged(value) => {
                    let mut shared_data_bm = shared_data_c1.borrow_mut();
                    shared_data_bm.configuration.normalized_dot = value.as_();
                    shared_data_bm.configuration.normalized_dot_dirty = true;
                }
                GuiMessage::SliderPostChanged(value) => {
                    let mut shared_data_bm = shared_data_c1.borrow_mut();
                    shared_data_bm.configuration.centerline_distance = value.as_();
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

                    let _ = chooser.set_directory(&std::path::PathBuf::from("examples"));
                    let _ = chooser.set_title("select your input data");
                    chooser.set_filter("*.obj");
                    chooser.show();
                    if let Some(filename) = chooser.filenames().first() {
                        let shared_data_c = Rc::clone(&shared_data_rc);
                        if let Err(err) =
                            add_data_from_file(shared_data_c, filename.to_str().unwrap())
                        {
                            println!("Failed to read file {:?}: {:?}", filename, err);
                        }
                        if let Some(filename) = filename.to_str() {
                            let w = &mut wind;
                            w.set_label(filename);
                        }
                    }
                }
                GuiMessage::MenuChoiceSaveOutline => {
                    let mut chooser =
                        dialog::NativeFileChooser::new(FileDialogType::BrowseSaveFile);

                    let _ = chooser.set_directory(&std::path::PathBuf::from("examples"));
                    let _ = chooser.set_title("select file to save outline to");
                    chooser.set_filter("*.obj");
                    chooser.show();
                    if let Some(filename) = chooser.filenames().first() {
                        let shared_data_c = Rc::clone(&shared_data_rc);
                        let shared_data_b = shared_data_c.borrow();
                        let mut data_to_save = Vec::<Vec<linestring_3d::Line3<T>>>::new();
                        for s in shared_data_b.shapes.iter().flatten() {
                            if let Some(ref centerline) = s.centerline {
                                data_to_save.push(
                                    centerline
                                        .segments
                                        .iter()
                                        .map(|l| {
                                            Line2::<<T as GenericVector3>::Vector2>::from([
                                                l.start.x.as_(),
                                                l.start.y.as_(),
                                                l.end.x.as_(),
                                                l.end.y.as_(),
                                            ])
                                            .copy_to_3d(linestring_3d::Plane::XY)
                                            .apply(
                                                |x: T| -> T {
                                                    let rv: T = shared_data_b
                                                        .configuration
                                                        .inverse_transform
                                                        .transform_point3(x);
                                                    rv
                                                },
                                            )
                                        })
                                        .collect(),
                                );
                            }
                        }
                        if let Err(err) = linestring::linestring_3d::save_to_obj_file(
                            filename.to_str().unwrap(),
                            "outline",
                            &data_to_save,
                        ) {
                            println!("Failed to write file: {:?}", err);
                        }
                    }
                }
                GuiMessage::MenuChoiceSaveCenterLine => {
                    let mut chooser =
                        dialog::NativeFileChooser::new(FileDialogType::BrowseSaveFile);

                    let _ = chooser.set_directory(&std::path::PathBuf::from("examples"));
                    let _ = chooser.set_title("select file to save outline to");
                    chooser.set_filter("*.obj");
                    chooser.show();
                    if let Some(filename) = chooser.filenames().first() {
                        let shared_data_c = Rc::clone(&shared_data_rc);
                        let shared_data_b = shared_data_c.borrow();
                        let mut data_to_save = Vec::<Vec<linestring_3d::Line3<T>>>::new();
                        for s in shared_data_b.shapes.iter().flatten() {
                            for r in s.centerline.iter() {
                                for ls in r.line_strings.iter().flatten() {
                                    data_to_save.push({
                                        let mut lsc = ls.clone();
                                        lsc.apply(&|x| {
                                            shared_data_b
                                                .configuration
                                                .inverse_transform
                                                .transform_point3(x)
                                        });
                                        lsc.window_iter().collect()
                                    });
                                }
                                for ls in r.lines.iter().flatten() {
                                    data_to_save.push(vec![ls.apply(|x| {
                                        shared_data_b
                                            .configuration
                                            .inverse_transform
                                            .transform_point3(x)
                                    })]);
                                }
                            }
                        }
                        if let Err(err) = linestring::linestring_3d::save_to_obj_file(
                            filename.to_str().unwrap(),
                            "centerline",
                            &data_to_save,
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
fn re_calculate<I: InputType + Send, T: GenericVector3>(
    mut shared_data_bm: RefMut<SharedData<I, T>>,
) where
    T: HasMatrix4,
    T::Scalar: OutputType,
    I: AsPrimitive<T::Scalar>,
    T::Scalar: AsPrimitive<I>,
{
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
            .filter_map(|x| threaded_re_calculate_error_handler(x, &configuration))
            .collect();
        shared_data_bm.shapes = Some(shapes);
    }
    shared_data_bm.configuration.normalized_dot_dirty = false;
    shared_data_bm.configuration.centerline_distance_dirty = false;
    shared_data_bm.configuration.input_distance_dirty = false;
    redraw();
}

fn threaded_re_calculate_error_handler<I: InputType, T: GenericVector3>(
    shape: Shape<I, T>,
    configuration: &Configuration<T>,
) -> Option<Shape<I, T>>
where
    T: HasMatrix4,
    T::Scalar: OutputType,
    I: AsPrimitive<T::Scalar>,
    T::Scalar: AsPrimitive<I>,
{
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
fn threaded_re_calculate<I: InputType, T: GenericVector3>(
    mut shape: Shape<I, T>,
    configuration: &Configuration<T>,
) -> Result<Shape<I, T>, CenterlineError>
where
    T: HasMatrix4,
    T::Scalar: OutputType,
    I: AsPrimitive<T::Scalar>,
    T::Scalar: AsPrimitive<I>,
{
    if shape.centerline.is_none() {
        shape.centerline = Some(Centerline::<I, T>::default());
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
    if shape.simplified_centerline.is_none()
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
fn recalculate_voronoi_diagram<I: InputType, T: GenericVector3>(
    shape: &mut Shape<I, T>,
    configuration: &Configuration<T>,
) -> Result<(), CenterlineError>
where
    T: HasMatrix4,
    T::Scalar: OutputType,
    I: AsPrimitive<T::Scalar>,
{
    #[cfg(feature = "console_debug")]
    println!("recalculate_voronoi_diagram()");
    if let Some(mut centerline_rw) = shape.centerline.take() {
        centerline_rw.build_voronoi()?;
        shape.centerline = Some(centerline_rw);
    }
    Ok(())
}

/// Re-calculate centerline
fn recalculate_centerline<I: InputType, T: GenericVector3>(
    shape: &mut Shape<I, T>,
    configuration: &Configuration<T>,
) -> Result<(), CenterlineError>
where
    T: HasMatrix4,
    T::Scalar: OutputType,
    I: AsPrimitive<T::Scalar>,
{
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
fn simplify_centerline<I: InputType, T: GenericVector3>(
    shape: &mut Shape<I, T>,
    configuration: &Configuration<T>,
) -> Result<(), CenterlineError>
where
    T: HasMatrix4,
    T::Scalar: OutputType,
{
    #[cfg(feature = "console_debug")]
    println!("simplify_centerline()");
    let mut simplified_centerline =
        if let Some(simplified_centerline) = shape.simplified_centerline.take() {
            simplified_centerline
        } else {
            Vec::<Vec<T>>::new()
        };
    simplified_centerline.clear();
    if let Some(ref centerline) = shape.centerline {
        if let Some(ref line_strings) = centerline.line_strings {
            for ls in line_strings.iter() {
                simplified_centerline.push(
                    ls.simplify_rdp(configuration.centerline_distance * 256_f32.sqrt().into()),
                );
            }
        }
    }
    shape.simplified_centerline = Some(simplified_centerline);
    Ok(())
}

/// Re-calculate voronoi input geometry by running simplify
/// Todo: self intersection test -> fail
fn recalculate_voronoi_input<I: InputType + 'static, T: GenericVector3>(
    shape: &mut Shape<I, T>,
    configuration: &Configuration<T>,
) -> Result<(), CenterlineError>
where
    T: HasMatrix4,
    T::Scalar: OutputType,
    T::Scalar: AsPrimitive<I>,
{
    #[cfg(feature = "console_debug")]
    println!("recalculate_voronoi_input()");
    if let Some(mut centerline) = shape.centerline.take() {
        centerline.segments.clear();
        for lines in shape.raw_data.set().iter() {
            //let mut set = Vec::<boostvoronoi::Line<i64>>::with_capacity(lines.len());
            if configuration.input_distance > T::Scalar::ZERO {
                #[cfg(feature = "console_debug")]
                let before = lines.0.len();
                let dist_p: T::Scalar = (configuration.input_distance) * 256_f32.sqrt().into();
                let s_lines = lines.simplify_rdp(dist_p);
                #[cfg(feature = "console_debug")]
                println!(
                    "reduced by {} points of {} = {}",
                    before - s_lines.0.len(),
                    before,
                    s_lines.0.len()
                );
                for lineseq in s_lines.window_iter() {
                    centerline.segments.push(boostvoronoi::Line::new(
                        boostvoronoi::Point {
                            x: lineseq.start.x().as_(),
                            y: lineseq.start.y().as_(),
                        },
                        boostvoronoi::Point {
                            x: lineseq.end.x().as_(),
                            y: lineseq.end.y().as_(),
                        },
                    ));
                }
            } else {
                #[cfg(feature = "console_debug")]
                println!("no reduction");
                for lineseq in lines.window_iter() {
                    centerline.segments.push(boostvoronoi::Line::new(
                        boostvoronoi::Point {
                            x: lineseq.start.x().as_(),
                            y: lineseq.start.y().as_(),
                        },
                        boostvoronoi::Point {
                            x: lineseq.end.x().as_(),
                            y: lineseq.end.y().as_(),
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

#[cfg(feature = "obj-rs")]
/// Add data to the input lines.
fn add_data_from_file<I: InputType + Send, T: GenericVector3>(
    shared_data: Rc<RefCell<SharedData<I, T>>>,
    filename: &str,
) -> Result<(), CenterlineError>
where
    T: HasMatrix4,
    T::Scalar: OutputType,
    f32: AsPrimitive<<T as HasXY>::Scalar>,
    i32: AsPrimitive<<T as HasXY>::Scalar>,
{
    let mut shared_data_bm = shared_data.borrow_mut();

    let obj_set = {
        let input = BufReader::new(File::open(filename).unwrap());
        obj::raw::parse_obj(input).or_else(|x| Err(CenterlineError::ObjError(x.to_string())))?
    };

    let lines = centerline::remove_internal_edges(obj_set)?;
    let lines = centerline::divide_into_shapes(lines.0, lines.1)?;
    let mut total_aabb = linestring_3d::Aabb3::<T>::default();
    for l in lines.iter() {
        total_aabb.update_aabb(l.get_aabb());
    }
    #[cfg(feature = "console_debug")]
    println!("total aabb b4:{:?}", total_aabb);

    let (_plane, transform, voronoi_input_aabb) = {
        let dimension: T::Scalar = (256.0 * (HF.min(WF) as f32 - 10.0)).as_();
        centerline::get_transform::<T>(total_aabb, dimension)?
    };
    println!("Read from file:'{}', plane was {:?}", filename, _plane);
    if let Some(inverse_transform) = transform.safe_inverse() {
        shared_data_bm.configuration.inverse_transform = inverse_transform;
    } else {
        return Err(CenterlineError::CouldNotCalculateInverseMatrix(
            "".to_string(),
        ));
    }

    // transform each linestring to 2d
    let mut raw_data: Vec<LineStringSet2<<T as GenericVector3>::Vector2>> = lines
        .par_iter()
        .map(|x| {
            let mut xc = x.clone();
            xc.apply(&|v| transform.transform_point3(v));
            xc.copy_to_2d(linestring_3d::Plane::XY)
        })
        .collect();
    {
        let truncate_float = |v: <T as GenericVector3>::Vector2| -> <T as GenericVector3>::Vector2 {
            <T as GenericVector3>::Vector2::new_2d(v.x().round(), v.y().round())
        };
        for r in raw_data.iter_mut() {
            r.apply(&truncate_float);
        }
    }

    // calculate the hull of each shape
    let raw_data: Vec<LineStringSet2<<T as GenericVector3>::Vector2>> = raw_data
        .into_par_iter()
        .map(|mut x| {
            x.calculate_convex_hull().unwrap();
            x
        })
        .collect();

    {
        let mut screen_aabb = Aabb2::new(<T as GenericVector3>::Vector2::new_2d(W.as_(), H.as_()));
        screen_aabb.update_with_point(<T as GenericVector3>::Vector2::new_2d(
            0.0.into(),
            0.0.into(),
        ));
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

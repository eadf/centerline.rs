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
use fltk::*;
use fnv;
use geo::Coordinate;
use intersect2d::algorithm::AlgorithmData;
use intersect2d::scale_to_coordinate;
use itertools::Itertools;
use linestring::cgmath_impl;
use linestring::cgmath_impl::LineStringSet3;
use num_traits::{Float, ToPrimitive};
use obj;
use ordered_float::OrderedFloat;
use std::cell::RefCell;
use std::cmp::max;
use std::fmt::Error;
use std::fs::File;
use std::io::prelude::*;
use std::io::BufReader;
use std::path::Path;
use std::rc::Rc;

const DRAW_TEXT: bool = true;
const WINDOW_SIZE: i32 = 800;

struct SharedData {
    lines: Vec<geo::Line<f64>>,
    window_center: (i32, i32),
}

fn main() -> Result<(), CenterlineError> {
    let app = app::App::default();
    let mut wind = window::Window::default()
        .with_size(WINDOW_SIZE, WINDOW_SIZE)
        .center_screen()
        .with_label("Centerline finder");

    wind.set_color(Color::Black);
    wind.end();
    wind.show();
    let shared_data_rc = Rc::from(RefCell::from(SharedData {
        lines: Vec::default(),
        window_center: (WINDOW_SIZE / 2, WINDOW_SIZE / 2),
    }));

    let alg_data_c = shared_data_rc.clone();
    add_data(alg_data_c)?;

    let alg_data_c = shared_data_rc.clone();
    // This is called whenever the window is drawn and redrawn (in the event loop)
    wind.draw(move || {
        let alg_data_b = alg_data_c.borrow();

        draw::set_draw_color(Color::White);
        for (line_index, line) in alg_data_b.lines.iter().enumerate() {
            // The scaling transform is already multiplied by 1024
            draw::draw_line(
                (line.start.x as i32 >> 10) + alg_data_b.window_center.0,
                (line.start.y as i32 >> 10) + alg_data_b.window_center.1,
                (line.end.x as i32 >> 10) + alg_data_b.window_center.0,
                (line.end.y as i32 >> 10) + alg_data_b.window_center.1,
            );
        }
    });
    let shared_data_c = Rc::clone(&shared_data_rc);
    wind.handle(move |ev| match ev {
        enums::Event::KeyUp => {
            let app_event_txt = &app::event_text() as &str;

            if app_event_txt == " " || app_event_txt == "c" {
                let alg_data_c = Rc::clone(&shared_data_c);
                let mut data = alg_data_c.borrow_mut();
                if app_event_txt == " " {
                    //data.compute(true)
                } else {
                    // app_event_txt == "c"{
                    //data.compute(false)
                }
            };
            true
        }
        enums::Event::Released => {
            let event = &app::event_coords();
            println!("mouse at {:?}", event);
            true
        }
        _ => false,
    });

    while app.wait() {
        wind.redraw();
        if !cfg!(windows) {
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
    Ok(())
}

/// Convert an array slice into a vec of geo::Line
/// Todo use the function of intersection2d that does just this
#[allow(dead_code)]
fn to_lines<U, T>(points: &[[U; 4]]) -> Vec<geo::Line<T>>
where
    U: ToPrimitive + Copy,
    T: Float + approx::UlpsEq + geo::CoordNum + PartialOrd,
    T::Epsilon: Copy,
{
    let mut rv = Vec::with_capacity(points.len());
    for p in points.iter() {
        rv.push(geo::Line::<T>::new(
            geo::Coordinate {
                x: T::from(p[0]).unwrap(),
                y: T::from(p[1]).unwrap(),
            },
            geo::Coordinate {
                x: T::from(p[2]).unwrap(),
                y: T::from(p[3]).unwrap(),
            },
        ));
    }
    rv
}

#[allow(dead_code)]
fn print_results(data: Rc<RefCell<AlgorithmData<f64>>>) {
    let data = data.borrow_mut();

    if let Some(results) = data.get_results() {
        let intersections = results.iter().map(|x| x.1.iter()).flatten().count() / 2;

        for r in results.iter() {
            print!("Intersection @({:.4},{:.4}) lines:", r.0.pos.x, r.0.pos.y);
            for l in r.1.iter() {
                print!("{},", l);
            }
            println!();
        }
        println!("In total {} unique intersection points.", intersections);

        let size = data.get_lines().len();

        println!(
            "Made {} calls to intersect() for {} line segments. Brute force approach would need at least {} calls",
            data.get_intersection_calls(),
            size, size*size/2
        );
    }
}

/// Add data to the input lines.
fn add_data(data: Rc<RefCell<SharedData>>) -> Result<(), CenterlineError> {
    let mut data_b = data.borrow_mut();

    let obj_set = {
        let input = BufReader::new(File::open("example/rust.obj").unwrap());
        obj::raw::parse_obj(input)?
    };

    let lines = centerline::remove_internal_edges(obj_set)?;
    let mut total_aabb = cgmath_impl::Aabb3::default();
    for l in lines.iter() {
        total_aabb.update_aabb(l.get_aabb());
    }
    println!("total aabb b4:{:?}", total_aabb);

    let transform = get_transform(&total_aabb)?;
    let lines: Vec<LineStringSet3<f32>> = lines.iter().map(|x| x.transform(&transform)).collect();
    //let mut total_aabb = cgmath_impl::Aabb3::default();
    //for l in lines.iter() {
    //    total_aabb.update_aabb(l.get_aabb());
    //}
    //println!("total aabb after:{:?}", total_aabb);

    data_b.lines.clear();
    for ls in lines.iter() {
        for l in ls.set.iter() {
            for lineseq in l.as_lines().iter() {
                data_b.lines.push(geo::Line::<f64>::new(
                    Coordinate {
                        x: lineseq.start.x as f64,
                        y: lineseq.start.y as f64,
                    },
                    Coordinate {
                        x: lineseq.end.x as f64,
                        y: lineseq.end.y as f64,
                    },
                ))
            }
        }
    }

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

fn get_transform(
    total_aabb: &cgmath_impl::Aabb3<f32>,
) -> Result<cgmath::Matrix4<f32>, CenterlineError> {
    let plane = if let Some(plane) = cgmath_impl::Plane::get_plane(total_aabb) {
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
            cgmath_impl::Plane::XY => cgmath::Matrix4::from_cols(x, y, z, w),
            cgmath_impl::Plane::XZ => cgmath::Matrix4::from_cols(x, z, y, w),
            cgmath_impl::Plane::ZY => cgmath::Matrix4::from_cols(z, y, x, w),
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

    Ok(total_transform)
}

use boostvoronoi::voronoi_builder as VB;
use boostvoronoi::InputType;
use geo::{Coordinate, Line};
#[allow(unused_imports)]
use intersect2d;
use std::ops::Neg;

type I1 = i32;
type F1 = f64;
type I2 = i64;
type F2 = f64;

#[allow(dead_code)]
fn to_segments<T>(points: &[[T; 4]]) -> Vec<Line<T>>
where
    T: InputType + Neg<Output = T>,
{
    let mut rv = Vec::with_capacity(points.len());
    for p in points.iter() {
        rv.push(Line::<T>::new(
            Coordinate::<T> { x: p[0], y: p[1] },
            Coordinate::<T> { x: p[2], y: p[3] },
        ));
    }
    rv
}

fn main() {
    #[allow(unused_variables)]
    let output = {
        let segments: [[I1; 4]; 5] = [
            [200, 200, 200, 400],
            [200, 400, 400, 400],
            [400, 400, 400, 200],
            [400, 200, 200, 200],
            [529, 242, 367, 107],
        ];

        let _s = to_segments::<I1>(&segments);

        let mut vb = VB::VoronoiBuilder::<I1, F1, I2, F2>::new();
        vb.with_segments(_s.iter()).expect("test_template");
        vb.construct().expect("test_template")
    };
    println!("done");
}

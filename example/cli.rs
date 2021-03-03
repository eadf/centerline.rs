use boostvoronoi::BvError;
use boostvoronoi::Line;
use boostvoronoi::Point;
use centerline::{Centerline, CenterlineError};

fn main() -> Result<(), CenterlineError> {
    let segments: Vec<Line<i32>> = [
        [0, 0, 100, 0].into(),
        [100, 0, 50, 50].into(),
        [50, 50, 10, 12].into(),
        [10, 12, 0,0].into(),
    ]
    .into();
    let mut centerline = Centerline::<i32, f32, i64, f64>::with_segments(segments);
    centerline.build_voronoi()?;
    println!(
        "Result: cells:{}, edges:{}, vertices:{}",
        centerline.diagram()?.cells().len(),
        centerline.diagram()?.edges().len(),
        centerline.diagram()?.vertices().len()
    );
    centerline.calculate_centerline()?;
    println!(
        "Result: lines:{}, arcs:{}, linsestrings:{}",
        centerline.lines.len(),
        centerline.arcs.len(),
        centerline.linestrings.len()
    );

    Ok(())
}

use boostvoronoi::Line;
use centerline::{Centerline, CenterlineError};

fn main() -> Result<(), CenterlineError> {
    let segments: Vec<Line<i32>> = [
        [0, 0, 100, 0].into(),
        [100, 0, 100, 100].into(),
        [100, 100, 0, 100].into(),
        [0, 100, 0, 0].into(),
        [40, 50, 60, 50].into(),
    ]
    .into();
    let mut centerline = Centerline::<i32, f32, i64, f64>::with_segments(segments);
    centerline.build_voronoi()?;
    println!(
        "Result: cells:{}, edges:{}, vertices:{}",
        centerline.diagram().cells().len(),
        centerline.diagram().edges().len(),
        centerline.diagram().vertices().len()
    );

    let rejected_edges = centerline.rejected_edges().unwrap();
    centerline.diagram().debug_print_all(|x:usize|!rejected_edges.bit(x));

    for (eid, e) in centerline
        .diagram()
        .edges()
        .iter()
        .enumerate()
        .filter(|(i, _)| !rejected_edges.bit(*i))
    {
        let twin = centerline.diagram().edge_get_twin(Some(e.get().get_id()));
        let twin_i:Vec<bool> = twin.iter().map(|x| rejected_edges.bit(x.0)).collect();
        print!("edge#{} {:?} {} {:?} {:?}", eid, e, rejected_edges.bit(eid), twin, twin_i.first());
        let eid = Some(e.get().get_id());
        let v0 = centerline.diagram().edge_get_vertex0(eid).unwrap();
        let v0 = centerline.diagram().vertex_get(Some(v0)).unwrap().get();

        let v1 = centerline.diagram().edge_get_vertex1(eid).unwrap();
        let v1 = centerline.diagram().vertex_get(Some(v1)).unwrap().get();

        let is_secondary = e.get().is_secondary();
        let is_curved = e.get().is_curved();

        println!(" ({:5.3},{:.3})-({:.3},{:.3}) v1:{} secondary:{} curved:{}", v0.x(), v0.y(), v1.x(), v1.y(), v1.get_id().0, is_secondary, is_curved);
    }

    centerline.calculate_centerline(0.0001, 0.1)?;
    println!(
        "Result: lines:{}, linestrings:{}",
        centerline.lines.iter().flatten().count(),
        centerline.line_strings.iter().flatten().count()
    );

    println!("Output");
    println!("");
    println!("linestrings:");
    for ls in centerline.line_strings.iter().flatten() {
        print!("linestring:");
        for point in ls.points().iter() {
            print!("[{},{},{}],", point.x, point.y, point.z);
        }
        println!("");
    }
    println!();
    println!("lines:");
    for line in centerline.lines.iter().flatten() {
        println!(
            "line:[{},{},{}]-[{},{},{}],",
            line.start.x, line.start.y, line.start.z, line.end.x, line.end.y, line.end.z,
        );
    }
    Ok(())
}

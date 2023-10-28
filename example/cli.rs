use centerline::{Centerline, CenterlineError, GrowingVob};
use vector_traits::glam::Vec3;

fn main() -> Result<(), CenterlineError> {
    let _test_segments: [[i32; 4]; 12] = [
        [-39092, 94519, -91873, 73333],
        [-91873, 73333, -119937, -42834],
        [-119937, -42834, -155353, -59623],
        [-155353, -59623, -250514, -39563],
        [-250514, -39563, -296960, -94300],
        [-296960, -94300, 144469, -94698],
        [144469, -94698, 289762, -122601],
        [289762, -122601, 296960, -115045],
        [296960, -115045, 210691, 117441],
        [210691, 117441, 113416, 122601],
        [113416, 122601, 73916, 119690],
        [73916, 119690, -39092, 94519],
    ];
    let segments = boostvoronoi::to_segments_offset::<i32, i32>(
        &_test_segments,
        1.0 / 1024.0,
        1.0 / 1024.0,
        350,
        350,
    );
    let mut centerline = Centerline::<i32, Vec3>::with_segments(segments);
    centerline.build_voronoi()?;
    println!(
        "Result: cells:{}, edges:{}, vertices:{}",
        centerline.diagram().cells().len(),
        centerline.diagram().edges().len(),
        centerline.diagram().vertices().len()
    );

    let rejected_edges = centerline.rejected_edges().unwrap();
    for (eid, e) in centerline
        .diagram()
        .edges()
        .iter()
        .enumerate()
        .filter(|(i, _)| !rejected_edges.get_f(*i))
    {
        let twin = centerline.diagram().edge_get(e.id())?.twin();
        let twin_i: Vec<bool> = twin.iter().map(|x| rejected_edges.get_f(x.0)).collect();
        print!(
            "edge#{} {:?} {} {:?} {:?}",
            eid,
            e,
            rejected_edges.get_f(eid),
            twin,
            twin_i.first()
        );
        // all edges without v1 and/or v0 are already filtered out

        let eid = e.id();
        let v0 = centerline.diagram().edge_get_vertex0(eid)?.unwrap();
        let v0 = centerline.diagram().vertex_get(v0)?;

        let v1 = centerline.diagram().edge_get_vertex1(eid)?.unwrap();
        let v1 = centerline.diagram().vertex_get(v1)?;

        let is_secondary = e.is_secondary();
        let is_curved = e.is_curved();

        println!(
            " ({:5.3},{:.3})-({:.3},{:.3}) v1:{} secondary:{} curved:{}",
            v0.x(),
            v0.y(),
            v1.x(),
            v1.y(),
            v1.get_id().0,
            is_secondary,
            is_curved
        );
    }

    centerline.calculate_centerline(0.0001, 0.1, None)?;
    println!(
        "Result: lines:{}, linestrings:{}",
        centerline.lines.iter().flatten().count(),
        centerline.line_strings.iter().flatten().count()
    );

    println!("Output");
    println!();
    println!("linestrings:");
    for ls in centerline.line_strings.iter().flatten() {
        print!("linestring:");
        for point in ls.points().iter() {
            print!("[{},{},{}],", point.x, point.y, point.z);
        }
        println!();
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

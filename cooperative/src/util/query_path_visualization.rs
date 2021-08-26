use rust_road_router::datastr::graph::NodeId;

pub fn print_path_coords(path: &[NodeId], lat: &[f32], lon: &[f32]) {
    print!("[");
    path.iter().cloned().for_each(|node| {
        let node = node as usize;
        print!("[{},{}],", lat[node], lon[node])
    });
    println!("],");
}
use rust_road_router::datastr::graph::NodeId;

pub fn generate_visualization_data(path: &Vec<NodeId>, lat: &Vec<f32>, lon: &Vec<f32>) {
    print!("[");
    path
        .iter()
        .cloned()
        .for_each(|node| {
            let node = node as usize;
            print!("[{},{}],", lat[node], lon[node])
        });
    println!("],");
}

/*fn format_coord(coord: i32) -> f64 {
    let mut s_coord: String;

    if coord >= 0 {
        s_coord = format!("{:07}", coord);
        s_coord.insert(2, '.');
    } else {
        s_coord = format!("{:08}", coord);
        s_coord.insert(3, '.');
    }

    s_coord.parse().unwrap()
}*/
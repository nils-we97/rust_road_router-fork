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
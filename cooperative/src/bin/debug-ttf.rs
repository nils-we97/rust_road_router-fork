use cooperative::dijkstra::potentials::corridor_lowerbound_potential::customization::CustomizedCorridorLowerbound;
use cooperative::graph::traffic_functions::BPRTrafficFunction;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_node_order::load_node_order;
use cooperative::util::cli_args::parse_arg_required;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::io::Load;
use std::env;
use std::error::Error;
use std::path::Path;

/// Parameter: <path_to_graph_and_ttf_directory>
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let path = Path::new(&directory);

    // load data
    let graph = load_capacity_graph(&path, 200, BPRTrafficFunction::default())?;
    let order = load_node_order(&path)?;
    let cch = CCH::fix_order_and_build(&graph, order);

    let debug_prefix_sum = Vec::<u32>::load_from(&path.join("debug").join("prefix_sum"))?;
    let debug_departure = Vec::<u32>::load_from(&path.join("debug").join("departure"))?;
    let debug_travel_time = Vec::<u32>::load_from(&path.join("debug").join("travel_time"))?;

    let mut departure = Vec::with_capacity(debug_prefix_sum.len() - 1);
    let mut travel_time = Vec::with_capacity(debug_prefix_sum.len() - 1);

    for a in debug_prefix_sum.windows(2) {
        departure.push(debug_departure[a[0] as usize..a[1] as usize].to_vec());
        travel_time.push(debug_travel_time[a[0] as usize..a[1] as usize].to_vec());
    }

    let _customized = CustomizedCorridorLowerbound::test_customization(&cch, &graph, 72, &departure, &travel_time);

    Ok(())
}

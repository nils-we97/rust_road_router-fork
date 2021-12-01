use cooperative::dijkstra::potentials::corridor_lowerbound_potential::customization::CustomizedApproximatedPeriodicTTF;
use cooperative::dijkstra::potentials::corridor_lowerbound_potential::customization_catchup::convert_to_td_graph;
use cooperative::io::io_interval_minima_customization::store_interval_minima;
use cooperative::util::cli_args::parse_arg_required;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::datastr::graph::time_dependent::TDGraph;
use rust_road_router::datastr::node_order::NodeOrder;
use rust_road_router::io::{Load, Reconstruct};
use rust_road_router::report::measure;
use std::env;
use std::error::Error;
use std::path::Path;

/// Pre-customize a given PTV graph. As the weights are not updated after each query,
/// we can save a significant amount of time by running the customization step only once.
///
/// Parameters: <path_to_graph> <num_intervals> <output_directory>
fn main() -> Result<(), Box<dyn Error>> {
    let (path, num_intervals, output_directory) = parse_required_args()?;
    let graph_directory = Path::new(&path);

    // load graph
    let (graph, time) = measure(|| TDGraph::reconstruct_from(&graph_directory).unwrap());
    println!("Loaded graph in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    // init cch
    let order = Vec::load_from(graph_directory.join("cch_perm")).map(NodeOrder::from_node_order)?;
    let cch = CCH::fix_order_and_build(&graph, order);

    let graph = convert_to_td_graph(&graph);

    // run customization
    let (customized, time) = measure(|| CustomizedApproximatedPeriodicTTF::new_from_ptv(&cch, &graph, num_intervals));
    println!("Complete customization took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    let customized_directory = graph_directory.join("customized");
    if !customized_directory.exists() {
        std::fs::create_dir(&customized_directory);
    }
    let output_directory = customized_directory.join(output_directory);
    if !output_directory.exists() {
        std::fs::create_dir(&output_directory);
    }

    store_interval_minima(&output_directory, &customized);
    println!("Stored customized struct in {}", output_directory.display());

    Ok(())
}

fn parse_required_args() -> Result<(String, u32, String), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let num_intervals: u32 = parse_arg_required(&mut args, "number of intervals")?;
    let output_directory: String = parse_arg_required(&mut args, "query type")?;

    Ok((graph_directory, num_intervals, output_directory))
}

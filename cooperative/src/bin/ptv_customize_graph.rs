use cooperative::dijkstra::potentials::corridor_lowerbound_potential::customization::CustomizedApproximatedPeriodicTTF;
use cooperative::dijkstra::potentials::corridor_lowerbound_potential::customization_catchup::convert_to_td_graph;
use cooperative::dijkstra::potentials::multi_metric_potential::customization::CustomizedMultiMetrics;
use cooperative::dijkstra::potentials::multi_metric_potential::interval_patterns::balanced_interval_pattern;
use cooperative::experiments::types::PotentialType;
use cooperative::graph::MAX_BUCKETS;
use cooperative::io::io_ptv_customization::{store_interval_minima, store_multiple_metrics};
use cooperative::util::cli_args::{parse_arg_optional, parse_arg_required};
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::datastr::graph::time_dependent::{TDGraph, Timestamp};
use rust_road_router::datastr::graph::{EdgeId, Weight};
use rust_road_router::datastr::node_order::NodeOrder;
use rust_road_router::io::{Load, Reconstruct};
use rust_road_router::report::measure;
use std::env;
use std::error::Error;
use std::path::Path;

/// Pre-customize a given PTV graph. As the weights are not updated after each query,
/// we can save a significant amount of time by running the customization step only once.
///
/// Parameters: <path_to_graph> <potential_type = MULTI_METRIC/CORRIDOR_LOWERBOUND> <output_directory>
/// Additional parameters, depending on `potential_type`:
/// CORRIDOR_LOWERBOUND: <num_intervals = 72>
/// MULTI_METRICS: <max_num_metrics = 20>
fn main() -> Result<(), Box<dyn Error>> {
    let (path, potential_type, output_directory, mut remaining_args) = parse_required_args()?;
    let graph_directory = Path::new(&path);

    // create output directory
    let customized_directory = graph_directory.join("customized");
    if !customized_directory.exists() {
        std::fs::create_dir(&customized_directory)?;
    }
    let output_directory = customized_directory.join(output_directory);
    if !output_directory.exists() {
        std::fs::create_dir(&output_directory)?;
    }

    // load graph
    let (graph, time) = measure(|| TDGraph::reconstruct_from(&graph_directory).unwrap());
    println!("Loaded graph in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    // init cch
    let order = Vec::load_from(graph_directory.join("cch_perm")).map(NodeOrder::from_node_order)?;
    let cch = CCH::fix_order_and_build(&graph, order);

    match potential_type {
        PotentialType::CorridorLowerbound => {
            let num_intervals = parse_arg_optional(&mut remaining_args, 72);

            let graph = convert_to_td_graph(&graph);
            let (customized, time) = measure(|| CustomizedApproximatedPeriodicTTF::new_from_ptv(&cch, &graph, num_intervals));
            println!("Complete customization took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

            println!("Started storing results...");
            store_interval_minima(&output_directory, &customized)?;
            println!("Stored customized struct in {}", output_directory.display());
        }
        PotentialType::MultiMetrics => {
            let num_metrics = parse_arg_optional(&mut remaining_args, 20);

            let (departure, travel_time) = retrieve_departure_and_travel_time(&graph);
            let (customized_multi_metric, time) =
                measure(|| CustomizedMultiMetrics::new(&cch, &departure, &travel_time, &balanced_interval_pattern(), num_metrics));
            println!("Complete customization took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

            println!("Started storing results...");
            store_multiple_metrics(&output_directory, &customized_multi_metric)?;
            println!("Stored customized struct in {}", output_directory.display());
        }
        PotentialType::MultiLevelBucket => unimplemented!(),
    }

    // run customization

    /*let upward_weights = Vec::<u32>::load_from(&output_directory.join("upward_intervals"))?;
    upward_weights.iter().zip(customized.upward_intervals.iter()).for_each(|(&a, &b)| {
        if a != b {
            println!("expected {}, loaded {}", a, b);
        }
    });
    drop(upward_weights);

    let downward_weights = Vec::<u32>::load_from(&output_directory.join("downward_intervals"))?;
    downward_weights.iter().zip(customized.downward_intervals.iter()).for_each(|(&a, &b)| {
        if a != b {
            println!("expected {}, loaded {}", a, b);
        }
    });
    drop(downward_weights);*/

    Ok(())
}

fn parse_required_args() -> Result<(String, PotentialType, String, impl Iterator<Item = String>), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let potential_type: PotentialType = parse_arg_required(&mut args, "Potential Type")?;
    let output_directory: String = parse_arg_required(&mut args, "query type")?;

    Ok((graph_directory, potential_type, output_directory, args))
}

fn retrieve_departure_and_travel_time(graph: &TDGraph) -> (Vec<Vec<Timestamp>>, Vec<Vec<Weight>>) {
    (0..graph.head().len())
        .into_iter()
        .map(|edge_id| {
            let plf = graph.travel_time_function(edge_id as EdgeId);

            let mut departures = plf.departure().to_vec();
            let mut travel_times = plf.travel_time().to_vec();

            if departures.is_empty() {
                departures = vec![0, MAX_BUCKETS];
                travel_times = vec![0, 0];
            } else if departures.last().unwrap_or(&0) != &MAX_BUCKETS {
                departures.push(MAX_BUCKETS);
                travel_times.push(travel_times.first().unwrap().clone());
            }

            (departures, travel_times)
        })
        .unzip()
}

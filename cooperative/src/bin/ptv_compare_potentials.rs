use cooperative::dijkstra::potentials::corridor_lowerbound_potential::CorridorLowerboundPotential;
use cooperative::dijkstra::potentials::multi_level_bucket_potential::customization::CustomizedMultiLevels;
use cooperative::dijkstra::potentials::multi_level_bucket_potential::CCHMultiLevelBucketPotential;
use cooperative::dijkstra::potentials::multi_metric_potential::customization::CustomizedMultiMetrics;
use cooperative::dijkstra::potentials::multi_metric_potential::interval_patterns::balanced_interval_pattern;
use cooperative::dijkstra::potentials::multi_metric_potential::potential::MultiMetricPotential;
use cooperative::dijkstra::potentials::TDPotential;
use cooperative::dijkstra::ptv_server::PTVQueryServer;
use cooperative::graph::MAX_BUCKETS;
use cooperative::io::io_ptv_customization::load_interval_minima;
use cooperative::io::io_queries::load_queries;
use cooperative::util::cli_args::parse_arg_required;
use rust_road_router::algo::ch_potentials::CCHPotData;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::algo::TDQuery;
use rust_road_router::datastr::graph::time_dependent::{TDGraph, Timestamp};
use rust_road_router::datastr::graph::{EdgeId, FirstOutGraph, Graph, Weight};
use rust_road_router::datastr::node_order::NodeOrder;
use rust_road_router::io::{Load, Reconstruct};
use rust_road_router::report::measure;
use std::env;
use std::error::Error;
use std::ops::Add;
use std::path::Path;

/// Compare runtimes of different potentials on the same graph with the same queries
/// Uses a given PTV graph and executes a set of pre-defined queries
/// then executes a set of queries (without updates) on several potentials
///
/// Additional parameters: <path_to_graph> <query_path>
fn main() -> Result<(), Box<dyn Error>> {
    let (graph_directory, query_directory) = parse_args()?;
    let path = Path::new(&graph_directory);

    let ((graph, lower_bound), time) = measure(|| {
        (
            TDGraph::reconstruct_from(&path).unwrap(),
            Vec::<u32>::load_from(&path.join("lower_bound")).unwrap(),
        )
    });
    println!("Loaded graph in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    // load pre-generated queries
    let queries = load_queries(&path.join("queries").join(query_directory))?;

    // init cch
    let order = Vec::load_from(path.join("cch_perm"))?;
    let node_order = NodeOrder::from_node_order(order);
    let cch = CCH::fix_order_and_build(&graph, node_order);

    // create departure/travel time structure
    let (departure, travel_time) = retrieve_departure_and_travel_time(&graph);

    // 0th potential: naive backward
    /*let naive_backward_pot = TDBackwardProfilePotential::new(&graph, &departure, &travel_time, 200);
    let mut server = PTVQueryServer::new_with_potential(graph, naive_backward_pot);
    execute_queries(&mut server, &queries, "Naive Backward Potential");
    let (graph, _) = server.decompose();*/

    // 1st potential: CCH lowerbound
    let lower_bound_graph = FirstOutGraph::new(graph.first_out(), graph.head(), &lower_bound[..]);
    let cch_pot_data = CCHPotData::new(&cch, &lower_bound_graph);
    let cch_lowerbound_pot = cch_pot_data.forward_potential();
    let mut server = PTVQueryServer::new_with_potential(graph, cch_lowerbound_pot);

    execute_queries(&mut server, &queries, "CCH Lowerbound Potential");
    let (graph, cch_lowerbound_pot) = server.decompose();
    drop(cch_lowerbound_pot);

    // ----------------------------------------------------------------------------- //
    // 2nd potential: Multi-Level-Bucket Potential
    let customized_multi_levels = CustomizedMultiLevels::new(
        &cch,
        &departure,
        &travel_time,
        &vec![86_400_000 / 6, 86_400_000 / 18],
        graph.num_arcs() as u64 * 100_000,
    );
    let multi_level_bucket_pot = CCHMultiLevelBucketPotential::new_forward(&customized_multi_levels, &cch_pot_data, 2);
    let mut server = PTVQueryServer::new_with_potential(graph, multi_level_bucket_pot);
    execute_queries(&mut server, &queries, "Multi Level Bucket Pot");
    let (graph, pot) = server.decompose();

    drop(pot);
    drop(customized_multi_levels);
    drop(cch_pot_data);

    // ----------------------------------------------------------------------------- //
    // 3rd potential: Multi-Metric Potential
    let (customized_multi_metric, time) = measure(|| CustomizedMultiMetrics::new(&cch, &departure, &travel_time, &balanced_interval_pattern(), 20));
    println!("Customized graph in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);
    let multi_metric_pot = MultiMetricPotential::new(&customized_multi_metric);
    let mut server = PTVQueryServer::new_with_potential(graph, multi_metric_pot);
    execute_queries(&mut server, &queries, "Multi Metric Pot");
    let (graph, pot) = server.decompose();

    drop(pot);
    drop(customized_multi_metric);
    // ----------------------------------------------------------------------------- //
    // 4th potential: Corridor-Lowerbound Potential
    //let td_graph = convert_to_td_graph(&graph);
    let (customized_corridor_lowerbound, time) = measure(|| load_interval_minima(&path.join("customized").join("customized_96")).unwrap());
    println!("Loaded customized data in {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

    //let customized_corridor_lowerbound = CustomizedApproximatedPeriodicTTF::new_from_ptv(&cch, &td_graph, 96);
    let corridor_lowerbound_pot = CorridorLowerboundPotential::new(&customized_corridor_lowerbound);
    let mut server = PTVQueryServer::new_with_potential(graph, corridor_lowerbound_pot);

    execute_queries(&mut server, &queries, "Corridor Lowerbound Potential");
    let (_, pot) = server.decompose();
    println!("Average corridor length: {}s", pot.average_corridor_length() / 1000.0);

    Ok(())
}

fn execute_queries<Pot: TDPotential>(server: &mut PTVQueryServer<Pot>, queries: &Vec<TDQuery<u32>>, pot_name: &str) {
    let mut sum_distances = 0u64;
    let mut num_relaxed_arcs = 0u64;
    let mut num_queue_pops = 0u64;

    let mut time_total = time::Duration::zero();
    let mut time_queries = time::Duration::zero();
    let mut time_potentials = time::Duration::zero();

    queries.iter().enumerate().for_each(|(idx, &query)| {
        let (result, time) = measure(|| server.query(query));

        time_total = time_total.add(time);
        time_queries = time_queries.add(result.time_query);
        time_potentials = time_potentials.add(result.time_potential);

        sum_distances += result.distance.unwrap_or(0) as u64;
        num_relaxed_arcs += result.num_relaxed_arcs as u64;
        num_queue_pops += result.num_queue_pops as u64;

        if (idx + 1) % 1000 == 0 {
            println!("Finished {}/{} queries", idx + 1, queries.len());
        }
    });

    println!("-----------------------------");
    println!("Result for {}:", pot_name);
    println!(
        "Total runtime: {} ms (potential init: {}, query + pot: {})",
        time_total.to_std().unwrap().as_nanos() as f64 / 1_000_000.0,
        time_potentials.to_std().unwrap().as_nanos() as f64 / 1_000_000.0,
        time_queries.to_std().unwrap().as_nanos() as f64 / 1_000_000.0,
    );
    println!(
        "Query statistics: {} relaxed arcs (avg: {}), {} queue pops (avg: {})",
        num_relaxed_arcs,
        num_relaxed_arcs / queries.len() as u64,
        num_queue_pops,
        num_queue_pops / queries.len() as u64
    );
    println!("Total distance: {} (avg: {})", sum_distances, sum_distances / queries.len() as u64);
    println!(
        "Potential estimation: {} (avg: {})",
        server.sum_potentials,
        server.sum_potentials / queries.len() as u64
    );
    println!("-----------------------------");
}

fn parse_args() -> Result<(String, String), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let query_directory: String = parse_arg_required(&mut args, "Query Directory")?;
    Ok((graph_directory, query_directory))
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

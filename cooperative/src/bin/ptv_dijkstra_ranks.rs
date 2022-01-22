use cooperative::dijkstra::potentials::corridor_lowerbound_potential::customization::CustomizedCorridorLowerbound;
use cooperative::dijkstra::potentials::multi_metric_potential::customization::CustomizedMultiMetrics;
use cooperative::dijkstra::ptv_server::{PTVQueryResult, PTVQueryServer};
use cooperative::io::io_ptv_customization::{load_interval_minima, load_multiple_metrics};
use cooperative::io::io_queries::load_queries;
use cooperative::util::cli_args::parse_arg_required;
use rust_road_router::algo::ch_potentials::{BorrowedCCHPot, CCHPotData};
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;
use rust_road_router::algo::TDQuery;
use rust_road_router::datastr::graph::time_dependent::{TDGraph, Timestamp};
use rust_road_router::datastr::graph::FirstOutGraph;
use rust_road_router::datastr::node_order::NodeOrder;
use rust_road_router::io::{Load, Reconstruct};
use rust_road_router::report::measure;
use std::env;
use std::error::Error;
use std::fs::File;
use std::io::Write;
use std::path::Path;

/// Executes a given set of pre-generated Dijkstra-Rank Queries with different potential approaches
/// The query results will be stored as a CSV file (queries/<output_name>.csv) for further visualization (e.g. with R)
///
/// Additional parameters: <path_to_graph> <path_to_queries> <output_name> <corridor_lowerbound_customized_path> <multi_metrics_customized_path>
fn main() -> Result<(), Box<dyn Error>> {
    let (graph_directory, query_directory, output_name, customized_cl, customized_mm) = parse_required_args()?;
    let path = Path::new(&graph_directory);

    // load graph
    let (graph, time) = measure(|| TDGraph::reconstruct_from(&path).unwrap());
    println!("Loaded graph in {} ms", time.as_secs_f64() * 1000.0);

    // init queries
    let query_path = path.join("queries").join(&query_directory);

    let queries = load_queries(&query_path)?;
    let num_queries_per_rank = *Vec::<u32>::load_from(&query_path.join("num_queries")).unwrap().first().unwrap();
    let max_rank = *Vec::<u32>::load_from(&query_path.join("max_rank")).unwrap().first().unwrap();

    let first_rank = max_rank + 1 - (queries.len() as u32 / num_queries_per_rank);
    let mut query_results = Vec::with_capacity(queries.len() * 3);
    debug_assert_eq!(first_rank, 8u32);

    // init cch
    let order = Vec::load_from(path.join("cch_perm"))?;
    let node_order = NodeOrder::from_node_order(order);
    let cch = CCH::fix_order_and_build(&graph, node_order);

    // ----------------------------------------------------------------------------- //
    // 1st potential: CCH lowerbound
    let lower_bound = Vec::<u32>::load_from(&path.join("lower_bound"))?;
    let lower_bound_graph = FirstOutGraph::new(graph.first_out(), graph.head(), &lower_bound[..]);

    let cch_pot_data = CCHPotData::new(&cch, &lower_bound_graph);
    let cch_lowerbound_pot = cch_pot_data.forward_potential();
    let mut server = PTVQueryServer::new(graph, cch_lowerbound_pot);

    execute_queries(
        &mut server,
        |s: &mut PTVQueryServer<BorrowedCCHPot>, q: &TDQuery<u32>| s.query(q),
        &queries,
        first_rank,
        num_queries_per_rank,
        &mut query_results,
        "cch-pot".to_string(),
    );
    let (graph, cch_lowerbound_pot) = server.decompose();
    drop(cch_lowerbound_pot);

    // ----------------------------------------------------------------------------- //
    // 2nd potential: Multi-Metric Potential
    let customized_path = path.join("customized").join(&customized_mm);
    let (customized_multi_metric, time) = measure(|| load_multiple_metrics(&customized_path, cch).unwrap());
    println!("Loaded customized data in {} ms", time.as_secs_f64() * 1000.0);

    let mut server = PTVQueryServer::new(graph, customized_multi_metric);
    execute_queries(
        &mut server,
        |s: &mut PTVQueryServer<CustomizedMultiMetrics>, q: &TDQuery<u32>| s.query(q),
        &queries,
        first_rank,
        num_queries_per_rank,
        &mut query_results,
        "multi-metric".to_string(),
    );
    let (graph, customized) = server.decompose();
    drop(customized);

    // ----------------------------------------------------------------------------- //
    // 3rd potential: Corridor-Lowerbound Potential
    let customized_path = path.join("customized").join(&customized_cl);
    let (customized_corridor_lowerbound, time) = measure(|| load_interval_minima(&customized_path).unwrap());
    println!("Loaded customized data in {} ms", time.as_secs_f64() * 1000.0);

    let mut server = PTVQueryServer::new(graph, customized_corridor_lowerbound);
    execute_queries(
        &mut server,
        |s: &mut PTVQueryServer<CustomizedCorridorLowerbound>, q: &TDQuery<u32>| s.query(q),
        &queries,
        first_rank,
        num_queries_per_rank,
        &mut query_results,
        "corridor-lowerbound".to_string(),
    );
    let (graph, customized) = server.decompose();
    drop(customized);
    drop(graph);

    let file_name = format!("{}.csv", output_name);
    write_results(&query_results, &path.join("queries").join(file_name))
}

fn execute_queries<Customized>(
    server: &mut PTVQueryServer<Customized>,
    query_fn: fn(&mut PTVQueryServer<Customized>, &TDQuery<u32>) -> PTVQueryResult,
    queries: &Vec<TDQuery<Timestamp>>,
    first_rank: u32,
    num_queries_per_rank: u32,
    results: &mut Vec<(String, u32, f64, u32)>,
    pot_name: String,
) {
    queries.iter().enumerate().for_each(|(idx, query)| {
        let (result, time) = measure(|| query_fn(server, query));

        let time = time.as_secs_f64() * 1000.0;
        let rank = first_rank + idx as u32 / num_queries_per_rank;

        results.push((pot_name.clone(), rank, time, result.num_queue_pops));

        if (idx + 1) % 1000 == 0 {
            println!("Finished {}/{} queries", idx + 1, queries.len());
        }
    });
}

fn write_results(results: &Vec<(String, u32, f64, u32)>, path: &Path) -> Result<(), Box<dyn Error>> {
    let mut file = File::create(path)?;

    let header = "pot_name,rank,time,search_space\n";
    file.write(header.as_bytes())?;

    for (pot_name, rank, time, search_space) in results {
        let line = format!("{},{},{},{}\n", pot_name, rank, time, search_space);
        file.write(line.as_bytes())?;
    }

    Ok(())
}

fn parse_required_args() -> Result<(String, String, String, String, String), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory = parse_arg_required(&mut args, "Graph Directory")?;
    let query_directory = parse_arg_required(&mut args, "Query Directory")?;
    let output_name = parse_arg_required(&mut args, "Output File Name")?;
    let customized_cl = parse_arg_required(&mut args, "Corridor-Lowerbound Customized File")?;
    let customized_mm = parse_arg_required(&mut args, "Multi-Metrics Customized File")?;

    Ok((graph_directory, query_directory, output_name, customized_cl, customized_mm))
}

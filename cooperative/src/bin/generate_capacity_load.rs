use std::env;
use std::error::Error;
use std::path::Path;

use cooperative::dijkstra::potentials::init_cch_potential::init_cch_potential;
use cooperative::dijkstra::server::{CapacityServer, CapacityServerOps};
use cooperative::graph::speed_functions::bpr_speed_function;
use cooperative::io::io_graph::{load_capacity_graph, store_capacity_buckets};
use cooperative::io::io_node_order::load_node_order;
use cooperative::io::io_queries::load_queries;
use cooperative::util::cli_args::{parse_arg_optional, parse_arg_required};

/// Executes a given set of queries on a new initialized capacity graph
/// and stores the capacity buckets afterwards.
///
/// Additional parameters: <path_to_graph> <num_buckets> <query_path> <out_directory = [num_buckets]_[timestamp]>
fn main() -> Result<(), Box<dyn Error>> {
    let (path, num_buckets, query_directory, out_directory) = parse_args()?;
    let graph_directory = Path::new(&path);
    let out_directory = Some(out_directory)
        .filter(|val| !val.is_empty())
        .unwrap_or(format!("{}_{}", num_buckets, time::get_time().sec));

    // load graph
    let graph = load_capacity_graph(graph_directory, num_buckets, bpr_speed_function)?;

    // load queries
    let queries = load_queries(&graph_directory.join("queries").join(query_directory))?;

    // load node order, build CCH -> we're using the static lowerbound pot to speed up calculations
    let order = load_node_order(graph_directory)?;
    let cch_pot_data = init_cch_potential(&graph, order);
    let mut server = CapacityServer::new_with_potential(graph, cch_pot_data.forward_potential());

    println!("Initialization finished, starting queries...");

    // execute queries
    queries.iter().enumerate().for_each(|(idx, &query)| {
        server.query(query, true);

        if (idx + 1) % 10000 == 0 {
            println!("Finished {}/{} queries", idx + 1, queries.len());
        }
    });

    // write result
    let (graph, _) = server.decompose();
    store_capacity_buckets(&graph, graph_directory, out_directory)
}

fn parse_args() -> Result<(String, u32, String, String), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let num_buckets: u32 = parse_arg_required(&mut args, "number of buckets")?;
    let query_directory: String = parse_arg_required(&mut args, "path to queries")?;
    let out_directory: String = parse_arg_optional(&mut args, "".to_string());

    Ok((graph_directory, num_buckets, query_directory, out_directory))
}

use std::error::Error;
use std::env;
use std::path::Path;
use rust_road_router::cli::CliErr;
use std::str::FromStr;
use rust_road_router::report::measure;
use cooperative::util::cli_args::{parse_arg_required, parse_arg_optional, QueryType};

/// Runs a specified number of random queries on a given graph.
/// Prints the resulting paths for further visualization.
///
/// Additional parameters: <path_to_graph> <query_type = uniform> <num_buckets = 50> <num_queries = 100>
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let query_type = parse_arg_optional(&mut args, QueryType::Uniform);
    let num_buckets = parse_arg_optional(&mut args, 50u32);
    let num_queries = parse_arg_optional(&mut args, 100u32);

    dbg!(graph_directory, query_type, num_buckets, num_queries);

    Ok(())
}
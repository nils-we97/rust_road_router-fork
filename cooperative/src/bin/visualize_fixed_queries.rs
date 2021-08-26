use std::env;
use std::error::Error;

use cooperative::util::cli_args::parse_arg_required;
use rust_road_router::cli::CliErr;

/// Runs a specified number of predefined queries on a given graph.
/// Prints the resulting paths for further visualization.
/// The query file is expected to be a csv file with columns <source>,<target>,<departure>
///
/// Additional parameters: <path_to_graph> <path_to_query_file>
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let query_file: String = parse_arg_required(&mut args, "Query File")?;

    dbg!(graph_directory, query_file);

    Ok(())
}
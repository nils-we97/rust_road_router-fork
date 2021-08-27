use cooperative::util::cli_args::{parse_arg_optional, parse_arg_required, QueryType};
use rust_road_router::cli::CliErr;
use std::env;
use std::error::Error;
use std::str::FromStr;

/// Evaluates the impact of time-dependent queries.
/// Comparison is provided by a more static setting with only 1 bucket per edge.
/// The evaluation itself takes place on the time-dependent graph.
/// Queries are generated randomly and either uniform, geometric or population-density-based.
///
/// Expected result: with a rising number of long-distance queries, the time-dependent graph
/// should outperform the static setting (in terms of travel times).
///
/// Additional parameters: <path_to_graph> <num_buckets> <num_queries> <query_mode = uniform>
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let num_buckets: u32 = parse_arg_required(&mut args, "number of buckets")?;
    let num_queries: u32 = parse_arg_required(&mut args, "number of queries")?;
    let query_mode = parse_arg_optional(&mut args, QueryType::Uniform);

    dbg!(graph_directory, num_buckets, num_queries, query_mode);
    Ok(())
}

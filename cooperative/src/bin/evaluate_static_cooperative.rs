use std::error::Error;
use std::env;
use cooperative::util::cli_args::{parse_arg_required, QueryType, parse_arg_optional};

/// Evaluate the difference between static (no updates) and cooperative routing.
/// In the static setting, no updates take place after calculating the routes.
/// The evaluation takes place on the frequently updated graph.
///
/// Expected result: with a rising number of queries, the cooperative queries should perform
/// significantly better (in terms of travel time).
///
/// Additional parameters: <path_to_graph> <num_buckets> <num_queries> <query_mode = uniform>
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let num_buckets: u32 = parse_arg_required(&mut args, "number of buckets")?;
    let num_queries: u32 = parse_arg_required(&mut args, "number of queries")?;
    let query_type: QueryType = parse_arg_optional(&mut args, QueryType::Uniform);

    dbg!(graph_directory, num_buckets, num_queries, query_type);

    Ok(())
}
use std::error::Error;
use std::env;
use cooperative::util::cli_args::{parse_arg_required, parse_arg_optional, QueryType};

/// Evaluates the quality of A*-Potentials with a rising number of queries.
/// As the potentials are calculated in advance (e.g. on a lowerbound graph), its quality will
/// get worse with with every additional query.
///
/// The algorithm will terminate as soon as the query runtime exceeds a given order of magnitude
/// in comparison with the first run.
/// To neglect random effects, each run consists of 1000 queries.
///
/// Expected result: with a rising number of queries, the cooperative queries should perform
/// significantly better (in terms of travel time).
///
/// Additional parameters: <path_to_graph> <num_buckets> <slowdown_factor> <query_mode = uniform>
/// <slowdown_factor> must be greater than 1 (and shouldn't be much larger than 10)
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let num_buckets: u32 = parse_arg_required(&mut args, "number of buckets")?;
    let slowdown_factor: f64 = parse_arg_required(&mut args, "slowdown factor")?;
    let query_mode: QueryType = parse_arg_optional(&mut args, QueryType::Uniform);

    dbg!(graph_directory, num_buckets, slowdown_factor, query_mode);

    Ok(())
}
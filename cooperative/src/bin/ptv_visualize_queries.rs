use cooperative::io::io_queries::load_queries;
use cooperative::util::cli_args::parse_arg_required;
use rand::{thread_rng, Rng};
use rust_road_router::io::Load;
use std::env;
use std::error::Error;
use std::path::Path;

/// Visualize start and target of a given query set
///
/// Parameters: <path_to_graph> <query_directory> <number_of_samples>
fn main() -> Result<(), Box<dyn Error>> {
    let (graph_directory, query_directory, number_of_samples) = parse_required_args()?;
    let path = Path::new(&graph_directory);
    let query_path = path.join("queries").join(&query_directory);

    let latitude = Vec::<f32>::load_from(path.join("latitude"))?;
    let longitude = Vec::<f32>::load_from(path.join("longitude"))?;
    let queries = load_queries(&query_path)?;

    let mut rng = thread_rng();

    (0..number_of_samples).into_iter().for_each(|_| {
        let rand = rng.gen_range(0..queries.len());
        let from = queries[rand].from as usize;
        let to = queries[rand].to as usize;

        println!("[[{}, {}], [{}, {}]],", latitude[from], longitude[from], latitude[to], longitude[to]);
    });

    Ok(())
}

fn parse_required_args() -> Result<(String, String, u32), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let query_directory: String = parse_arg_required(&mut args, "number of intervals")?;
    let number_of_samples: u32 = parse_arg_required(&mut args, "number of samples")?;

    Ok((graph_directory, query_directory, number_of_samples))
}

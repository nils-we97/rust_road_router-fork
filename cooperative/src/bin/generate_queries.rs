use cooperative::experiments::queries::departure_distributions::{
    ConstantDeparture, DepartureDistribution, NormalDeparture, RushHourDeparture, UniformDeparture,
};
use cooperative::experiments::queries::dijkstra_rank::{generate_dijkstra_rank_queries, generate_population_dijkstra_rank_queries};
use cooperative::experiments::queries::population_density_based::{
    generate_geometric_population_density_based_queries, generate_uniform_population_density_based_queries,
};
use cooperative::experiments::queries::random_geometric::generate_random_geometric_queries;
use cooperative::experiments::queries::random_uniform::generate_random_uniform_queries;
use cooperative::experiments::queries::{GraphType, QueryType};
use cooperative::graph::speed_functions::bpr_speed_function;
use cooperative::io::io_coordinates::load_coords;
use cooperative::io::io_graph::load_capacity_graph;
use cooperative::io::io_population_grid::load_population_grid;
use cooperative::io::io_queries::store_queries;
use cooperative::util::cli_args::parse_arg_required;
use rust_road_router::datastr::graph::time_dependent::TDGraph;
use rust_road_router::datastr::graph::{FirstOutGraph, Graph, OwnedGraph};
use rust_road_router::io::{Load, Reconstruct, Store};
use std::env;
use std::error::Error;
use std::path::Path;

/// Generate random queries and store them in a given directory
///
/// First parameters: <path_to_graph> <type = CAPACITY/PTV> <num_queries> <query_type> <output_directory>
/// Additional parameters, depending on `query_type`:
/// uniform/geometric: ---
/// population-grid-based: <path_to_population_grid_file>
/// dijkstra-rank: <max_rank_pow> (for each rank power 7 <= i <= max_rank_power), `num_queries` are generated
/// population-grid & dijkstra-rank: <path_to_population_grid_file> <max_rank_pow>
///
/// Results will be written to directory <path_to_graph>/queries/<output_directory>/
fn main() -> Result<(), Box<dyn Error>> {
    let (path, graph_type, num_queries, query_type, output_directory, mut remaining_args) = parse_required_args()?;
    let graph_directory = Path::new(&path);

    let graph = match graph_type {
        GraphType::PTV => {
            let graph = TDGraph::reconstruct_from(&graph_directory).unwrap();
            println!("Number of constant edges: {} of {}", graph.num_constant(), graph.num_arcs());
            let lower_bound = Vec::<u32>::load_from(&graph_directory.join("lower_bound")).unwrap();
            OwnedGraph::new(graph.first_out().to_vec(), graph.head().to_vec(), lower_bound)
        }
        GraphType::CAPACITY => {
            let graph = load_capacity_graph(graph_directory, 1, bpr_speed_function).unwrap();
            OwnedGraph::new(graph.first_out().to_vec(), graph.head().to_vec(), graph.freeflow_time().to_vec())
        }
    };

    let (queries, additional_data) = match query_type {
        QueryType::Uniform => {
            let queries = generate_random_uniform_queries(graph.num_nodes() as u32, num_queries, UniformDeparture::new());
            (queries, None)
        }
        QueryType::UniformRushHourDep => {
            let queries = generate_random_uniform_queries(graph.num_nodes() as u32, num_queries, RushHourDeparture::new());
            (queries, None)
        }
        QueryType::UniformNormalDep => {
            let queries = generate_random_uniform_queries(graph.num_nodes() as u32, num_queries, NormalDeparture::new());
            (queries, None)
        }
        QueryType::Geometric | QueryType::GeometricRushHourDep => {
            let queries = match graph_type {
                GraphType::PTV => {
                    // for PTV graphs, we do not have a valid distance metric => use travel time instead
                    if query_type == QueryType::Geometric {
                        generate_random_geometric_queries(&graph, false, num_queries, UniformDeparture::new())
                    } else {
                        generate_random_geometric_queries(&graph, false, num_queries, RushHourDeparture::new())
                    }
                }
                GraphType::CAPACITY => {
                    // capacity graph has its own distance metric => rebuild graph before
                    let distance = Vec::<u32>::load_from(graph_directory.join("distance"))?;
                    let distance_graph = FirstOutGraph::new(graph.first_out(), graph.head(), distance);

                    if query_type == QueryType::Geometric {
                        generate_random_geometric_queries(&distance_graph, true, num_queries, UniformDeparture::new())
                    } else {
                        generate_random_geometric_queries(&distance_graph, true, num_queries, RushHourDeparture::new())
                    }
                }
            };

            (queries, None)
        }
        QueryType::DijkstraRank | QueryType::DijkstraRankRushHourDep => {
            let max_rank_pow: u32 = parse_arg_required(&mut remaining_args, "power of last rank (2^x)")?;
            let queries = if query_type == QueryType::DijkstraRank {
                generate_dijkstra_rank_queries(&graph, num_queries, max_rank_pow, UniformDeparture::new())
            } else {
                generate_dijkstra_rank_queries(&graph, num_queries, max_rank_pow, RushHourDeparture::new())
            };

            (queries, Some(vec![("num_queries", vec![num_queries]), ("max_rank", vec![max_rank_pow])]))
        }
        QueryType::PopulationDijkstraRank | QueryType::PopulationDijkstraRankRushHourDep => {
            // load population data
            let population_path: String = parse_arg_required(&mut remaining_args, "population grid directory")?;
            let population_directory = Path::new(&population_path);
            let (longitude, latitude) = load_coords(graph_directory)?;
            let (grid_tree, grid_population) = load_population_grid(population_directory)?;

            // retrieve dijkstra-rank data
            let max_rank_pow: u32 = parse_arg_required(&mut remaining_args, "power of last rank (2^x)")?;

            let queries = if query_type == QueryType::PopulationDijkstraRank {
                generate_population_dijkstra_rank_queries(
                    &longitude,
                    &latitude,
                    &grid_tree,
                    &grid_population,
                    &graph,
                    num_queries,
                    max_rank_pow,
                    UniformDeparture::new(),
                )
            } else {
                generate_population_dijkstra_rank_queries(
                    &longitude,
                    &latitude,
                    &grid_tree,
                    &grid_population,
                    &graph,
                    num_queries,
                    max_rank_pow,
                    RushHourDeparture::new(),
                )
            };

            (queries, Some(vec![("num_queries", vec![num_queries]), ("max_rank", vec![max_rank_pow])]))
        }
        _ => {
            // for population queries, we have to use some additional data
            let population_path: String = parse_arg_required(&mut remaining_args, "population grid directory")?;
            let population_directory = Path::new(&population_path);

            let (longitude, latitude) = load_coords(graph_directory)?;
            let (grid_tree, grid_population) = load_population_grid(population_directory)?;

            let queries = match query_type {
                QueryType::PopulationUniform => {
                    generate_uniform_population_density_based_queries(&longitude, &latitude, &grid_tree, &grid_population, num_queries, UniformDeparture::new())
                }
                QueryType::PopulationUniformConstantDep => generate_uniform_population_density_based_queries(
                    &longitude,
                    &latitude,
                    &grid_tree,
                    &grid_population,
                    num_queries,
                    ConstantDeparture::new(),
                ),
                QueryType::PopulationGeometric => generate_geometric_population_density_based_queries(
                    &graph,
                    &longitude,
                    &latitude,
                    &grid_tree,
                    &grid_population,
                    num_queries,
                    RushHourDeparture::new(),
                ),
                _ => unimplemented!(),
            };

            (queries, None)
        }
    };

    // check if subfolder `queries` exists
    let query_directory = graph_directory.join("queries");
    if !query_directory.exists() {
        std::fs::create_dir(query_directory)?;
    }

    // add new subfolder in `queries`
    let output_dir = graph_directory.join("queries").join(output_directory);
    if output_dir.exists() {
        panic!("This output directory exists already!");
    } else {
        std::fs::create_dir(&output_dir)?;
    }

    store_queries(&queries, &output_dir)?;

    if let Some(v) = additional_data {
        for (name, data) in v {
            data.write_to(&output_dir.join(name))?;
        }
    }

    println!("Wrote {} queries to {}", num_queries, output_dir.display());

    Ok(())
}

fn parse_required_args() -> Result<(String, GraphType, u32, QueryType, String, impl Iterator<Item = String>), Box<dyn Error>> {
    let mut args = env::args().skip(1);

    let graph_directory: String = parse_arg_required(&mut args, "Graph Directory")?;
    let graph_type = parse_arg_required(&mut args, "Graph Type (PTV/CAPACITY)")?;
    let num_queries: u32 = parse_arg_required(&mut args, "number of queries")?;
    let query_type = parse_arg_required(&mut args, "query type")?;
    let output_directory: String = parse_arg_required(&mut args, "Query Output Directory")?;

    Ok((graph_directory, graph_type, num_queries, query_type, output_directory, args))
}

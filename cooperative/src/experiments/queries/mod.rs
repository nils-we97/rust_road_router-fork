use std::str::FromStr;

use rust_road_router::algo::TDQuery;
use rust_road_router::cli::CliErr;
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{Link, LinkIterable};

use crate::experiments::queries::departure_distributions::{ConstantDeparture, DepartureDistribution, NormalDeparture, UniformDeparture};
use crate::experiments::queries::population_density_based::generate_uniform_population_density_based_queries;
use crate::experiments::queries::random_geometric::generate_random_geometric_queries;
use crate::experiments::queries::random_uniform::generate_random_uniform_queries;
use crate::io::io_population_grid::PopulationGridEntry;
use kdtree::kdtree::Kdtree;

pub mod departure_distributions;
pub mod population_density_based;
pub mod random_geometric;
pub mod random_uniform;

#[derive(Debug, Clone)]
pub enum QueryType {
    Uniform,
    UniformRushHourDep,
    UniformNormalDep,
    Geometric,
    GeometricRushHourDep,
    PopulationUniform,
    PopulationUniformConstantDep,
    PopulationGeometric,
    PopulationGeometricConstantDep,
}

impl FromStr for QueryType {
    type Err = CliErr;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_uppercase().as_str() {
            "UNIFORM" => Ok(QueryType::Uniform),
            "UNIFORM_RUSH_HOUR" => Ok(QueryType::UniformRushHourDep),
            "UNIFORM_NORMAL_DEPARTURE" => Ok(QueryType::UniformNormalDep),
            "GEOMETRIC" => Ok(QueryType::Geometric),
            "GEOMETRIC_RUSH_HOUR" => Ok(QueryType::GeometricRushHourDep),
            "POPULATION_UNIFORM" => Ok(QueryType::PopulationUniform),
            "POPULATION_UNIFORM_CONSTANT_DEPARTURE" => Ok(QueryType::PopulationUniformConstantDep),
            "POPULATION_GEOMETRIC" => Ok(QueryType::PopulationGeometric),
            "POPULATION_GEOMETRIC_CONSTANT_DEPARTURE" => Ok(QueryType::PopulationGeometricConstantDep),
            _ => Err(CliErr("Unknown Query Type!")),
        }
    }
}

pub fn generate_queries<G: LinkIterable<Link>>(graph: &G, query_type: QueryType, num_queries: u32) -> Vec<TDQuery<Timestamp>> {
    match query_type {
        QueryType::Uniform => generate_random_uniform_queries(graph.num_nodes() as u32, num_queries, UniformDeparture::new()),
        QueryType::UniformNormalDep => generate_random_uniform_queries(graph.num_nodes() as u32, num_queries, NormalDeparture::new()),
        QueryType::Geometric => generate_random_geometric_queries(graph, num_queries, UniformDeparture::new()),
        _ => unimplemented!(),
    }
}

pub fn generate_population_queries<G: LinkIterable<Link>>(
    _graph: &G,
    query_type: QueryType,
    num_queries: u32,
    grid_tree: &Kdtree<PopulationGridEntry>,
    grid_population: &Vec<u32>,
    longitude: &Vec<f32>,
    latitude: &Vec<f32>,
) -> Vec<TDQuery<Timestamp>> {
    // TODO implementation for geometric queries
    match query_type {
        QueryType::PopulationUniform => {
            generate_uniform_population_density_based_queries(longitude, latitude, grid_tree, grid_population, num_queries, UniformDeparture::new())
        }
        QueryType::PopulationUniformConstantDep => {
            generate_uniform_population_density_based_queries(longitude, latitude, grid_tree, grid_population, num_queries, ConstantDeparture::new())
        }
        /*QueryType::PopulationGeometric => {}
        QueryType::PopulationGeometricConstantDep => {}*/
        _ => unimplemented!(),
    }
}

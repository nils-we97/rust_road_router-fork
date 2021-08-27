use std::str::FromStr;

use rust_road_router::algo::TDQuery;
use rust_road_router::cli::CliErr;
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{Link, LinkIterable};

use crate::experiments::queries::departure_distributions::{ConstantDeparture, DepartureDistribution, UniformDeparture};
use crate::experiments::queries::random_geometric::generate_random_geometric_queries;
use crate::experiments::queries::random_uniform::generate_random_uniform_queries;

pub mod departure_distributions;
pub mod population_density_based;
pub mod random_geometric;
pub mod random_uniform;

#[derive(Debug, Clone)]
pub enum QueryType {
    Uniform,
    UniformConstantDep,
    Geometric,
    GeometricConstantDep,
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
            "UNIFORM_CONSTANT_DEPARTURE" => Ok(QueryType::UniformConstantDep),
            "GEOMETRIC" => Ok(QueryType::Geometric),
            "GEOMETRIC_CONSTANT_DEPARTURE" => Ok(QueryType::GeometricConstantDep),
            /*"POPULATION_UNIFORM" => Ok(QueryType::PopulationUniform),
            "POPULATION_UNIFORM_CONSTANT_DEPARTURE" => Ok(QueryType::PopulationUniformConstantDep),
            "POPULATION_GEOMETRIC" => Ok(QueryType::PopulationGeometric),
            "POPULATION_GEOMETRIC_CONSTANT_DEPARTURE" => Ok(QueryType::PopulationGeometricConstantDep),*/
            _ => Err(CliErr("Unknown Query Type!")),
        }
    }
}

pub fn generate_queries<G: LinkIterable<Link>>(graph: &G, query_type: QueryType, num_queries: u32) -> Vec<TDQuery<Timestamp>> {
    match query_type {
        QueryType::Uniform => generate_random_uniform_queries(graph.num_nodes() as u32, num_queries, UniformDeparture::new()),
        QueryType::UniformConstantDep => generate_random_uniform_queries(graph.num_nodes() as u32, num_queries, ConstantDeparture::new()),
        QueryType::Geometric => generate_random_geometric_queries(graph, num_queries, UniformDeparture::new()),
        QueryType::GeometricConstantDep => generate_random_geometric_queries(graph, num_queries, ConstantDeparture::new()),
        _ => unimplemented!(),
    }
}

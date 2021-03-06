use rand::{thread_rng, Rng};

use rust_road_router::algo::{GenQuery, TDQuery};
use rust_road_router::datastr::graph::time_dependent::Timestamp;

use crate::experiments::queries::departure_distributions::DepartureDistribution;
use crate::experiments::queries::population_density_based::{build_population_grid, find_population_interval};
use crate::io::io_population_grid::PopulationGridEntry;
use kdtree::kdtree::Kdtree;
use rust_road_router::algo::dijkstra::{DefaultOps, DijkstraData, DijkstraInit, DijkstraRun};
use rust_road_router::datastr::graph::{Link, LinkIterable, NodeId};

pub fn generate_dijkstra_rank_queries<G: LinkIterable<Link>, D: DepartureDistribution>(
    graph: &G,
    num_queries_per_rank: u32,
    max_rank_pow: u32,
    mut departure_distribution: D,
) -> Vec<TDQuery<Timestamp>> {
    let max_rank = 2u32.pow(max_rank_pow);

    // assert that context is valid, i.e. the maximum rank is feasible
    // this assumption must also hold in release mode!
    assert!(max_rank_pow < 32, "Max. rank power should be less than 32!");
    assert!(
        graph.num_nodes() as u32 > max_rank,
        "Max. rank power is too high for the current graph (max rank: {}, num nodes: {})",
        max_rank,
        graph.num_nodes()
    );

    // init context
    let mut rng = thread_rng();
    let mut data = DijkstraData::new(graph.num_nodes());
    let mut queries = vec![TDQuery::new(0, 0, 0); (num_queries_per_rank * (max_rank_pow - 7)) as usize];

    for query_idx in 0..num_queries_per_rank as usize {
        let mut result: Option<Vec<NodeId>> = None;
        let mut source = 0;

        while result.is_none() {
            let mut rank_nodes = Vec::with_capacity(max_rank_pow as usize);

            // pick a random start node
            source = rng.gen_range(0..graph.num_nodes()) as NodeId;

            let mut ops = DefaultOps::default();
            let mut dijkstra = DijkstraRun::query(graph, &mut data, &mut ops, DijkstraInit::from(source));
            let mut counter = 0u32;
            let mut next_rank = 256; // 2^8, direct neighbors make no sense!

            while let Some(node) = dijkstra.next() {
                counter += 1;
                if counter == next_rank {
                    rank_nodes.push(node);

                    if next_rank == max_rank {
                        result = Some(rank_nodes);
                        break;
                    } else {
                        next_rank *= 2;
                    }
                }
            }
        }

        result.unwrap().iter().enumerate().for_each(|(rank_idx, &target)| {
            let query = &mut queries[rank_idx * num_queries_per_rank as usize + query_idx];
            query.from = source;
            query.to = target;
            // pick a random departure in each query!
            query.departure = departure_distribution.rand(&mut rng);
        });
    }

    queries
}

pub fn generate_population_dijkstra_rank_queries<G: LinkIterable<Link>, D: DepartureDistribution>(
    longitude: &Vec<f32>,
    latitude: &Vec<f32>,
    grid_tree: &Kdtree<PopulationGridEntry>,
    grid_population: &Vec<u32>,
    graph: &G,
    num_queries_per_rank: u32,
    max_rank_pow: u32,
    mut departure_distribution: D,
) -> Vec<TDQuery<Timestamp>> {
    // init population grid
    let (vertex_grid, grid_population_intervals, population_counter) = build_population_grid(longitude, latitude, grid_tree, grid_population);

    // init dijkstra-rank structs
    assert!(max_rank_pow < 32, "Max. rank power should be less than 32!");
    let max_rank = 2u32.pow(max_rank_pow);
    assert!(
        graph.num_nodes() as u32 > max_rank,
        "Max. rank power is too high for the current graph (max rank: {}, num nodes: {})",
        max_rank,
        graph.num_nodes()
    );

    // init context
    let mut rng = thread_rng();
    let mut data = DijkstraData::new(graph.num_nodes());
    let mut queries = vec![TDQuery::new(0, 0, 0); (num_queries_per_rank * (max_rank_pow - 7)) as usize];

    for query_idx in 0..num_queries_per_rank as usize {
        let mut result: Option<Vec<NodeId>> = None;
        let mut source = 0;

        while result.is_none() {
            let mut rank_nodes = Vec::with_capacity(max_rank_pow as usize);

            // pick a random start node according to population density
            let start_cell_id = find_population_interval(&grid_population_intervals, rng.gen_range(0..population_counter));
            let start_cell_vertex_pos = rng.gen_range(0..vertex_grid[start_cell_id].len());
            source = vertex_grid[start_cell_id][start_cell_vertex_pos];

            let mut ops = DefaultOps::default();
            let mut dijkstra = DijkstraRun::query(graph, &mut data, &mut ops, DijkstraInit::from(source));
            let mut counter = 0u32;
            let mut next_rank = 256; // 2^8, direct neighbors make no sense!

            while let Some(node) = dijkstra.next() {
                counter += 1;
                if counter == next_rank {
                    rank_nodes.push(node);

                    if next_rank == max_rank {
                        result = Some(rank_nodes);
                        break;
                    } else {
                        next_rank *= 2;
                    }
                }
            }
        }

        result.unwrap().iter().enumerate().for_each(|(rank_idx, &target)| {
            let query = &mut queries[rank_idx * num_queries_per_rank as usize + query_idx];
            query.from = source;
            query.to = target;
            // pick a random departure in each query!
            query.departure = departure_distribution.rand(&mut rng);
        });
    }

    queries
}

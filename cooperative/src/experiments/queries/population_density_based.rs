use kdtree::kdtree::Kdtree;
use rand::{thread_rng, Rng};

use rust_road_router::algo::{GenQuery, TDQuery};
use rust_road_router::datastr::graph::NodeId;

use crate::experiments::queries::departure_distributions::DepartureDistribution;
use crate::io::io_population_grid::PopulationGridEntry;
use rust_road_router::datastr::graph::time_dependent::Timestamp;

pub fn generate_uniform_population_density_based_queries<D: DepartureDistribution>(
    longitude: &Vec<f32>,
    latitude: &Vec<f32>,
    grid_tree: &Kdtree<PopulationGridEntry>,
    grid_population: &Vec<u32>,
    num_queries: u32,
    mut departure_distribution: D,
) -> Vec<TDQuery<Timestamp>> {
    // distribute population into buckets
    let mut vertex_grid = vec![Vec::new(); grid_population.len()];

    (0..longitude.len()).into_iter().for_each(|node_id| {
        let nearest_cell = grid_tree.nearest_search(&PopulationGridEntry::from_coords(longitude[node_id], latitude[node_id]));
        vertex_grid[nearest_cell.id].push(node_id as NodeId);
    });

    // build prefix sum upon population entries
    // consider a cell's population if there is at least one corresponding node
    let mut grid_population_intervals = Vec::new();
    let mut population_counter = 0;

    for i in 0..grid_population.len() {
        if !vertex_grid[i].is_empty() {
            grid_population_intervals.push((population_counter, i));
            population_counter += grid_population[i];
        }
    }
    grid_population_intervals.push((population_counter, grid_population.len())); // sentinel element

    // generate queries based on population inside each grid
    let mut rng = thread_rng();
    let mut queries = (0..num_queries)
        .into_iter()
        .map(|_| {
            // draw random start cell according to population density, pick a random node inside
            let start_cell_id = find_interval(&grid_population_intervals, rng.gen_range(0..population_counter));
            let start_cell_vertex_pos = rng.gen_range(0..vertex_grid[start_cell_id].len());
            let from = vertex_grid[start_cell_id][start_cell_vertex_pos];

            let target_cell_id = find_interval(&grid_population_intervals, rng.gen_range(0..population_counter));
            let target_cell_vertex_pos = rng.gen_range(0..vertex_grid[target_cell_id].len());
            let to = vertex_grid[target_cell_id][target_cell_vertex_pos];

            TDQuery::new(from, to, departure_distribution.rand(&mut rng))
        })
        .collect::<Vec<TDQuery<Timestamp>>>();

    // sort queries by departure for a more realistic usage scenario
    queries.sort_by_key(|query| query.departure);

    queries
}

fn find_interval(vec: &Vec<(u32, usize)>, val: u32) -> usize {
    let idx = vec.binary_search_by_key(&val, |&(prefix_sum, _)| prefix_sum);

    if idx.is_ok() {
        vec[idx.unwrap()].1
    } else {
        debug_assert!(idx.unwrap_err() >= 1 && idx.unwrap_err() < vec.len(), "Missing sentinel elements!");
        vec[idx.unwrap_err() - 1].1
    }
}

// TODO geometric queries, ...

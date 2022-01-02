use kdtree::kdtree::Kdtree;
use rand::{thread_rng, Rng};

use rust_road_router::algo::{GenQuery, TDQuery};
use rust_road_router::datastr::graph::{Link, LinkIterable, NodeId};

use crate::experiments::queries::departure_distributions::DepartureDistribution;
use crate::io::io_population_grid::PopulationGridEntry;
use rand_distr::Distribution;
use rand_distr::Geometric;
use rust_road_router::algo::dijkstra::{DefaultOps, DijkstraData, DijkstraInit, DijkstraRun};
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use std::collections::HashSet;

const INV_AVERAGE_TRIP_LENGTH: f64 = 0.000025; // avg trip length is ~40 km
const INV_AVERAGE_TRIP_DURATION: f64 = 1.0 / (2_700_000.0); // avg trip duration: 45 minutes

pub fn generate_uniform_population_density_based_queries<D: DepartureDistribution>(
    longitude: &Vec<f32>,
    latitude: &Vec<f32>,
    grid_tree: &Kdtree<PopulationGridEntry>,
    grid_population: &Vec<u32>,
    num_queries: u32,
    mut departure_distribution: D,
) -> Vec<TDQuery<Timestamp>> {
    // init population grid
    let (vertex_grid, grid_population_intervals, population_counter) = build_population_grid(longitude, latitude, grid_tree, grid_population);

    // generate queries based on population inside each grid
    let mut rng = thread_rng();
    let mut queries = (0..num_queries)
        .into_iter()
        .map(|_| {
            // draw random start cell according to population density, pick a random node inside
            let start_cell_id = find_population_interval(&grid_population_intervals, rng.gen_range(0..population_counter));
            let start_cell_vertex_pos = rng.gen_range(0..vertex_grid[start_cell_id].len());
            let from = vertex_grid[start_cell_id][start_cell_vertex_pos];

            let target_cell_id = find_population_interval(&grid_population_intervals, rng.gen_range(0..population_counter));
            let target_cell_vertex_pos = rng.gen_range(0..vertex_grid[target_cell_id].len());
            let to = vertex_grid[target_cell_id][target_cell_vertex_pos];

            TDQuery::new(from, to, departure_distribution.rand(&mut rng))
        })
        .collect::<Vec<TDQuery<Timestamp>>>();

    // sort queries by departure for a more realistic usage scenario
    queries.sort_by_key(|query| query.departure);

    queries
}

pub fn generate_geometric_population_density_based_queries<D: DepartureDistribution, G: LinkIterable<Link>>(
    graph: &G,
    longitude: &Vec<f32>,
    latitude: &Vec<f32>,
    grid_tree: &Kdtree<PopulationGridEntry>,
    grid_population: &Vec<u32>,
    num_queries: u32,
    mut departure_distribution: D,
    use_distance_metric: bool,
) -> Vec<TDQuery<Timestamp>> {
    // init population grid
    let (vertex_grid, grid_population_intervals, population_counter) = build_population_grid(longitude, latitude, grid_tree, grid_population);

    // generate queries based on population inside each grid
    let mut rng = thread_rng();
    let mut data = DijkstraData::new(graph.num_nodes());

    let probability = if use_distance_metric {
        INV_AVERAGE_TRIP_LENGTH
    } else {
        INV_AVERAGE_TRIP_DURATION
    };
    let distribution = Geometric::new(probability).unwrap();

    let mut queries = (0..num_queries)
        .into_iter()
        .map(|i| {
            if (i + 1) % 100 == 0 {
                println!("Finished {} queries", (i + 1));
            }

            // run a query to determine the target cell
            let mut possible_target_cells = HashSet::new();
            let mut from = 0;

            // double check that we have a valid result
            while possible_target_cells.is_empty() {
                possible_target_cells.clear();

                // draw random start cell according to population density, pick a random node inside
                let start_cell_id = find_population_interval(&grid_population_intervals, rng.gen_range(0..population_counter));
                let start_cell_vertex_pos = rng.gen_range(0..vertex_grid[start_cell_id].len());
                from = vertex_grid[start_cell_id][start_cell_vertex_pos];

                // draw distance according to geometric distribution
                let distance = distribution.sample(&mut rng) as u32;
                // allow a slight deviation to discover more cells in the closer neighborhood
                let lower_threshold = (distance * 9) / 10;
                let upper_threshold = (distance * 11) / 10;

                let query = TDQuery::new(from, 0, 0);
                let mut ops = DefaultOps::default();
                let mut dijkstra = DijkstraRun::query(graph, &mut data, &mut ops, DijkstraInit::from_query(&query));

                while let Some(node) = dijkstra.next() {
                    if *dijkstra.tentative_distance(node) > upper_threshold {
                        // cancel as soon as the tentative distance exceeds the threshold
                        break;
                    } else if *dijkstra.tentative_distance(node) >= lower_threshold {
                        // otherwise, everything within [lower, upper] threshold is relevant
                        let grid_search = PopulationGridEntry::from_coords(longitude[node as usize], latitude[node as usize]);
                        possible_target_cells.insert(grid_tree.nearest_search(&grid_search).id);
                    }
                }
            }

            // build prefix sum over population inside those cells
            let mut target_population_intervals = Vec::new();
            let mut population_counter = 0;

            for cell in possible_target_cells {
                // double check if there is a vertex inside that cell
                if vertex_grid[cell].len() > 0 {
                    target_population_intervals.push((population_counter, cell));
                    population_counter += grid_population[cell];
                }
            }
            target_population_intervals.push((population_counter, grid_population.len())); // sentinel element

            // select the target cell by population density, then pick a random node inside there
            let rand = rng.gen_range(0..population_counter);
            let mut selected_cell = 0;
            for i in 0..target_population_intervals.len() - 1 {
                if target_population_intervals[i].0 <= rand && target_population_intervals[i + 1].0 > rand {
                    selected_cell = target_population_intervals[i].1;
                }
            }
            let target_cell_vertex_pos = rng.gen_range(0..vertex_grid[selected_cell].len());
            let to = vertex_grid[selected_cell][target_cell_vertex_pos];

            TDQuery::new(from, to, departure_distribution.rand(&mut rng))
        })
        .collect::<Vec<TDQuery<Timestamp>>>();

    // sort queries by departure for a more realistic usage scenario
    queries.sort_by_key(|query| query.departure);

    queries
}

pub fn find_population_interval(vec: &Vec<(u32, usize)>, val: u32) -> usize {
    let idx = vec.binary_search_by_key(&val, |&(prefix_sum, _)| prefix_sum);

    if idx.is_ok() {
        vec[idx.unwrap()].1
    } else {
        debug_assert!(idx.unwrap_err() >= 1 && idx.unwrap_err() < vec.len(), "Missing sentinel elements!");
        vec[idx.unwrap_err() - 1].1
    }
}

pub fn build_population_grid(
    longitude: &Vec<f32>,
    latitude: &Vec<f32>,
    grid_tree: &Kdtree<PopulationGridEntry>,
    grid_population: &Vec<u32>,
) -> (Vec<Vec<u32>>, Vec<(u32, usize)>, u32) {
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

    // sentinel element
    grid_population_intervals.push((population_counter, grid_population.len()));

    (vertex_grid, grid_population_intervals, population_counter)
}

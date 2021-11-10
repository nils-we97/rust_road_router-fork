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

const INV_AVERAGE_TRIP_LENGTH: f64 = 1.0 / 1_000_000.0;

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

pub fn generate_geometric_population_density_based_queries<D: DepartureDistribution, G: LinkIterable<Link>>(
    graph: &G,
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
    let mut data = DijkstraData::new(graph.num_nodes());
    let distribution = Geometric::new(INV_AVERAGE_TRIP_LENGTH).unwrap();

    let mut queries = (0..num_queries)
        .into_iter()
        .map(|_| {
            // run a query to determine the target cell
            let mut result = None;

            // double check that we have a valid result
            while result.is_none() {
                // draw random start cell according to population density, pick a random node inside
                let start_cell_id = find_interval(&grid_population_intervals, rng.gen_range(0..population_counter));
                let start_cell_vertex_pos = rng.gen_range(0..vertex_grid[start_cell_id].len());
                let from = vertex_grid[start_cell_id][start_cell_vertex_pos];

                // draw distance according to geometric distribution, allow corridor of +- 5km
                let distance = distribution.sample(&mut rng) as u32;

                let query = TDQuery::new(from, 0, 0);
                let mut ops = DefaultOps::default();
                let mut dijkstra = DijkstraRun::query(graph, &mut data, &mut ops, DijkstraInit::from_query(&query));

                while let Some(node) = dijkstra.next() {
                    // cancel as soon as the tentative distance exceeds the threshold
                    if *dijkstra.tentative_distance(node) > distance {
                        result = Some((from, node));
                        break;
                    }
                }
            }

            // get the cell of the current node
            let (from, node) = result.unwrap();
            let node_cell = grid_tree.nearest_search(&PopulationGridEntry::from_coords(longitude[node as usize], latitude[node as usize]));

            // look in a distance of 1000 x 1000m around that node, insert all cells
            let mut possible_target_cells = HashSet::new();
            for width in -5..5 {
                for height in -5..5 {
                    let point = PopulationGridEntry {
                        id: 0,
                        coords: [
                            node_cell.coords[0] + width as f64 * (100.0 / 111_111.0),
                            node_cell.coords[1] + height as f64 * (100.0 / 111_111.0),
                        ],
                    };
                    possible_target_cells.insert(grid_tree.nearest_search(&point).id);
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

fn find_interval(vec: &Vec<(u32, usize)>, val: u32) -> usize {
    let idx = vec.binary_search_by_key(&val, |&(prefix_sum, _)| prefix_sum);

    if idx.is_ok() {
        vec[idx.unwrap()].1
    } else {
        debug_assert!(idx.unwrap_err() >= 1 && idx.unwrap_err() < vec.len(), "Missing sentinel elements!");
        vec[idx.unwrap_err() - 1].1
    }
}

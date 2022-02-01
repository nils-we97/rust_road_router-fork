use rust_road_router::algo::dijkstra::{DijkstraData, DijkstraOps, Label, State};
use rust_road_router::algo::{GenQuery, TDQuery};
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{Arc, EdgeId, EdgeIdT, Graph, LinkIterable, NodeIdT, Weight, INFINITY};
use rust_road_router::datastr::index_heap::Indexing;
use rust_road_router::report;
use rust_road_router::report::*;
use std::time::{Duration, Instant};

use crate::dijkstra::capacity_dijkstra_ops::CapacityDijkstraOps;
use crate::dijkstra::model::{CapacityQueryResult, DistanceMeasure, MeasuredCapacityQueryResult, PathResult};
use crate::dijkstra::potentials::corridor_lowerbound_potential::customization::CustomizedCorridorLowerbound;
use crate::dijkstra::potentials::corridor_lowerbound_potential::CorridorLowerboundPotential;
use crate::dijkstra::potentials::multi_metric_potential::customization::CustomizedMultiMetrics;
use crate::dijkstra::potentials::multi_metric_potential::potential::MultiMetricPotential;
use crate::dijkstra::potentials::TDPotential;
use crate::graph::capacity_graph::CapacityGraph;
use rust_road_router::algo::customizable_contraction_hierarchy::CCH;

pub struct CapacityServer<PotCustomized> {
    graph: CapacityGraph,
    dijkstra: DijkstraData<Weight, EdgeIdT, Weight>,
    customized: PotCustomized,
    result_valid: bool,
    update_valid: bool,
}

impl<PotCustomized> CapacityServer<PotCustomized> {
    pub fn new(graph: CapacityGraph, customized: PotCustomized) -> Self {
        let n = graph.num_nodes();

        Self {
            graph,
            dijkstra: DijkstraData::new(n),
            customized,
            result_valid: true,
            update_valid: true,
        }
    }

    pub fn result_valid(&self) -> bool {
        self.result_valid
    }

    pub fn update_valid(&self) -> bool {
        self.update_valid
    }

    pub fn decompose(self) -> (CapacityGraph, PotCustomized) {
        (self.graph, self.customized)
    }

    pub fn borrow_graph(&self) -> &CapacityGraph {
        &self.graph
    }

    fn distance_internal<Pot: TDPotential>(
        dijkstra: &mut DijkstraData<Weight, EdgeIdT, Weight>,
        graph: &CapacityGraph,
        pot: &mut Pot,
        result_valid: &mut bool,
        query: &TDQuery<Timestamp>,
    ) -> DistanceMeasure {
        report!("algo", "TD Dijkstra with Capacities");

        // if the latest result was not valid, block the query execution
        if !*result_valid {
            return DistanceMeasure {
                distance: None,
                potential: None,
                time_potential: Duration::ZERO,
                time_query: Duration::ZERO,
                num_queue_pushs: 0,
                num_queue_pops: 0,
                num_relaxed_arcs: 0,
            };
        }

        let mut result = None;
        let mut num_queue_pops = 0;
        let mut num_queue_pushs = 0;
        let mut num_relaxed_arcs = 0;

        // time-dependent potentials are a little bit more complicated
        // for now, a slight modification of the generic dijkstra code should suffice

        // prepro: initialize potential
        let (_, time_potential) = measure(|| pot.init(query.from, query.to, query.departure));

        let start = Instant::now();
        let mut ops = CapacityDijkstraOps::default();

        // 1. reset data
        dijkstra.queue.clear();
        dijkstra.distances.reset();

        // 2. init dijkstra from start node
        dijkstra.queue.push(State {
            key: query.departure,
            node: query.from,
        });
        dijkstra.distances[query.from as usize] = query.departure;
        dijkstra.predecessors[query.from as usize].0 = query.from;

        // 3. run query
        while let Some(State { node, .. }) = dijkstra.queue.pop() {
            num_queue_pops += 1;

            if node == query.to {
                result = Some(dijkstra.distances[query.to as usize] - dijkstra.distances[query.from as usize]);
                break;
            }

            for link in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(graph, node) {
                num_relaxed_arcs += 1;
                let linked = ops.link(graph, &dijkstra.predecessors, NodeIdT(node), &dijkstra.distances[node as usize], &link);

                if ops.merge(&mut dijkstra.distances[link.head() as usize], linked) {
                    dijkstra.predecessors[link.head() as usize] = (node, ops.predecessor_link(&link));
                    let next_distance = &dijkstra.distances[link.head() as usize];

                    if let Some(next_key) = pot.potential(link.head(), next_distance.clone()).map(|p| p + next_distance.key()) {
                        let next = State {
                            node: link.head(),
                            key: next_key,
                        };
                        if dijkstra.queue.contains_index(next.as_index()) {
                            dijkstra.queue.decrease_key(next);
                        } else {
                            num_queue_pushs += 1;
                            dijkstra.queue.push(next);
                        }
                    }
                }
            }
        }

        let time_query = start.elapsed();

        *result_valid = match result {
            None => {
                // case that should not happen: not reachable, but potential says so
                pot.potential(query.from, query.departure).is_none()
            }
            Some(1) => {
                // nasty edge cases, caused by our graph preprocessing -> everything okay here
                println!("-- WARNING: Distance 1, Potential: {:?}", &pot.potential(query.from, query.departure));
                true
            }
            Some(dist) => dist >= pot.potential(query.from, query.departure).unwrap_or(INFINITY) && pot.verify_result(dist),
        };

        if !*result_valid {
            println!(
                "Result: {}, Potential: {}",
                result.unwrap_or(INFINITY),
                pot.potential(query.from, query.departure).unwrap_or(INFINITY)
            );
        }

        DistanceMeasure {
            distance: result.filter(|_| *result_valid),
            potential: pot.potential(query.from, query.departure),
            time_potential,
            time_query,
            num_queue_pushs,
            num_queue_pops,
            num_relaxed_arcs,
        }
    }

    fn path_internal(&self, query: &TDQuery<Timestamp>) -> PathResult {
        let mut node_path = Vec::new();
        let mut edge_path = Vec::new();
        node_path.push(query.to());

        // determine path nodes/edges by recursively traversing through the predecessors of the target node
        while *node_path.last().unwrap() != query.from() {
            let (next_node, next_edge) = self.dijkstra.predecessors[*node_path.last().unwrap() as usize];
            node_path.push(next_node);
            edge_path.push(next_edge.0);
        }

        // reverse paths
        node_path.reverse();
        edge_path.reverse();

        // determine timestamps of departures at each vertex
        let mut departure = Vec::with_capacity(node_path.len());
        let mut current_time = query.initial_state();

        for i in 0..node_path.len() - 1 {
            departure.push(current_time);

            // update travel time by traversing the next edge at the current time
            let ttf = self.graph.travel_time_function(edge_path[i]);
            current_time += ttf.eval(current_time)
        }

        departure.push(current_time); // arrival time at target node

        PathResult::new(node_path, edge_path, departure)
    }

    fn path_distance_internal(&self, edge_path: &Vec<EdgeId>, query_start: Timestamp) -> Weight {
        let mut duration = 0;

        for edge in edge_path {
            duration += self.graph.eval_history_free(*edge, query_start + duration);

            // abort if the distance exceeds infinity
            if duration > INFINITY {
                duration = INFINITY;
                break;
            }
        }

        duration
    }
}

impl CapacityServer<CustomizedCorridorLowerbound> {
    pub fn customize(&mut self, mut customized: CustomizedCorridorLowerbound) {
        std::mem::swap(&mut self.customized, &mut customized);
        self.result_valid = true;
        self.update_valid = true;
    }

    pub fn customize_upper_bound(&mut self, cch: &CCH) {
        self.customized.customize_upper_bound(cch, &self.graph);
        self.result_valid = true;
        self.update_valid = true;
    }
}

impl CapacityServer<CustomizedMultiMetrics> {
    pub fn customize(&mut self, intervals: &Vec<(u32, u32)>, num_max_metrics: usize) {
        self.customized.customize(&self.graph, intervals, num_max_metrics);
        self.result_valid = true;
        self.update_valid = true;
    }

    pub fn customize_upper_bound(&mut self) {
        self.customized.customize_upper_bound(&self.graph);
        self.result_valid = true;
        self.update_valid = true;
    }
}

pub trait CapacityServerOps {
    fn distance(&mut self, query: &TDQuery<Timestamp>) -> DistanceMeasure;
    fn update(&mut self, path: &PathResult);
    fn path(&self, query: &TDQuery<Timestamp>) -> PathResult;
    fn path_distance(&self, edge_path: &Vec<EdgeId>, query_start: Timestamp) -> Weight;

    fn query(&mut self, query: &TDQuery<Timestamp>, update: bool) -> Option<CapacityQueryResult> {
        if let Some(distance) = self.distance(query).distance {
            let path = self.path(&query);
            debug_assert_eq!(*path.departure.last().unwrap() - *path.departure.first().unwrap(), distance);
            if update {
                self.update(&path);
            }
            Some(CapacityQueryResult::new(distance, path))
        } else {
            None
        }
    }

    fn query_measured(&mut self, query: &TDQuery<Timestamp>, update: bool) -> MeasuredCapacityQueryResult {
        let distance_result = self.distance(query);

        if let Some(distance) = distance_result.distance {
            let path = self.path(query);
            debug_assert_eq!(*path.departure.last().unwrap() - *path.departure.first().unwrap(), distance);

            if update {
                let (_, update_time) = measure(|| self.update(&path));

                MeasuredCapacityQueryResult {
                    query_result: Some(CapacityQueryResult::new(distance, path)),
                    distance_result,
                    update_time,
                }
            } else {
                MeasuredCapacityQueryResult {
                    query_result: Some(CapacityQueryResult::new(distance, path)),
                    distance_result,
                    update_time: Duration::ZERO,
                }
            }
        } else {
            MeasuredCapacityQueryResult {
                query_result: None,
                distance_result,
                update_time: Duration::ZERO,
            }
        }
    }
}

impl<PotCustomized: TDPotential> CapacityServerOps for CapacityServer<PotCustomized> {
    fn distance(&mut self, query: &TDQuery<u32>) -> DistanceMeasure {
        Self::distance_internal(&mut self.dijkstra, &self.graph, &mut self.customized, &mut self.result_valid, query)
    }

    fn update(&mut self, path: &PathResult) {
        self.graph.increase_weights(&path.edge_path, &path.departure);
    }

    fn path(&self, query: &TDQuery<Timestamp>) -> PathResult {
        self.path_internal(query)
    }

    fn path_distance(&self, edge_path: &Vec<EdgeId>, query_start: Timestamp) -> Weight {
        self.path_distance_internal(edge_path, query_start)
    }
}

impl CapacityServerOps for CapacityServer<CustomizedMultiMetrics> {
    fn distance(&mut self, query: &TDQuery<Timestamp>) -> DistanceMeasure {
        let mut pot = MultiMetricPotential::prepare(&mut self.customized);

        Self::distance_internal(&mut self.dijkstra, &self.graph, &mut pot, &mut self.result_valid, query)
    }

    fn update(&mut self, path: &PathResult) {
        self.update_valid = self
            .graph
            .increase_weights(&path.edge_path, &path.departure)
            .iter()
            .all(|&(edge_id, edge_lower, edge_upper)| {
                if let Some(shortcut_id) = self.customized.orig_edge_to_forward_shortcut[edge_id as usize] {
                    let (lower_bound, upper_bound) = self.customized.forward_cch_bounds[shortcut_id as usize];

                    debug_assert!(lower_bound <= edge_lower);
                    if upper_bound < edge_upper {
                        println!("Bound violated: Found {}, expected <= {}", edge_upper, upper_bound);
                        return false;
                    }
                }

                if let Some(shortcut_id) = self.customized.orig_edge_to_backward_shortcut[edge_id as usize] {
                    let (lower_bound, upper_bound) = self.customized.backward_cch_bounds[shortcut_id as usize];

                    debug_assert!(lower_bound <= edge_lower);
                    if upper_bound < edge_upper {
                        println!("Bound violated: Found {}, expected <= {}", edge_upper, upper_bound);
                        return false;
                    }
                }

                true
            });
    }

    fn path(&self, query: &TDQuery<Timestamp>) -> PathResult {
        self.path_internal(query)
    }

    fn path_distance(&self, edge_path: &Vec<EdgeId>, query_start: Timestamp) -> u32 {
        self.path_distance_internal(edge_path, query_start)
    }
}

impl CapacityServerOps for CapacityServer<CustomizedCorridorLowerbound> {
    fn distance(&mut self, query: &TDQuery<Timestamp>) -> DistanceMeasure {
        let mut pot = CorridorLowerboundPotential::prepare_capacity(&mut self.customized);

        Self::distance_internal(&mut self.dijkstra, &self.graph, &mut pot, &mut self.result_valid, query)
    }

    fn update(&mut self, path: &PathResult) {
        debug_assert!(self.customized.customized_bounds.is_some());
        let customized_bounds = self.customized.customized_bounds.as_ref().unwrap();

        self.update_valid = self
            .graph
            .increase_weights(&path.edge_path, &path.departure)
            .iter()
            .all(|&(edge_id, lower_bound, upper_bound)| {
                debug_assert!(upper_bound > 0);
                if let Some(shortcut_id) = customized_bounds.orig_edge_to_forward_shortcut[edge_id as usize] {
                    debug_assert!(customized_bounds.upward[shortcut_id as usize].0 <= lower_bound);
                    if customized_bounds.upward[shortcut_id as usize].1 < upper_bound {
                        println!(
                            "Bound violated: Found {}, expected <= {}",
                            upper_bound, customized_bounds.upward[shortcut_id as usize].1
                        );
                        return false;
                    }
                }

                if let Some(shortcut_id) = customized_bounds.orig_edge_to_backward_shortcut[edge_id as usize] {
                    debug_assert!(customized_bounds.downward[shortcut_id as usize].0 <= lower_bound);
                    if customized_bounds.downward[shortcut_id as usize].1 < upper_bound {
                        println!(
                            "Bound violated: Found {}, expected <= {}",
                            upper_bound, customized_bounds.downward[shortcut_id as usize].1
                        );
                        return false;
                    }
                }

                true
            });
    }

    fn path(&self, query: &TDQuery<Timestamp>) -> PathResult {
        self.path_internal(query)
    }

    fn path_distance(&self, edge_path: &Vec<EdgeId>, query_start: Timestamp) -> u32 {
        self.path_distance_internal(edge_path, query_start)
    }
}

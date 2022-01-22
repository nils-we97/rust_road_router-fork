use crate::dijkstra::capacity_dijkstra_ops::CapacityDijkstraOps;
use crate::dijkstra::model::{CapacityQueryResult, DistanceMeasure, MeasuredCapacityQueryResult, PathResult};
use crate::dijkstra::potentials::multi_metric_potential::customization::CustomizedMultiMetrics;
use crate::dijkstra::potentials::multi_metric_potential::potential::MultiMetricPotential;
use crate::dijkstra::potentials::TDPotential;
use crate::dijkstra::server::CapacityServerOps;
use crate::graph::capacity_graph::CapacityGraph;
use crate::graph::ModifiableWeight;
use rust_road_router::algo::dijkstra::{DijkstraData, DijkstraOps, Label, State};
use rust_road_router::algo::{GenQuery, TDQuery};
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{Arc, EdgeId, EdgeIdT, Graph, LinkIterable, NodeIdT, Weight, INFINITY};
use rust_road_router::datastr::index_heap::Indexing;
use rust_road_router::report::measure;
use std::borrow::BorrowMut;
use std::time::{Duration, Instant};

pub struct MultiMetricPotentialServer {
    graph: CapacityGraph,
    dijkstra: DijkstraData<Weight, EdgeIdT, Weight>,
    customized: CustomizedMultiMetrics,
    requires_pot_update: bool,
}

impl MultiMetricPotentialServer {
    pub fn new(graph: CapacityGraph, customized: CustomizedMultiMetrics) -> Self {
        let n = graph.num_nodes();

        Self {
            graph,
            dijkstra: DijkstraData::new(n),
            customized,
            requires_pot_update: false,
        }
    }

    pub fn requires_pot_update(&self) -> bool {
        self.requires_pot_update
    }

    pub fn decompose(self) -> (CapacityGraph, CustomizedMultiMetrics) {
        (self.graph, self.customized)
    }

    pub fn borrow_graph(&self) -> &CapacityGraph {
        &self.graph
    }

    pub fn customize(&mut self) {
        //std::mem::swap(&mut self.customized, &mut customized);
        self.requires_pot_update = false;
    }

    pub fn customize_upper_bound(&mut self) {
        self.customized.customize_upper_bound(&self.graph);
        self.requires_pot_update = false;
    }
}

impl CapacityServerOps for MultiMetricPotentialServer {
    fn query(&mut self, query: TDQuery<Timestamp>, update: bool) -> Option<CapacityQueryResult> {
        let query_result = self.distance(query.clone());

        if self.requires_pot_update {
            None
        } else if let Some(distance) = query_result.distance {
            let path = self.path(query);
            debug_assert_eq!(*path.departure.last().unwrap() - *path.departure.first().unwrap(), distance);
            if update {
                self.update(&path);
            }
            Some(CapacityQueryResult::new(distance, path))
        } else {
            None
        }
    }

    fn query_measured(&mut self, query: TDQuery<u32>, update: bool) -> MeasuredCapacityQueryResult {
        let dist = self.distance(query.clone());

        if let Some(distance) = dist.distance {
            let path = self.path(query);

            if update {
                let (_, update_time) = measure(|| self.update(&path));

                MeasuredCapacityQueryResult {
                    query_result: Some(CapacityQueryResult::new(distance, path)),
                    distance_result: dist,
                    update_time,
                }
            } else {
                MeasuredCapacityQueryResult {
                    query_result: Some(CapacityQueryResult::new(distance, path)),
                    distance_result: dist,
                    update_time: Duration::ZERO,
                }
            }
        } else {
            MeasuredCapacityQueryResult {
                query_result: None,
                distance_result: dist,
                update_time: Duration::ZERO,
            }
        }
    }

    fn update(&mut self, path: &PathResult) {
        self.graph.increase_weights(&path.edge_path, &path.departure);
    }

    fn distance(&mut self, query: TDQuery<Timestamp>) -> DistanceMeasure {
        let from = query.from();
        let init = query.initial_state();
        let to = query.to();

        let mut result = None;
        let mut num_queue_pops = 0;
        let mut num_queue_pushs = 0;
        let mut num_relaxed_arcs = 0;

        // time-dependent potentials are a little bit more complicated
        // for now, a slight modification of the generic dijkstra code should suffice

        // prepro: initialize potential
        let mut pot = MultiMetricPotential::prepare(self.customized.borrow_mut());
        let (_, time_potential) = measure(|| pot.init(from, to, query.departure));

        let start = Instant::now();
        let mut ops = CapacityDijkstraOps::default();

        // 1. reset data
        self.dijkstra.queue.clear();
        self.dijkstra.distances.reset();

        // 2. init dijkstra from start node
        self.dijkstra.queue.push(State { key: init.key(), node: from });
        self.dijkstra.distances[from as usize] = init;
        self.dijkstra.predecessors[from as usize].0 = from;

        // 3. run query
        while let Some(State { node, .. }) = self.dijkstra.queue.pop() {
            num_queue_pops += 1;

            if node == to {
                result = Some(self.dijkstra.distances[to as usize] - self.dijkstra.distances[from as usize]);
                break;
            }

            for link in LinkIterable::<(NodeIdT, EdgeIdT)>::link_iter(&self.graph, node) {
                num_relaxed_arcs += 1;
                let linked = ops.link(
                    &self.graph,
                    &self.dijkstra.predecessors,
                    NodeIdT(node),
                    &self.dijkstra.distances[node as usize],
                    &link,
                );

                if ops.merge(&mut self.dijkstra.distances[link.head() as usize], linked) {
                    self.dijkstra.predecessors[link.head() as usize] = (node, ops.predecessor_link(&link));
                    let next_distance = &self.dijkstra.distances[link.head() as usize];

                    if let Some(next_key) = pot.potential(link.head(), next_distance.clone()).map(|p| p + next_distance.key()) {
                        let next = State {
                            node: link.head(),
                            key: next_key,
                        };
                        if self.dijkstra.queue.contains_index(next.as_index()) {
                            self.dijkstra.queue.decrease_key(next);
                        } else {
                            num_queue_pushs += 1;
                            self.dijkstra.queue.push(next);
                        }
                    }
                }
            }
        }

        let time_query = start.elapsed();

        self.requires_pot_update =
            !pot.verify_result(result.unwrap_or(INFINITY)) || result.unwrap_or(INFINITY + 1) < pot.potential(from, init).unwrap_or(INFINITY);
        if self.requires_pot_update {
            println!(
                "Lower-Bound property violated on query {:?}: Result: {}, Potential: {}",
                &query,
                result.unwrap_or(INFINITY),
                pot.potential(from, init).unwrap_or(INFINITY)
            );
        }

        DistanceMeasure {
            distance: result.filter(|_| !self.requires_pot_update),
            time_potential,
            time_query,
            num_queue_pushs,
            num_queue_pops,
            num_relaxed_arcs,
        }
    }

    fn path(&self, query: TDQuery<Timestamp>) -> PathResult {
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

    fn path_distance(&self, edge_path: &Vec<EdgeId>, query_start: Timestamp) -> Weight {
        let mut duration = 0;

        for edge in edge_path {
            duration += self.graph.travel_time_function(*edge).eval(query_start + duration);

            // abort if the distance exceeds infinity
            if duration > INFINITY {
                duration = INFINITY;
                break;
            }
        }

        duration
    }
}
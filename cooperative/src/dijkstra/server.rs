use rust_road_router::algo::a_star::ZeroPotential;
use rust_road_router::algo::dijkstra::{DijkstraData, DijkstraOps, Label, State};
use rust_road_router::algo::{GenQuery, TDQuery};
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::datastr::graph::{Arc, EdgeIdT, Graph, Weight};
use rust_road_router::datastr::index_heap::Indexing;
use rust_road_router::report;
use rust_road_router::report::*;

use crate::dijkstra::capacity_dijkstra_ops::CapacityDijkstraOps;
use crate::dijkstra::get_neighbors;
use crate::dijkstra::model::{CapacityQueryResult, PathResult};
use crate::dijkstra::potentials::TDPotential;
use crate::graph::capacity_graph::CapacityGraph;
use crate::graph::ModifiableWeight;
use std::ops::Add;

pub struct CapacityServer<Pot = ZeroPotential> {
    graph: CapacityGraph,
    dijkstra: DijkstraData<Weight, EdgeIdT>,
    potential: Pot,
}

impl<Pot: TDPotential> CapacityServer<Pot> {
    pub fn new(graph: CapacityGraph) -> CapacityServer {
        let nodes = graph.num_nodes();

        CapacityServer {
            graph,
            dijkstra: DijkstraData::new(nodes),
            potential: ZeroPotential(),
        }
    }

    pub fn new_with_potential(graph: CapacityGraph, potential: Pot) -> CapacityServer<Pot> {
        let nodes = graph.num_nodes();

        CapacityServer {
            graph,
            dijkstra: DijkstraData::new(nodes),
            potential,
        }
    }

    pub fn decompose(self) -> (CapacityGraph, Pot) {
        (self.graph, self.potential)
    }

    pub fn borrow_graph(&self) -> &CapacityGraph {
        &self.graph
    }
}

pub trait CapacityServerOps {
    fn query(&mut self, query: TDQuery<Timestamp>, update: bool) -> Option<CapacityQueryResult>;
    fn query_measured(
        &mut self,
        query: TDQuery<Timestamp>,
        update: bool,
    ) -> (time::Duration, time::Duration, time::Duration, time::Duration, Option<CapacityQueryResult>);
    fn update(&mut self, path: &PathResult) -> (time::Duration, time::Duration);
    fn distance(&mut self, query: TDQuery<Timestamp>) -> (time::Duration, time::Duration, Option<Weight>);
    fn path(&self, query: TDQuery<Timestamp>) -> PathResult;
    fn path_distance(&self, path: &PathResult) -> Weight;
}

impl<Pot: TDPotential> CapacityServerOps for CapacityServer<Pot> {
    fn query(&mut self, query: TDQuery<Timestamp>, update: bool) -> Option<CapacityQueryResult> {
        let (_, _, distance) = self.distance(query.clone());

        if distance.is_some() {
            let path = self.path(query);
            debug_assert_eq!(*path.departure.last().unwrap() - *path.departure.first().unwrap(), distance.unwrap());
            if update {
                self.update(&path);
            }
            return Some(CapacityQueryResult::new(distance.unwrap(), path));
        }

        return None;
    }

    fn query_measured(
        &mut self,
        query: TDQuery<u32>,
        update: bool,
    ) -> (time::Duration, time::Duration, time::Duration, time::Duration, Option<CapacityQueryResult>) {
        let (time_potential, time_query, distance) = self.distance(query.clone());

        if distance.is_some() {
            let (path, time_path) = measure(|| self.path(query));

            if update {
                let (time_buckets, time_ttf) = self.update(&path);
                (
                    time_potential,
                    time_query.add(time_path),
                    time_buckets,
                    time_ttf,
                    Some(CapacityQueryResult::new(distance.unwrap(), path)),
                )
            } else {
                (
                    time_potential,
                    time_query.add(time_path),
                    time::Duration::zero(),
                    time::Duration::zero(),
                    Some(CapacityQueryResult::new(distance.unwrap(), path)),
                )
            }
        } else {
            (time_potential, time_query, time::Duration::zero(), time::Duration::zero(), None)
        }
    }

    fn update(&mut self, path: &PathResult) -> (time::Duration, time::Duration) {
        //let (_, t) = measure(|| self.graph.increase_weights(&path.edge_path, &path.departure));
        //(t, time::Duration::zero())
        self.graph.increase_weights(&path.edge_path, &path.departure)
    }

    fn distance(&mut self, query: TDQuery<Timestamp>) -> (time::Duration, time::Duration, Option<Weight>) {
        report!("algo", "TD Dijkstra with Capacities");

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
        let (_, time_potential) = measure(|| self.potential.init(from, to, query.departure));
        let pot = &mut self.potential;

        let start = time::now();
        let mut ops = CapacityDijkstraOps::default();

        // 1. reset data
        self.dijkstra.queue.clear();
        self.dijkstra.distances.reset();

        // 2. init dijkstra from start node
        self.dijkstra.queue.push(State { key: init.key(), node: from });
        self.dijkstra.distances[from as usize] = init;
        self.dijkstra.predecessors[from as usize].0 = from;

        // 3. run query
        while let Some(State { node, key }) = self.dijkstra.queue.pop() {
            num_queue_pops += 1;

            if node == to {
                // distance labels can be greater than `MAX_BUCKETS`, so simply taking the difference suffices (no modulo operations required!)
                result = Some(self.dijkstra.distances[to as usize] - self.dijkstra.distances[from as usize]);
                break;
            }

            for link in get_neighbors(&self.graph, node) {
                num_relaxed_arcs += 1;
                let linked = ops.link(&self.graph, &self.dijkstra.distances[node as usize], &link);

                if ops.merge(&mut self.dijkstra.distances[link.head() as usize], linked) {
                    self.dijkstra.predecessors[link.head() as usize] = (node, ops.predecessor_link(&link));
                    let next_distance = &self.dijkstra.distances[link.head() as usize];

                    if let Some(next_key) = pot.potential(link.head(), key).map(|p| p + next_distance.key()) {
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

        let time_dijkstra = time::now() - start;
        report!("num_queue_pops", num_queue_pops);
        report!("num_queue_pushs", num_queue_pushs);
        report!("num_relaxed_arcs", num_relaxed_arcs);

        (time_potential, time_dijkstra, result)
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

    fn path_distance(&self, path: &PathResult) -> Weight {
        path.edge_path
            .iter()
            .enumerate()
            .map(|(idx, &edge_id)| self.graph.travel_time_function(edge_id).eval(path.departure[idx]))
            .sum()
    }
}

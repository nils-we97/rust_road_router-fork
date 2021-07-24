use std::borrow::Borrow;

use rust_road_router::algo::GenQuery;
use rust_road_router::algo::dijkstra::{DijkstraData, DijkstraRun, DefaultOpsWithLinkPath};
use rust_road_router::datastr::graph::{EdgeId, Graph, Weight, EdgeIdT};
use rust_road_router::datastr::graph::time_dependent::Timestamp;
use rust_road_router::report::*;
use rust_road_router::report;

use crate::dijkstra::model::{CapacityQueryResult, PathResult, TDPathResult};
use crate::dijkstra::td_capacity_dijkstra_ops::TDCapacityDijkstraOps;
use crate::graph::capacity_graph::CapacityGraph;
use crate::graph::ModifiableWeight;
use crate::graph::td_capacity_graph::TDCapacityGraph;

pub struct CapacityServer<G> {
    graph: G,
    dijkstra: DijkstraData<Weight, EdgeIdT>,
}

pub trait CapacityServerOps<G, P> {
    fn new(graph: G) -> CapacityServer<G>;
    fn query(&mut self, query: impl GenQuery<Weight> + Clone, update: bool) -> Option<CapacityQueryResult<P>>;
    fn update(&mut self, path: &P);
    fn distance(&mut self, query: impl GenQuery<Weight> + Clone) -> Option<Weight>;
    fn path(&self, query: impl GenQuery<Weight> + Clone) -> P;
}

impl CapacityServerOps<CapacityGraph, PathResult> for CapacityServer<CapacityGraph> {
    fn new(graph: CapacityGraph) -> CapacityServer<CapacityGraph> {
        let nodes = graph.num_nodes();

        CapacityServer {
            graph,
            dijkstra: DijkstraData::new(nodes),
        }
    }

    fn query(
        &mut self,
        query: impl GenQuery<Weight> + Clone,
        update: bool,
    ) -> Option<CapacityQueryResult<PathResult>> {
        let query_copy = query.clone();

        let (distance, time) = measure(|| self.distance(query));
        println!("Query took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

        if distance.is_some() {
            let path = self.path(query_copy);
            if update {
                let (_, time) = measure(|| self.update(&path));
                println!("Update took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);
            }
            return Some(CapacityQueryResult::new(distance.unwrap(), path));
        }

        return None;
    }

    fn update(&mut self, path: &PathResult) {
        self.graph.increase_weights(&path.edge_path);
    }

    fn distance(&mut self, query: impl GenQuery<Weight>) -> Option<Weight> {
        report!("algo", "Dijkstra with Capacities");

        let to = query.to();
        let mut ops = DefaultOpsWithLinkPath::default();
        let mut dijkstra = DijkstraRun::query(self.graph.borrow(), &mut self.dijkstra, &mut ops, query);

        let mut result = None;
        let mut num_queue_pops = 0;
        while let Some(node) = dijkstra.next() {
            num_queue_pops += 1;
            if node == to {
                result = Some(*dijkstra.tentative_distance(node));
                break;
            }
        }

        report!("num_queue_pops", num_queue_pops);
        report!("num_queue_pushs", dijkstra.num_queue_pushs());
        report!("num_relaxed_arcs", dijkstra.num_relaxed_arcs());

        result
    }

    fn path(&self, query: impl GenQuery<Weight>) -> PathResult {
        let mut node_path = Vec::new();
        let mut edge_path = Vec::new();
        node_path.push(query.to());

        while *node_path.last().unwrap() != query.from() {
            let (next_node, next_edge) = self.dijkstra.predecessors[*node_path.last().unwrap() as usize];
            node_path.push(next_node);
            edge_path.push(next_edge.0);
        }
        node_path.reverse();
        edge_path.reverse();

        PathResult::new(node_path, edge_path)
    }
}

impl CapacityServerOps<TDCapacityGraph, TDPathResult> for CapacityServer<TDCapacityGraph> {
    fn new(graph: TDCapacityGraph) -> CapacityServer<TDCapacityGraph> {
        let nodes = graph.num_nodes();

        CapacityServer {
            graph,
            dijkstra: DijkstraData::new(nodes),
        }
    }

    fn query(
        &mut self,
        query: impl GenQuery<Weight> + Clone,
        update: bool,
    ) -> Option<CapacityQueryResult<TDPathResult>> {
        let query_copy = query.clone();
        let (distance, time) = measure(|| self.distance(query));
        println!("//Query took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);

        if distance.is_some() {
            let path = self.path(query_copy);
            if update {
                let (_, time) = measure(|| self.update(&path));
                println!("//Update took {} ms", time.to_std().unwrap().as_nanos() as f64 / 1_000_000.0);
            }
            return Some(CapacityQueryResult::new(distance.unwrap(), path));
        }

        return None;
    }

    fn update(&mut self, path: &TDPathResult) {
        let edges_with_timestamps = path
            .edge_path
            .iter()
            .zip(path.departure.iter())
            .map(|(&a, &b)| (a, b))
            .collect::<Vec<(EdgeId, Timestamp)>>();

        self.graph.increase_weights(&edges_with_timestamps);
    }

    fn distance(&mut self, query: impl GenQuery<Weight>) -> Option<Weight> {
        report!("algo", "TD Dijkstra with Capacities");

        let to = query.to();
        let mut ops = TDCapacityDijkstraOps::default();

        let mut dijkstra = DijkstraRun::query(
            self.graph.borrow(),
            &mut self.dijkstra,
            &mut ops,
            query,
        );

        let mut result = None;
        let mut num_queue_pops = 0;
        while let Some(node) = dijkstra.next() {
            num_queue_pops += 1;
            if node == to {
                result = Some(*dijkstra.tentative_distance(node));
                break;
            }
        }

        report!("num_queue_pops", num_queue_pops);
        report!("num_queue_pushs", dijkstra.num_queue_pushs());
        report!("num_relaxed_arcs", dijkstra.num_relaxed_arcs());

        result
    }

    fn path(&self, query: impl GenQuery<Weight>) -> TDPathResult {
        let mut node_path = Vec::new();
        let mut edge_path = Vec::new();
        node_path.push(query.to());

        // determine path nodes/edges by recursively traversing through the predecessors of the target node
        while *node_path.last().unwrap() != query.from() {
            let (next_node, next_edge) = self.dijkstra.predecessors[*node_path.last().unwrap() as usize];
            node_path.push(next_node);
            edge_path.push(next_edge.0);
        }

        // determine timestamps of departures at each vertex
        // TODO: verify distance == (departure[last] - departure[first])
        let mut departure = Vec::with_capacity(node_path.len());
        let mut current_time = query.initial_state();

        for i in 0..node_path.len() - 1 {
            departure.push(current_time);

            // update travel time by traversing the next edge at the current time
            let ttf = self.graph.travel_time_function(edge_path[i]);
            current_time += ttf.eval(current_time)
        }

        departure.push(current_time); // arrival time at target node


        TDPathResult::new(node_path, edge_path, departure)
    }
}
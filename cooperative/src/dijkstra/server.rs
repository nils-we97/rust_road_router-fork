use rust_road_router::algo::dijkstra::{DijkstraData, DijkstraRun, DefaultOps};
use rust_road_router::datastr::graph::{Weight, Graph, NodeId};
use crate::graph::capacity_graph::{CapacityGraph, ModifiableWeight};
use rust_road_router::algo::{GenQuery, Query};
use std::borrow::Borrow;
use rust_road_router::report::*;
use rust_road_router::report;

pub struct CapacityServer {
    graph: CapacityGraph,
    dijkstra: DijkstraData<Weight>
}

impl CapacityServer {
    pub fn new(graph: CapacityGraph) -> CapacityServer {
        let nodes = graph.num_nodes();

        CapacityServer {
            graph,
            dijkstra: DijkstraData::new(nodes),
        }
    }

    pub fn query(&mut self, query: impl GenQuery<Weight>) -> Option<(Weight, Vec<NodeId>)> {
        let query_copy = Query::new(query.from(), query.to(), 0);

        self
            .distance(query)
            .map(|dist| (dist, self.path(query_copy)))
    }

    pub fn update(&mut self, path: &Vec<NodeId>) {
        let mut edge_path = vec![0; path.len() - 1];

        for i in 0..path.len() - 1 {
            edge_path[i] = self.graph.get_edge_id(path[i], path[i+1]).unwrap();
        }

        self.graph.increase_weights(&edge_path);
    }

    fn distance(&mut self, query: impl GenQuery<Weight>) -> Option<Weight> {
        report!("algo", "Dijkstra with Capacities");

        let to = query.to();
        let mut ops = DefaultOps::default();
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

    fn path(&self, query: impl GenQuery<Weight>) -> Vec<NodeId> {
        let mut path = Vec::new();
        path.push(query.to());

        while *path.last().unwrap() != query.from() {
            let next = self.dijkstra.predecessors[*path.last().unwrap() as usize];
            path.push(next);
        }

        path.reverse();

        path
    }
}
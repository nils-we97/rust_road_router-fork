use rust_road_router::algo::dijkstra::DijkstraOps;
use rust_road_router::datastr::graph::{EdgeIdT, NodeId, NodeIdT, Weight};

use crate::graph::capacity_graph::CapacityGraph;

pub struct CapacityDijkstraOps();

impl DijkstraOps<CapacityGraph> for CapacityDijkstraOps {
    type Label = Weight;
    type Arc = (NodeIdT, EdgeIdT);
    type LinkResult = Weight;
    type PredecessorLink = EdgeIdT;

    #[inline(always)]
    fn link(&mut self, graph: &CapacityGraph, _parents: &[(NodeId, EdgeIdT)], _tail: NodeIdT, label: &Weight, link: &Self::Arc) -> Self::LinkResult {
        label + graph.travel_time_function(link.1 .0).eval(*label)
    }

    #[inline(always)]
    fn merge(&mut self, label: &mut Weight, linked: Self::LinkResult) -> bool {
        if linked < *label {
            *label = linked;
            return true;
        }
        false
    }

    fn predecessor_link(&self, &(_, edge_id): &Self::Arc) -> Self::PredecessorLink {
        edge_id
    }
}

impl Default for CapacityDijkstraOps {
    fn default() -> Self {
        Self {}
    }
}

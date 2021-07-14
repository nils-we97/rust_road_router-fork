use crate::graph::td_capacity_graph::TDCapacityGraph;
use rust_road_router::algo::dijkstra::DijkstraOps;
use rust_road_router::datastr::graph::{Weight, NodeId, EdgeId};

pub struct TDCapacityDijkstraOps();

impl DijkstraOps<TDCapacityGraph> for TDCapacityDijkstraOps {
    type Label = Weight;
    type Arc = (NodeId, EdgeId);
    type LinkResult = Weight;

    #[inline(always)]
    fn link(&mut self, graph: &TDCapacityGraph, label: &Weight, link: &Self::Arc) -> Self::LinkResult {
        label + graph.travel_time_function(link.1).eval(*label)
    }

    #[inline(always)]
    fn merge(&mut self, label: &mut Weight, linked: Self::LinkResult) -> bool {
        if linked < *label {
            *label = linked;
            return true;
        }
        false
    }
}

impl Default for TDCapacityDijkstraOps {
    fn default() -> Self {
        Self {}
    }
}
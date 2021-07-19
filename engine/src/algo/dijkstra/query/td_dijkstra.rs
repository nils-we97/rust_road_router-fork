use super::*;
use crate::datastr::graph::time_dependent::*;

pub struct TDDijkstraOps();

impl DijkstraOps<TDGraph> for TDDijkstraOps {
    type Label = Weight;
    type LinkResult = Weight;
    type Arc = (NodeIdT, EdgeIdT);
    type PredecessorLink = ();

    #[inline(always)]
    fn link(&mut self, graph: &TDGraph, label: &Weight, link: &Self::Arc) -> Self::LinkResult {
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

    #[inline(always)]
    fn predecessor_link(&self, _link: &Self::Arc) -> Self::PredecessorLink {
        ()
    }
}

impl Default for TDDijkstraOps {
    fn default() -> Self {
        Self {}
    }
}

pub struct LiveTDDijkstraOps();

impl DijkstraOps<LiveTDGraph> for LiveTDDijkstraOps {
    type Label = Weight;
    type LinkResult = Weight;
    type Arc = (NodeIdT, EdgeIdT);
    type PredecessorLink = ();

    #[inline(always)]
    fn link(&mut self, graph: &LiveTDGraph, label: &Weight, link: &Self::Arc) -> Self::LinkResult {
        label + graph.eval(link.1 .0, *label)
    }

    #[inline(always)]
    fn merge(&mut self, label: &mut Weight, linked: Self::LinkResult) -> bool {
        if linked < *label {
            *label = linked;
            return true;
        }
        false
    }

    #[inline(always)]
    fn predecessor_link(&self, _link: &Self::Arc) -> Self::PredecessorLink {
        ()
    }
}

impl Default for LiveTDDijkstraOps {
    fn default() -> Self {
        Self {}
    }
}

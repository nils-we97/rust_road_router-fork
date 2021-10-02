use rust_road_router::algo::GenQuery;
use rust_road_router::datastr::graph::floating_time_dependent::{FlWeight, TTFPoint, Timestamp};
use rust_road_router::datastr::graph::NodeId;
use rust_road_router::datastr::node_order::NodeOrder;

#[derive(Clone, Debug)]
pub struct TDDirectedPartialBackwardProfileQuery {
    pub target: NodeId,
    pub earliest_arrival_lower_bound: Timestamp,
    pub earliest_arrival_upper_bound: Timestamp,
}

impl GenQuery<Vec<TTFPoint>> for TDDirectedPartialBackwardProfileQuery {
    fn new(_from: NodeId, _to: NodeId, _initial_state: Vec<TTFPoint>) -> Self {
        unimplemented!()
    } // not needed

    fn from(&self) -> NodeId {
        self.target
    }

    fn to(&self) -> NodeId {
        unimplemented!()
    }

    fn initial_state(&self) -> Vec<TTFPoint> {
        vec![
            TTFPoint {
                at: self.earliest_arrival_lower_bound,
                val: FlWeight::ZERO,
            },
            TTFPoint {
                at: self.earliest_arrival_upper_bound,
                val: FlWeight::ZERO,
            },
        ]
    }

    fn permutate(&mut self, _order: &NodeOrder) {
        unimplemented!()
    }
}

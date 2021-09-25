use rust_road_router::algo::GenQuery;
use rust_road_router::datastr::graph::floating_time_dependent::{FlWeight, TTFPoint, Timestamp};
use rust_road_router::datastr::graph::NodeId;
use rust_road_router::datastr::node_order::NodeOrder;

#[derive(Clone, Debug)]
pub struct TDPartialBackwardProfileQuery {
    pub target: NodeId,
    pub earliest_arrival: Timestamp,
    pub initial_timeframe: Timestamp,
}

impl GenQuery<Vec<TTFPoint>> for TDPartialBackwardProfileQuery {
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
                at: self.earliest_arrival,
                val: FlWeight::ZERO,
            },
            TTFPoint {
                at: Timestamp(self.earliest_arrival.0 + self.initial_timeframe.0),
                val: FlWeight::ZERO,
            },
        ]
    }

    fn permutate(&mut self, _order: &NodeOrder) {
        unimplemented!()
    }
}

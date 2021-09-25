use rust_road_router::algo::dijkstra::Label;
use rust_road_router::algo::GenQuery;
use rust_road_router::datastr::graph::floating_time_dependent::TTFPoint;
use rust_road_router::datastr::graph::NodeId;
use rust_road_router::datastr::node_order::NodeOrder;

pub struct TDBackwardProfileQuery(pub NodeId);

impl GenQuery<Vec<TTFPoint>> for TDBackwardProfileQuery {
    fn new(_from: NodeId, _to: NodeId, _initial_state: Vec<TTFPoint>) -> Self {
        unimplemented!()
    } // not needed

    fn from(&self) -> NodeId {
        self.0
    }

    fn to(&self) -> NodeId {
        unimplemented!()
    }

    fn initial_state(&self) -> Vec<TTFPoint> {
        Vec::<TTFPoint>::neutral()
    }

    fn permutate(&mut self, _order: &NodeOrder) {
        unimplemented!()
    }
}

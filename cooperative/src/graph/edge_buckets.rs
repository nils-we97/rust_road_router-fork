use rust_road_router::datastr::graph::time_dependent::Timestamp;

use crate::graph::{Capacity, Velocity};

#[derive(Debug, Clone)]
pub enum SpeedBuckets {
    One(Velocity),
    Many(Vec<(Timestamp, Velocity)>),
}

impl SpeedBuckets {
    pub fn many(&mut self) -> &mut Vec<(Timestamp, Velocity)> {
        if let SpeedBuckets::Many(ret) = self {
            ret
        } else {
            panic!("Wrong type: Expected `Many`, found `One`")
        }
    }

    pub fn one(&mut self) -> &mut Velocity {
        if let SpeedBuckets::One(ret) = self {
            ret
        } else {
            panic!("Wrong type: Expected `One`, found `Many`")
        }
    }
}

#[derive(Debug, Clone)]
pub enum CapacityBuckets {
    Unused,
    Used(Vec<(Timestamp, Capacity)>),
}

impl CapacityBuckets {
    pub fn inner(&mut self) -> &mut Vec<(Timestamp, Velocity)> {
        if let CapacityBuckets::Used(ret) = self {
            ret
        } else {
            panic!("Wrong type: cannot retrieve inner value of unused capacity bucket!")
        }
    }
}

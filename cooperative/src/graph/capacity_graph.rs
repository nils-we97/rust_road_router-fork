use conversion::speed_profile_to_tt_profile;
use rust_road_router::datastr::graph::time_dependent::{PiecewiseLinearFunction, Timestamp};
use rust_road_router::datastr::graph::{EdgeId, Graph, NodeId, Weight};

use crate::graph::edge_buckets::{CapacityBuckets, SpeedBuckets};
use crate::graph::{Capacity, ExportableCapacity, ModifiableWeight, Velocity, MAX_BUCKETS};
use rust_road_router::report::measure;
use std::panic;

/// Structure of a time-dependent graph with capacity buckets for each edge
/// After each query, the capacities of all edges on the shortest path get modified
#[derive(Debug)]
pub struct CapacityGraph {
    num_buckets: u32,

    first_out: Vec<EdgeId>,
    head: Vec<NodeId>,

    // dynamic values, subject to change on updates
    used_capacity: Vec<CapacityBuckets>,
    speed: Vec<SpeedBuckets>,
    departure: Vec<Vec<Timestamp>>, //TODO combine departure and travel_time as PLF
    travel_time: Vec<Vec<Weight>>,

    // static values
    distance: Vec<Weight>,
    freeflow_speed: Vec<Weight>,
    max_capacity: Vec<Capacity>,
    freeflow_time: Vec<Weight>,

    speed_function: fn(Velocity, Capacity, Capacity) -> Weight,
}

impl CapacityGraph {
    /// Create a new `CapacityGraph` from the given containers and a weight function
    pub fn new(
        num_buckets: u32,
        first_out: Vec<EdgeId>,
        head: Vec<NodeId>,
        distance: Vec<Weight>,
        freeflow_time: Vec<Weight>,
        max_capacity: Vec<Capacity>, // given in capacity / hour
        speed_function: fn(Velocity, Capacity, Capacity) -> Weight,
    ) -> Self {
        // assert that input parameters are valid!
        assert!(num_buckets > 0 && MAX_BUCKETS % num_buckets == 0); // avoid rounding when accessing buckets!
        assert!(
            first_out.len() > 0 && first_out.len() < <NodeId>::MAX as usize,
            "size of 'first_out' must be in valid range"
        );
        assert!(head.len() > 0 && head.len() < <EdgeId>::MAX as usize, "size of 'head' must be in valid range");
        assert_eq!(first_out.first(), Some(&0), "first element of 'first_out' must be '0'");
        assert_eq!(first_out.last(), Some(&(head.len() as u32)), "last element of 'first_out' must be #edges");
        assert_eq!(distance.len(), head.len(), "data containers must have the same size!");
        assert_eq!(freeflow_time.len(), head.len(), "data containers must have the same size!");
        assert_eq!(max_capacity.len(), head.len(), "data containers must have the same size!");

        // bucket containers with used capacities
        let used_capacity = vec![CapacityBuckets::Unused; max_capacity.len()];

        // adjust capacity of each edge -> more buckets do not allow more traffic flow
        let capacity_adjustment_factor = (num_buckets as f64) / 24.0;
        let max_capacity = max_capacity
            .iter()
            .map(|&capacity| (capacity as f64 * capacity_adjustment_factor) as Capacity)
            .collect::<Vec<Capacity>>();

        // calculate freeflow speed along each edge (assuming distance in meters and time in seconds!)
        let freeflow_speed = freeflow_time
            .iter()
            .zip(distance.iter())
            .map(|(&time_s, &dist_m)| {
                debug_assert!(dist_m > 0 && time_s > 0, "Invalid distance/time values! (must be > 0)");
                (dist_m * 36) / (time_s * 10)
            })
            .collect::<Vec<Velocity>>();

        // container for actual speed, initially equivalent to freeflow speed
        let speed = freeflow_speed.iter().map(|&speed| SpeedBuckets::One(speed)).collect::<Vec<SpeedBuckets>>();

        // adjust initial freeflow time (assuming it is given in seconds and needed in milliseconds!)
        let freeflow_time = freeflow_time.iter().map(|&val| val * 1000).collect::<Vec<Weight>>();

        let departure = vec![Vec::new(); head.len()];
        let travel_time = vec![Vec::new(); head.len()];

        let mut ret = Self {
            num_buckets,
            first_out,
            head,
            used_capacity,
            speed,
            departure,
            travel_time,
            distance,
            freeflow_speed,
            max_capacity,
            freeflow_time,
            speed_function,
        };

        (0..ret.num_arcs()).into_iter().for_each(|i| ret.update_travel_time_profile(i));

        ret
    }

    /// Borrow a slice of `first_out`
    pub fn first_out(&self) -> &[EdgeId] {
        &self.first_out
    }

    /// Borrow a slice of `head`
    pub fn head(&self) -> &[NodeId] {
        &self.head
    }

    /// Borrow a slice of `weight` TODO
    pub fn weight(&self, edge_id: EdgeId, departure: Timestamp) -> Weight {
        self.travel_time_function(edge_id).eval(departure)
    }

    /// Borrow a slice of `departure` TODO
    pub fn departure(&self) -> &Vec<Vec<Timestamp>> {
        &self.departure
    }

    /// Borrow a slice of `travel_time`
    pub fn travel_time(&self) -> &Vec<Vec<Weight>> {
        &self.travel_time
    }

    /// Borrow a slice of `freeflow_time`: useful as lowerbound time for potentials
    pub fn freeflow_time(&self) -> &Vec<Weight> {
        &self.freeflow_time
    }

    /// Borrow an individual travel time function.
    #[inline(always)]
    pub fn travel_time_function(&self, edge_id: EdgeId) -> PiecewiseLinearFunction {
        let edge_id = edge_id as usize;
        PiecewiseLinearFunction::new(&self.departure[edge_id], &self.travel_time[edge_id])
    }

    /// round timestamp to nearest bucket interval
    #[inline(always)]
    fn round_timestamp(&self, timestamp: Timestamp) -> Timestamp {
        let bucket_size = MAX_BUCKETS / self.num_buckets;
        bucket_size * ((timestamp % MAX_BUCKETS) / bucket_size)
    }

    fn update_travel_time_profile(&mut self, edge_id: usize) {
        // TODO improve performance
        // profile contains all points + (max_ts, speed[0])
        let profile = match &self.speed[edge_id] {
            SpeedBuckets::One(speed) => speed_profile_to_tt_profile(&vec![(0, *speed), (MAX_BUCKETS, *speed)], self.distance[edge_id]),
            SpeedBuckets::Many(speeds) => speed_profile_to_tt_profile(speeds, self.distance[edge_id]),
        };

        let (departure, travel_time): (Vec<Timestamp>, Vec<Weight>) = profile.iter().cloned().unzip();

        self.departure[edge_id] = departure;
        self.travel_time[edge_id] = travel_time;
    }
}

impl ModifiableWeight for CapacityGraph {
    fn increase_weights(&mut self, edges: &[EdgeId], departure: &[Timestamp]) -> (time::Duration, time::Duration) {
        let start = time::now();
        edges.iter().zip(departure.iter()).for_each(|(&edge_id, &timestamp)| {
            let edge_id = edge_id as usize;
            let ts_rounded = self.round_timestamp(timestamp);
            let next_ts = (ts_rounded + (MAX_BUCKETS / self.num_buckets)) % MAX_BUCKETS;

            match &mut self.used_capacity[edge_id] {
                CapacityBuckets::Unused => {
                    // edge hasn't been used yet
                    self.used_capacity[edge_id] = CapacityBuckets::Used(vec![(ts_rounded, 1)]);

                    // also update speeds
                    let adjusted_speed = (self.speed_function)(self.freeflow_speed[edge_id], self.max_capacity[edge_id], 1);

                    // this is a bit nasty: depending on the daytime, several speed buckets have to be created
                    if ts_rounded == 0 {
                        // case 1: adjustment at midnight bucket => also consider last sentinel element!
                        self.speed[edge_id] = SpeedBuckets::Many(vec![
                            (0, adjusted_speed),
                            (next_ts, self.freeflow_speed[edge_id]),
                            (MAX_BUCKETS, adjusted_speed),
                        ]);
                    } else if next_ts == 0 {
                        // case 2: next period would be midnight => don't create midnight bucket twice!
                        self.speed[edge_id] = SpeedBuckets::Many(vec![
                            (0, self.freeflow_speed[edge_id]),
                            (ts_rounded, adjusted_speed),
                            (MAX_BUCKETS, self.freeflow_speed[edge_id]),
                        ]);
                    } else {
                        // case 3 (standard): edge gets passed nowhere close to midnight
                        self.speed[edge_id] = SpeedBuckets::Many(vec![
                            (0, self.freeflow_speed[edge_id]),
                            (ts_rounded, adjusted_speed),
                            (next_ts, self.freeflow_speed[edge_id]),
                            (MAX_BUCKETS, self.freeflow_speed[edge_id]),
                        ]);
                    }
                }
                CapacityBuckets::Used(capacity) => {
                    // edge has been used before => update speed buckets!
                    let speed = self.speed[edge_id].many();

                    // check whether the respective bucket already exists
                    let position = capacity.binary_search_by_key(&ts_rounded, |&(ts, _)| ts);
                    if position.is_ok() {
                        // bucket exists => increase capacity by 1 vehicle and update speed
                        // note: neighboring speed buckets can be left untouched because they already exist!
                        let capacity_idx = position.unwrap();

                        capacity[capacity_idx].1 += 1;

                        let speed_idx = speed.binary_search_by_key(&ts_rounded, |&(ts, _)| ts).ok().unwrap();
                        speed[speed_idx].1 = (self.speed_function)(self.freeflow_speed[edge_id], self.max_capacity[edge_id], capacity[capacity_idx].1);

                        // if the change occurs at midnight, then also update the sentinel element
                        if speed_idx == 0 {
                            let last_idx = speed.len() - 1;
                            speed[last_idx].1 = speed[0].1;
                        }
                    } else {
                        // no traffic flow at `ts_rounded` yet => insert capacity
                        let capacity_idx = position.unwrap_err();
                        capacity.insert(capacity_idx, (ts_rounded, 1));

                        // update speed - careful: bucket may already exist!
                        let speed_pos = speed.binary_search_by_key(&ts_rounded, |&(ts, _)| ts);
                        let speed_idx: usize;

                        if speed_pos.is_ok() {
                            // case 1: speed bucket already exists -> update
                            speed_idx = speed_pos.unwrap();
                            speed[speed_idx] = (ts_rounded, (self.speed_function)(self.freeflow_speed[edge_id], self.max_capacity[edge_id], 1));

                            // if the change occurs at midnight, then also update the sentinel element
                            if speed_idx == 0 {
                                let last_idx = speed.len() - 1;
                                speed[last_idx].1 = speed[0].1;
                            }
                        } else {
                            // case 2: speed bucket did not exist yet -> insert
                            speed_idx = speed_pos.unwrap_err();
                            speed.insert(
                                speed_idx,
                                (ts_rounded, (self.speed_function)(self.freeflow_speed[edge_id], self.max_capacity[edge_id], 1)),
                            );
                        }

                        // additionally, it is required to check whether the neighboring bucket already exists
                        // if this bucket does not exist already, the TTF algorithm would falsely
                        // assume that the speed is the same within the next bucket(s)
                        if next_ts != 0 && speed[speed_idx + 1].0 != next_ts {
                            // only update if the next existing bucket ts != `next_ts`
                            speed.insert(speed_idx + 1, (next_ts, self.freeflow_speed[edge_id]));
                        }
                    }
                }
            }
        });
        let time_buckets = time::now() - start;

        let (_, time_ttf) = measure(|| edges.iter().for_each(|&edge_id| self.update_travel_time_profile(edge_id as usize)));
        //let (_, time_ttf_update) = measure(|| self.update_travel_time_profile(edge_id));

        (time_buckets, time_ttf)
    }

    fn reset_weights(&mut self) {
        for edge_id in 0..self.num_arcs() {
            self.used_capacity[edge_id] = CapacityBuckets::Unused;
            self.speed[edge_id] = SpeedBuckets::One(self.freeflow_speed[edge_id]);
            self.update_travel_time_profile(edge_id);
        }
    }
}

impl ExportableCapacity for CapacityGraph {
    fn export_capacities(&self) -> Vec<Vec<(u32, u32)>> {
        self.used_capacity
            .iter()
            .map(|bucket| match bucket {
                CapacityBuckets::Unused => Vec::new(),
                CapacityBuckets::Used(inner) => inner.clone(),
            })
            .collect::<Vec<Vec<(Timestamp, Capacity)>>>()
    }

    /// update: adjust capacity buckets, update speed buckets and TTFs
    fn update_capacities(&mut self, capacities: Vec<Vec<(u32, u32)>>) {
        assert_eq!(capacities.len(), self.num_arcs(), "Failed to provide a capacity bucket for each edge.");

        capacities.iter().enumerate().for_each(|(edge_id, capacity)| {
            // case 1: bucket is empty
            if capacity.is_empty() {
                self.used_capacity[edge_id] = CapacityBuckets::Unused;
            } else {
                self.used_capacity[edge_id] = CapacityBuckets::Used(capacity.clone());
            }
        })
        // TODO update speeds too
    }
}

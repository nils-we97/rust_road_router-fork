use conversion::speed_profile_to_tt_profile;
use rust_road_router::datastr::graph::time_dependent::{PiecewiseLinearFunction, Timestamp};
use rust_road_router::datastr::graph::{EdgeId, Graph, NodeId, Weight};

use crate::graph::edge_buckets::{CapacityBuckets, SpeedBuckets};
use crate::graph::{travel_time, velocity, Capacity, ExportableCapacity, ModifiableWeight, Velocity, MAX_BUCKETS};
use rust_road_router::report::measure;

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
        let capacity_adjustment_factor = 24.0 / (num_buckets as f64);
        let max_capacity = max_capacity
            .iter()
            .map(|&capacity| (capacity as f64 * capacity_adjustment_factor) as Capacity)
            .collect::<Vec<Capacity>>();

        // calculate freeflow speed along each edge (assuming distance in meters and time in seconds!)
        let freeflow_speed = freeflow_time
            .iter()
            .zip(distance.iter())
            .map(|(&time_s, &len_m)| velocity(len_m, time_s))
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

    /// Borrow a slice of `departure` TODO
    pub fn departure(&self) -> &Vec<Vec<Timestamp>> {
        &self.departure
    }

    /// Borrow a slice of `travel_time`
    pub fn travel_time(&self) -> &Vec<Vec<Weight>> {
        &self.travel_time
    }

    /// Borrow a slice of `distance`
    pub fn distance(&self) -> &Vec<Weight> {
        &self.distance
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
        match &self.speed[edge_id] {
            SpeedBuckets::One(speed) => {
                // avoid expensive calculations for the easy case, TTF is trivial here :)
                let travel_time = travel_time(*speed, self.distance[edge_id]);
                debug_assert!(travel_time < MAX_BUCKETS, "travel time must not exceed 24 hours!");

                self.departure[edge_id] = vec![0, MAX_BUCKETS];
                self.travel_time[edge_id] = vec![travel_time, travel_time];
            }
            SpeedBuckets::Many(speed_profile) => {
                // for multiple entries, a more complex travel time profile is required
                // TODO avoid updating the whole profile
                let tt_profile = speed_profile_to_tt_profile(speed_profile, self.distance[edge_id]);
                let (departure, travel_time): (Vec<Timestamp>, Vec<Weight>) = tt_profile.iter().cloned().unzip();

                self.departure[edge_id] = departure;
                self.travel_time[edge_id] = travel_time;
            }
        }
    }
}

impl ModifiableWeight for CapacityGraph {
    fn increase_weights(&mut self, edges: &[EdgeId], departure: &[Timestamp]) -> (time::Duration, time::Duration) {
        let start = time::now();
        edges.iter().zip(departure.iter()).for_each(|(&edge_id, &timestamp)| {
            let edge_id = edge_id as usize;

            if self.num_buckets == 1 {
                // special case treatment: graph has only one capacity/speed bucket per edge
                // in this case, some edge cases can be avoided, thus making the updates more efficient
                let prev_capacity = self.used_capacity[edge_id].get(0).unwrap_or(0);
                self.used_capacity[edge_id] = CapacityBuckets::Used(vec![(0, prev_capacity + 1)]);

                let adjusted_speed = (self.speed_function)(self.freeflow_speed[edge_id], self.max_capacity[edge_id], prev_capacity + 1);
                self.speed[edge_id] = SpeedBuckets::One(adjusted_speed);
            } else {
                let ts_rounded = self.round_timestamp(timestamp);
                let next_ts = (ts_rounded + (MAX_BUCKETS / self.num_buckets)) % MAX_BUCKETS;

                // update capacity
                self.used_capacity[edge_id].increment(ts_rounded);

                // update velocity: get adjusted speed along segment
                let updated_capacity = self.used_capacity[edge_id].get(ts_rounded);
                debug_assert!(updated_capacity.is_some());
                let adjusted_speed = (self.speed_function)(self.freeflow_speed[edge_id], self.max_capacity[edge_id], updated_capacity.unwrap());

                // optimization: only update if the adjusted speed differs from the freeflow speed!
                // this reduces the number of TTF breakpoints and thus improves the overall runtime significantly
                if adjusted_speed != self.freeflow_speed[edge_id] {
                    self.speed[edge_id].update(ts_rounded, adjusted_speed, next_ts, self.freeflow_speed[edge_id]);
                }
            }
        });
        let time_buckets = time::now() - start;

        let (_, time_ttf) = measure(|| edges.iter().for_each(|&edge_id| self.update_travel_time_profile(edge_id as usize)));

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
            debug_assert!(
                capacity.len() as u32 <= self.num_buckets,
                "Invalid number of buckets detected ({}, max. allowed: {})",
                capacity.len(),
                self.num_buckets
            );
            debug_assert!((*capacity.last().unwrap_or(&(0, 0))).0 < MAX_BUCKETS, "Sentinel element must not be present!");

            // update capacities and speeds
            if capacity.is_empty() {
                self.used_capacity[edge_id] = CapacityBuckets::Unused;
                self.speed[edge_id] = SpeedBuckets::One(self.freeflow_speed[edge_id]);
            } else {
                self.used_capacity[edge_id] = CapacityBuckets::Used(capacity.clone());

                // create speed profile, special case treatment for single-bucket graphs
                if self.num_buckets == 1 {
                    let velocity = (self.speed_function)(self.freeflow_speed[edge_id], self.max_capacity[edge_id], capacity[0].1);
                    self.speed[edge_id] = SpeedBuckets::One(velocity);
                } else {
                    let step_ts = MAX_BUCKETS / self.num_buckets;
                    let mut speed_profile = Vec::new();

                    // initial breakpoint
                    if capacity[0].0 == 0 {
                        let adjusted_speed = (self.speed_function)(self.freeflow_speed[edge_id], self.max_capacity[edge_id], capacity[0].1);
                        speed_profile.push((0, adjusted_speed));

                        if adjusted_speed != self.freeflow_speed[edge_id] {
                            speed_profile.push((step_ts, self.freeflow_speed[edge_id]));
                        }
                    } else {
                        speed_profile.push((0, self.freeflow_speed[edge_id]));
                    }

                    // for all following points, just iterate over the remaining capacity items
                    capacity[1..].iter().for_each(|&(bucket_ts, bucket_capacity)| {
                        // only extend speed function if new speed differs from freeflow speed!
                        // otherwise, this just blows up the travel time function
                        let adjusted_speed = (self.speed_function)(self.freeflow_speed[edge_id], self.max_capacity[edge_id], bucket_capacity);
                        if adjusted_speed != self.freeflow_speed[edge_id] {
                            if (*speed_profile.last().unwrap()).0 == bucket_ts {
                                // adjust last entry
                                (*speed_profile.last_mut().unwrap()).1 = adjusted_speed;
                            } else {
                                // push new entry
                                speed_profile.push((bucket_ts, adjusted_speed));
                            }

                            // add additional breakpoint at next bucket
                            if bucket_ts + step_ts < MAX_BUCKETS {
                                speed_profile.push((bucket_ts + step_ts, self.freeflow_speed[edge_id]));
                            }
                        }
                    });

                    // add sentinel element for last breakpoint
                    speed_profile.push((MAX_BUCKETS, speed_profile[0].1));

                    // update speed profile with as little space consumption if possible,
                    // i.e. if the constructed function is equal to the constant freeflow speed,
                    // then use the shortcut version of `One`
                    self.speed[edge_id] = if speed_profile.len() == 2 && speed_profile[0].1 == self.freeflow_speed[edge_id] {
                        SpeedBuckets::One(speed_profile[0].1)
                    } else {
                        SpeedBuckets::Many(speed_profile)
                    };
                }
            }

            // update TTF
            self.update_travel_time_profile(edge_id);
        });
    }
}

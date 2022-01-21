use rust_road_router::datastr::graph::time_dependent::{PiecewiseLinearFunction, Timestamp};
use rust_road_router::datastr::graph::{EdgeId, Graph, NodeId, Weight, INFINITY};

use crate::graph::edge_buckets::CapacityBuckets;
use crate::graph::traffic_functions::BPRTrafficFunction;
use crate::graph::{Capacity, ExportableCapacity, ModifiableWeight, MAX_BUCKETS};
use conversion::speed_profile_to_tt_profile;
use std::cmp::{max, min};

/// Structure of a time-dependent graph with capacity buckets for each edge
/// After each query, the capacities of all edges on the shortest path get modified
#[derive(Debug)]
pub struct CapacityGraph {
    num_buckets: u32,

    // graph structure
    first_out: Vec<EdgeId>,
    head: Vec<NodeId>,

    // dynamic values, subject to change on updates
    used_capacity: Vec<CapacityBuckets>,
    departure: Vec<Vec<Timestamp>>,
    travel_time: Vec<Vec<Weight>>,

    // static values
    distance: Vec<Weight>,
    max_capacity: Vec<Capacity>,
    free_flow_travel_time: Vec<Weight>,
    free_flow_speed_kmh: Vec<Weight>,

    traffic_function: BPRTrafficFunction,
}

impl CapacityGraph {
    /// Create a new `CapacityGraph` from the given containers and a weight function
    pub fn new(
        num_buckets: u32,
        first_out: Vec<EdgeId>,
        head: Vec<NodeId>,
        distance: Vec<Weight>,
        free_flow_travel_time: Vec<Weight>,
        max_capacity: Vec<Capacity>, // given in capacity / hour
        traffic_function: BPRTrafficFunction,
    ) -> Self {
        // assert that input parameters are valid
        assert!(num_buckets > 0 && MAX_BUCKETS % num_buckets == 0); // avoid rounding when accessing buckets!
        assert!(
            first_out.len() > 0 && first_out.len() < <NodeId>::MAX as usize,
            "size of 'first_out' must be in valid range"
        );
        assert!(head.len() > 0 && head.len() < <EdgeId>::MAX as usize, "size of 'head' must be in valid range");
        assert_eq!(first_out.first(), Some(&0), "first element of 'first_out' must be '0'");
        assert_eq!(first_out.last(), Some(&(head.len() as u32)), "last element of 'first_out' must be #edges");
        assert_eq!(distance.len(), head.len(), "data containers must have the same size!");
        assert_eq!(free_flow_travel_time.len(), head.len(), "data containers must have the same size!");
        assert_eq!(max_capacity.len(), head.len(), "data containers must have the same size!");

        // adjust capacity of each edge -> more buckets do not allow more traffic flow
        let capacity_adjustment_factor = 24.0 / (num_buckets as f64);
        let max_capacity = max_capacity
            .iter()
            .map(|&capacity| {
                // avoid unnecessary edges
                if capacity >= 50 {
                    (capacity as f64 * capacity_adjustment_factor) as Capacity
                } else {
                    0
                }
            })
            .collect::<Vec<Capacity>>();

        // initialize free-flow speed
        // fallback to speed 1 if capacity or time are invalid
        let free_flow_speed_kmh = (0..head.len())
            .into_iter()
            .map(|idx| {
                if free_flow_travel_time[idx] == 1 || free_flow_travel_time[idx] >= INFINITY || max_capacity[idx] == 0 {
                    1
                } else {
                    (3600 * distance[idx]) / free_flow_travel_time[idx]
                }
            })
            .collect::<Vec<Weight>>();

        let free_flow_travel_time = (0..head.len())
            .into_iter()
            .map(|idx| {
                if free_flow_travel_time[idx] >= INFINITY || max_capacity[idx] == 0 {
                    INFINITY
                } else if free_flow_travel_time[idx] == 1 {
                    1
                } else {
                    3600 * distance[idx] / free_flow_speed_kmh[idx]
                }
            })
            .collect::<Vec<u32>>();

        assert!(!free_flow_travel_time.iter().any(|&x| x > INFINITY));

        // initialize bucket containers as well as departure and travel_time structs
        let used_capacity = vec![CapacityBuckets::Unused; max_capacity.len()];

        let departure = vec![vec![0, MAX_BUCKETS]; head.len()];
        let travel_time = (0..head.len())
            .into_iter()
            .map(|i| {
                if max_capacity[i] == 0 {
                    vec![INFINITY, INFINITY]
                } else {
                    vec![free_flow_travel_time[i], free_flow_travel_time[i]]
                }
            })
            .collect::<Vec<Vec<Weight>>>();

        Self {
            num_buckets,
            first_out,
            head,
            used_capacity,
            departure,
            travel_time,
            distance,
            free_flow_speed_kmh,
            max_capacity,
            free_flow_travel_time,
            traffic_function,
        }
    }

    /// Borrow a slice of `first_out`
    pub fn first_out(&self) -> &[EdgeId] {
        &self.first_out
    }

    /// Borrow a slice of `head`
    pub fn head(&self) -> &[NodeId] {
        &self.head
    }

    /// Borrow a slice of `departure`
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

    /// Borrow a slice of `max_capacity`
    pub fn max_capacity(&self) -> &Vec<Weight> {
        &self.max_capacity
    }

    /// Borrow a slice of `free_flow_time`: useful as lower bound time for potentials
    pub fn free_flow_time(&self) -> &Vec<Weight> {
        &self.free_flow_travel_time
    }

    /// Borrow an individual travel time function.
    #[inline(always)]
    pub fn travel_time_function(&self, edge_id: EdgeId) -> PiecewiseLinearFunction {
        let edge_id = edge_id as usize;
        PiecewiseLinearFunction::new(&self.departure[edge_id], &self.travel_time[edge_id])
    }

    /// estimate memory consumption
    pub fn get_mem_size(&self) -> usize {
        // static graph data: first_out, head, distance, max-capacity and freeflow time
        let static_graph_size = 4
            * (self.first_out.capacity()
                + self.head.capacity()
                + self.distance.capacity()
                + self.max_capacity.capacity()
                + self.free_flow_speed_kmh.capacity()
                + self.free_flow_travel_time.capacity());

        let capacity_bucket_size = self
            .used_capacity
            .iter()
            .map(|buckets| match buckets {
                CapacityBuckets::Unused => 0,
                CapacityBuckets::Used(data) => data.capacity() * 8,
            })
            .sum::<usize>();

        let ttf_size = self
            .departure
            .iter()
            .zip(self.travel_time.iter())
            .map(|(dep, tt)| dep.capacity() * 4 + tt.capacity() * 4)
            .sum::<usize>();

        static_graph_size + capacity_bucket_size + ttf_size
    }

    /// get the number of used buckets
    pub fn get_bucket_usage(&self) -> usize {
        self.used_capacity
            .iter()
            .map(|buckets| match buckets {
                CapacityBuckets::Unused => 0,
                CapacityBuckets::Used(data) => data.len(),
            })
            .sum()
    }

    pub fn num_buckets(&self) -> u32 {
        self.num_buckets
    }

    /// round timestamp to nearest bucket interval
    #[inline(always)]
    fn round_timestamp(&self, timestamp: Timestamp) -> Timestamp {
        let bucket_size = MAX_BUCKETS / self.num_buckets;
        bucket_size * ((timestamp % MAX_BUCKETS) / bucket_size)
    }

    fn rebuild_travel_time_profile(&mut self, edge_id: usize) {
        if self.max_capacity[edge_id] == 0 {
            // zero-capacity edges must not be traversed at all!
            debug_assert_eq!(self.departure[edge_id].len(), 2);
            debug_assert_eq!(min(self.travel_time[edge_id][0], self.travel_time[edge_id][1]), INFINITY);
        } else if !self.used_capacity[edge_id].is_used() || self.free_flow_travel_time[edge_id] < 100 {
            // unused edges can be kept at their free-flow time
            // dummy edges with extremely short distances (e.g. at junctions) won't be penalized either
            debug_assert_eq!(self.departure[edge_id].len(), 2);
            debug_assert_eq!(
                max(self.travel_time[edge_id][0], self.travel_time[edge_id][1]),
                self.free_flow_travel_time[edge_id]
            );
        } else if self.num_buckets == 1 {
            // special-case treatment for single-bucket graphs -> updating the capacities and ttf is straightforward
            let travel_time = self.traffic_function.travel_time(
                self.free_flow_travel_time[edge_id],
                self.max_capacity[edge_id],
                self.used_capacity[edge_id].inner()[0].1,
            );

            self.departure[edge_id] = vec![0, MAX_BUCKETS];
            self.travel_time[edge_id] = vec![travel_time, travel_time];
        } else {
            let free_flow_speed = self.free_flow_speed_kmh[edge_id];
            let max_capacity = self.max_capacity[edge_id];

            let bucket_size = MAX_BUCKETS / self.num_buckets;

            let used_capacities = self.used_capacity[edge_id].inner();
            let (first_capacity_ts, first_capacity_val) = used_capacities[0];

            // calculate speeds, convert them into ttf
            let mut speeds = Vec::new();

            // special-case treatment for first entry
            let start_index = if first_capacity_ts == 0 {
                speeds.push((0, self.traffic_function.speed(free_flow_speed, max_capacity, first_capacity_val)));
                1
            } else {
                speeds.push((0, free_flow_speed));
                0
            };

            // regular procedure for all but the sentinel entry
            for i in start_index..used_capacities.len() {
                let (current_ts, current_val) = used_capacities[i];
                let current_speed = self.traffic_function.speed(free_flow_speed, max_capacity, current_val);
                let (last_ts, _last_speed) = *speeds.last().unwrap();

                // check if we have to prepend a free flow entry
                if current_ts - last_ts > bucket_size {
                    speeds.push((last_ts + bucket_size, free_flow_speed));
                }

                speeds.push((current_ts, current_speed));
            }

            // special case treatment for last entry: check if we have to prepend an entry, then add sentinel
            let (last_ts, _last_speed) = *speeds.last().unwrap();
            let (_, first_speed) = speeds[0];

            if
            /*last_speed != first_speed &&*/
            last_ts + bucket_size < MAX_BUCKETS {
                speeds.push((last_ts + bucket_size, free_flow_speed));
            }
            speeds.push((MAX_BUCKETS, first_speed));

            // convert speed to tt profile
            let (departure, travel_time) = speed_profile_to_tt_profile(&speeds, self.distance[edge_id]).iter().cloned().unzip();
            self.departure[edge_id] = departure;
            self.travel_time[edge_id] = travel_time;
        }
    }
}

impl ModifiableWeight for CapacityGraph {
    fn increase_weights(&mut self, edges: &[EdgeId], departure: &[Timestamp]) {
        edges.iter().zip(departure.iter()).for_each(|(&edge_id, &timestamp)| {
            let edge_id = edge_id as usize;

            if self.num_buckets == 1 {
                // special case treatment for single-bucket graph
                let prev_capacity = match &self.used_capacity[edge_id] {
                    CapacityBuckets::Unused => 0,
                    CapacityBuckets::Used(data) => {
                        debug_assert!(data.len() == 1 && data[0].0 == 0);
                        data[0].1
                    }
                };

                self.used_capacity[edge_id] = CapacityBuckets::Used(vec![(0, prev_capacity + 1)]);
            } else {
                // find suitable bucket in which to insert, then update capacity
                let ts_rounded = self.round_timestamp(timestamp);
                self.used_capacity[edge_id].increment(ts_rounded);
            }
            self.rebuild_travel_time_profile(edge_id);
        });
    }

    fn reset_weights(&mut self) {
        for edge_id in 0..self.num_arcs() {
            self.used_capacity[edge_id] = CapacityBuckets::Unused;
            self.departure[edge_id] = vec![0, MAX_BUCKETS];
            self.travel_time[edge_id] = vec![self.free_flow_travel_time[edge_id], self.free_flow_travel_time[edge_id]];
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

            self.used_capacity[edge_id] = if capacity.is_empty() {
                CapacityBuckets::Unused
            } else {
                CapacityBuckets::Used(capacity.clone())
            };
            self.rebuild_travel_time_profile(edge_id);
        });
    }
}

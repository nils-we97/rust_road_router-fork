use rust_road_router::datastr::graph::time_dependent::{PiecewiseLinearFunction, Timestamp};
use rust_road_router::datastr::graph::{EdgeId, Graph, NodeId, Weight, INFINITY};

use crate::graph::edge_buckets::CapacityBuckets;
use crate::graph::travel_time_function::{build_ttf, update_ttf};
use crate::graph::{Capacity, ExportableCapacity, ModifiableWeight, MAX_BUCKETS};

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

    traffic_function: fn(Weight, Capacity, Capacity) -> Weight,
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
        traffic_function: fn(Weight, Capacity, Capacity) -> Weight,
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
                if capacity >= 100 {
                    (capacity as f64 * capacity_adjustment_factor) as Capacity
                } else {
                    0
                }
            })
            .collect::<Vec<Capacity>>();

        // adjust initial freeflow time
        let free_flow_travel_time = free_flow_travel_time
            .iter()
            .enumerate()
            .map(|(idx, &val)| if max_capacity[idx] > 0 { val } else { INFINITY })
            .collect::<Vec<Weight>>();

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

    /// get memory consumption
    pub fn get_mem_size(&self) -> usize {
        // static graph data: first_out, head, distance, max-capacity and freeflow time
        let static_graph_size = std::mem::size_of_val(&*self.first_out)
            + std::mem::size_of_val(&*self.head)
            + std::mem::size_of_val(&*self.distance)
            + std::mem::size_of_val(&*self.max_capacity)
            + std::mem::size_of_val(&*self.free_flow_travel_time);

        let capacity_bucket_size = self
            .used_capacity
            .iter()
            .map(|buckets| match buckets {
                CapacityBuckets::Unused => std::mem::size_of_val(&CapacityBuckets::Unused),
                CapacityBuckets::Used(data) => std::mem::size_of_val(&*buckets) + std::mem::size_of_val(&*data),
            })
            .sum::<usize>();

        let ttf_size = self
            .departure
            .iter()
            .zip(self.travel_time.iter())
            .map(|(dep, tt)| std::mem::size_of_val(&*dep) + std::mem::size_of_val(&*tt))
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
            self.departure[edge_id] = vec![0, MAX_BUCKETS];
            self.travel_time[edge_id] = vec![INFINITY, INFINITY];
        } else if !self.used_capacity[edge_id].is_used() {
            self.departure[edge_id] = vec![0, MAX_BUCKETS];
            self.travel_time[edge_id] = vec![self.free_flow_travel_time[edge_id], self.free_flow_travel_time[edge_id]];
        } else if self.num_buckets == 1 {
            let travel_time = (self.traffic_function)(
                self.free_flow_travel_time[edge_id],
                self.max_capacity[edge_id],
                self.used_capacity[edge_id].inner()[0].1,
            );

            self.departure[edge_id] = vec![0, MAX_BUCKETS];
            self.travel_time[edge_id] = vec![travel_time, travel_time];
        } else {
            let free_flow_time = self.free_flow_travel_time[edge_id];
            let max_capacity = self.max_capacity[edge_id];
            let (first_capacity_ts, first_capacity_val) = self.used_capacity[edge_id].inner()[0];

            let (departure, mut travel_time): (Vec<u32>, Vec<u32>) = if first_capacity_ts == 0 {
                // first entry starts with 0 -> simply overwrite this value
                self.used_capacity[edge_id]
                    .inner()
                    .iter()
                    .chain([(MAX_BUCKETS, first_capacity_val)].iter())
                    .map(|&(timestamp, used_capacity)| (timestamp, (self.traffic_function)(free_flow_time, max_capacity, used_capacity)))
                    .unzip()
            } else {
                // first value does not start with 0 -> add additional element;
                [(0, 0)]
                    .iter()
                    .chain(self.used_capacity[edge_id].inner().iter())
                    .chain(vec![(MAX_BUCKETS, 0)].iter())
                    .map(|&(timestamp, used_capacity)| (timestamp, (self.traffic_function)(free_flow_time, max_capacity, used_capacity)))
                    .unzip()
            };

            let any_infty = travel_time.iter().any(|&x| x == 2147483647);
            if any_infty {
                println!(
                    "Infinity entry found! {:?} (capacities: {:?}), edge: {}, freeflow {}, dist {}, max capacity: {}",
                    travel_time,
                    self.used_capacity[edge_id].inner(),
                    edge_id,
                    self.free_flow_travel_time[edge_id],
                    self.distance[edge_id],
                    self.max_capacity[edge_id]
                );
            }

            // preserve fifo property
            build_ttf(&departure, &mut travel_time);

            if any_infty {
                println!("Updated travel time: {:?}", travel_time);
            }

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

                // no adjustment on departures needed (they will remain constant!)
                // travel time profile update is straightforward
                let travel_time = (self.traffic_function)(self.free_flow_travel_time[edge_id], self.max_capacity[edge_id], prev_capacity + 1);
                debug_assert!(self.travel_time[edge_id][0] <= travel_time);
                self.travel_time[edge_id][0] = travel_time;
                self.travel_time[edge_id][1] = travel_time;
            } else {
                // find suitable bucket in which to insert
                let ts_rounded = self.round_timestamp(timestamp);
                let free_flow_time = self.free_flow_travel_time[edge_id];
                let max_capacity = self.max_capacity[edge_id];

                // update capacity
                let (is_new_bucket, pos) = self.used_capacity[edge_id].increment(ts_rounded);

                if is_new_bucket {
                    // new bucket container has been added -> recreate entire function
                    self.rebuild_travel_time_profile(edge_id);
                } else {
                    // a single container has been updated -> update the respective entry and fix the function if needed
                    let edge_capacities = self.used_capacity[edge_id].inner();
                    let updated_capacity_val = edge_capacities[pos].1;

                    let tt_pos = pos + (edge_capacities[0].0 > 0) as usize;
                    self.travel_time[edge_id][tt_pos] = (self.traffic_function)(free_flow_time, max_capacity, updated_capacity_val);
                    if tt_pos == 0 {
                        *self.travel_time[edge_id].last_mut().unwrap() = self.travel_time[edge_id][0];
                    }

                    // preserve fifo property
                    update_ttf(&self.departure[edge_id], &mut self.travel_time[edge_id], tt_pos, MAX_BUCKETS);
                }
            }
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

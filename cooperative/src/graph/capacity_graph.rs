use std::ops::Range;

use conversion::speed_profile_to_tt_profile;
use rust_road_router::datastr::graph::time_dependent::{PiecewiseLinearFunction, Timestamp};
use rust_road_router::datastr::graph::{EdgeId, EdgeIdGraph, EdgeIdT, EdgeRandomAccessGraph, Graph, Link, LinkIterable, NodeId, NodeIdT, Weight};

use crate::graph::{Capacity, ModifiableWeight, Velocity, MAX_BUCKETS};
use std::panic;

/// Structure of a time-dependent graph with capacity buckets for each edge
/// After each query, the capacities of all edges on the shortest path get modified
#[derive(Debug)]
pub struct CapacityGraph {
    num_buckets: u32,

    first_out: Vec<EdgeId>,
    head: Vec<NodeId>,

    // dynamic values, subject to change on updates
    used_capacity: Vec<Vec<(Timestamp, Weight)>>,
    speed: Vec<Vec<(Timestamp, Velocity)>>,
    departure: Vec<Vec<Timestamp>>, //TODO combine departure and travel_time as PLF
    travel_time: Vec<Vec<Weight>>,

    // static values
    distance: Vec<Weight>,
    freeflow_speed: Vec<Weight>,
    max_capacity: Vec<Capacity>,
    lowerbound_time: Vec<Weight>,

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
        let used_capacity = vec![vec![(0, 0)]; max_capacity.len()];

        // adjust capacity of each edge such that more buckets do not allow more traffic flow
        let capacity_adjustment_factor = (num_buckets as f64) / 24.0;

        let max_capacity = max_capacity
            .iter()
            .map(|&capacity| (capacity as f64 * capacity_adjustment_factor) as Capacity)
            .collect::<Vec<Capacity>>();

        // calculate freeflow speed along each edge
        let freeflow_speed = freeflow_time
            .iter()
            .zip(distance.iter())
            .map(|(&time, &dist)| convert_to_velocity(dist, time))
            .collect::<Vec<Velocity>>();

        // lowertime bounds, required for potential calculation
        let lowerbound_time = freeflow_speed
            .iter()
            .zip(distance.iter())
            .map(|(&velocity, &distance)| {
                let speeds = [(0, velocity), (MAX_BUCKETS, velocity)];
                let tt_profile = speed_profile_to_tt_profile(&speeds, distance);
                tt_profile.first().map(|&(_, time)| time).unwrap_or(1)
            })
            .collect::<Vec<Weight>>();

        // container for actual speed, initially equivalent to freeflow speed
        let speed = freeflow_speed
            .iter()
            .map(|&speed| vec![(0, speed), (MAX_BUCKETS, speed)])
            .collect::<Vec<Vec<(Timestamp, Velocity)>>>();

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
            lowerbound_time,
            speed_function,
        };

        (0..ret.num_arcs()).into_iter().for_each(|i| ret.update_travel_time_profile(i));

        ret
    }

    /// Borrow an individual travel time function.
    #[inline(always)]
    pub fn travel_time_function(&self, edge_id: EdgeId) -> PiecewiseLinearFunction {
        let edge_id = edge_id as usize;
        PiecewiseLinearFunction::new(&self.departure[edge_id], &self.travel_time[edge_id])
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

    /// round timestamp to nearest bucket interval
    #[inline(always)]
    fn round_timestamp(&self, timestamp: Timestamp) -> Timestamp {
        let bucket_size = MAX_BUCKETS / self.num_buckets;
        bucket_size * ((timestamp % MAX_BUCKETS) / bucket_size)
    }

    fn update_travel_time_profile(&mut self, edge_id: usize) {
        // profile contains all points + (max_ts, speed[0])
        let profile = speed_profile_to_tt_profile(&self.speed[edge_id], self.distance[edge_id]);

        self.departure[edge_id] = Vec::with_capacity(profile.len());
        self.travel_time[edge_id] = Vec::with_capacity(profile.len());

        profile.iter().cloned().for_each(|(timestamp, weight)| {
            self.departure[edge_id].push(timestamp);
            self.travel_time[edge_id].push(weight);
        });
    }
}

impl Graph for CapacityGraph {
    fn num_nodes(&self) -> usize {
        self.first_out.len() - 1
    }

    fn num_arcs(&self) -> usize {
        self.head.len()
    }

    fn degree(&self, node: NodeId) -> usize {
        let node = node as usize;
        assert!(node < self.num_nodes());
        (self.first_out[node + 1] - self.first_out[node]) as usize
    }
}

impl ModifiableWeight for CapacityGraph {
    fn increase_weights(&mut self, path: &[(EdgeId, Timestamp)]) {
        path.iter().cloned().for_each(|(edge_id, timestamp)| {
            let edge_id = edge_id as usize;
            let ts_rounded = self.round_timestamp(timestamp);

            // check whether the respective bucket already exists
            let position = self.used_capacity[edge_id].binary_search_by_key(&ts_rounded, |&(ts, _)| ts);

            if position.is_ok() {
                // bucket exists -> increase capacity by 1 vehicle
                let pos = position.unwrap();
                self.used_capacity[edge_id][pos].1 += 1;
                self.speed[edge_id][pos].1 =
                    (self.speed_function)(self.freeflow_speed[edge_id], self.max_capacity[edge_id], self.used_capacity[edge_id][pos].1);
            } else {
                // bucket does not exist yet -> create a new bucket
                let pos = position.unwrap_err();

                self.used_capacity[edge_id].insert(pos, (ts_rounded, 1));
                self.speed[edge_id].insert(
                    pos,
                    (ts_rounded, (self.speed_function)(self.freeflow_speed[edge_id], self.max_capacity[edge_id], 1)),
                );

                // additionally, we have to check whether the neighboring bucket already exists
                // if so, there is nothing to do
                // else, we have to add an additional bucket with capacity 0 -> otherwise, the algorithm
                // falsely assumes that the speed in the next interval is equal to the current interval
                let next_ts = (ts_rounded + (MAX_BUCKETS / self.num_buckets)) % MAX_BUCKETS;

                // access with speed array as it has a sentinel element at the end
                if next_ts != 0 && self.speed[edge_id][pos + 1].0 != next_ts {
                    self.used_capacity[edge_id].insert(pos + 1, (next_ts, 0));
                    self.speed[edge_id].insert(pos + 1, (next_ts, self.freeflow_speed[edge_id]));
                }
            }

            let last_idx = self.speed[edge_id].len() - 1;

            self.speed[edge_id][last_idx].1 = self.speed[edge_id][0].1;
            self.update_travel_time_profile(edge_id);
        });
    }

    fn decrease_weights(&mut self, path: &[(EdgeId, Timestamp)]) {
        path.iter().cloned().for_each(|(edge_id, timestamp)| {
            let edge_id = edge_id as usize;
            let ts_rounded = self.round_timestamp(timestamp);

            let position = self.used_capacity[edge_id].binary_search_by_key(&ts_rounded, |&(ts, _)| ts);

            if position.is_ok() {
                let pos = position.unwrap();
                self.used_capacity[edge_id][pos].1 -= 1;
                self.speed[edge_id][pos].1 =
                    (self.speed_function)(self.freeflow_speed[edge_id], self.max_capacity[edge_id], self.used_capacity[edge_id][pos].1);
            } else {
                panic!("Trying to decrease capacity of a zero-capacity bucket!")
            }

            self.update_travel_time_profile(edge_id);
        });
    }

    fn reset_weights(&mut self) {
        for edge_id in 0..self.num_arcs() {
            self.used_capacity[edge_id] = vec![(0, 0)];
            self.speed[edge_id] = vec![(0, self.freeflow_speed[edge_id]), (MAX_BUCKETS, self.freeflow_speed[edge_id])];
            self.update_travel_time_profile(edge_id);
        }
    }
}

/// trait needed for `EdgeRandomAccessGraph` trait below
impl EdgeIdGraph for CapacityGraph {
    #[rustfmt::skip]
    type IdxIter<'a> where Self: 'a = impl Iterator<Item=EdgeIdT> + 'a;

    fn edge_indices(&self, from: NodeId, to: NodeId) -> Self::IdxIter<'_> {
        self.neighbor_edge_indices(from)
            .filter(move |&edge_id| self.head[edge_id as usize] == to)
            .map(EdgeIdT)
    }

    #[inline(always)]
    fn neighbor_edge_indices(&self, node: NodeId) -> Range<u32> {
        let node = node as usize;
        (self.first_out[node])..(self.first_out[node + 1])
    }

    #[inline(always)]
    fn neighbor_edge_indices_usize(&self, node: NodeId) -> Range<usize> {
        let node = node as usize;
        (self.first_out[node] as usize)..(self.first_out[node + 1] as usize)
    }
}

/// trait needed for CCH potentials
impl EdgeRandomAccessGraph<Link> for CapacityGraph {
    #[inline(always)]
    fn link(&self, edge_id: u32) -> Link {
        let edge_id = edge_id as usize;
        Link {
            node: self.head[edge_id],
            weight: self.lowerbound_time[edge_id],
        }
    }
}

/// trait needed for CCH potentials
impl LinkIterable<NodeIdT> for CapacityGraph {
    type Iter<'a> = impl Iterator<Item = NodeIdT> + 'a;
    //type Iter<'a> = std::iter::Cloned<std::iter::Map<std::slice::Iter<'a, NodeId>, fn(&NodeId) -> NodeIdT>>;

    #[inline(always)]
    fn link_iter(&self, node: NodeId) -> Self::Iter<'_> {
        self.head[self.neighbor_edge_indices_usize(node)].iter().map(|&l| NodeIdT(l))
    }
}

/// trait needed for BackwardProfilePotential (creating reversed graph)
impl LinkIterable<(NodeIdT, EdgeIdT)> for CapacityGraph {
    type Iter<'a> = impl Iterator<Item = (NodeIdT, EdgeIdT)> + 'a;

    //type Iter<'a> = std::iter::Zip<std::iter::Cloned<std::slice::Iter<'a, NodeId>>, std::ops::Range<EdgeIdT>>;

    #[inline(always)]
    fn link_iter(&self, node: NodeId) -> Self::Iter<'_> {
        self.head[self.neighbor_edge_indices_usize(node)]
            .iter()
            .cloned()
            .zip(self.neighbor_edge_indices(node))
            .map(|(node, edge)| (NodeIdT(node), EdgeIdT(edge)))
    }
}

/// trait needed for CCH potentials
impl LinkIterable<Link> for CapacityGraph {
    #[allow(clippy::type_complexity)]
    type Iter<'a> = impl Iterator<Item = Link> + 'a;

    #[inline(always)]
    fn link_iter(&self, node: u32) -> Self::Iter<'_> {
        self.neighbor_edge_indices_usize(node).into_iter().map(move |idx| Link {
            node: self.head[idx],
            weight: self.lowerbound_time[idx],
        })
    }
}

// additional helper functions
/// determine the velocity in km_h for a given distance in meters and time in seconds
fn convert_to_velocity(dist_m: Weight, time_s: Weight) -> Velocity {
    assert!(dist_m < u32::MAX / 36, "integer overflow detected");
    if dist_m == 0 || time_s == 0 {
        1 // avoid division by zero exceptions. Travel time will be 0 anyway
    } else {
        (dist_m * 36) / (time_s * 10)
    }
}

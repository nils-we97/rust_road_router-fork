use rust_road_router::datastr::graph::{EdgeId, NodeId, Weight, Graph, LinkIterable, Link, RandomLinkAccessGraph};
use rust_road_router::io::{Deconstruct, Store};
use rust_road_router::datastr::graph::time_dependent::{Timestamp, PiecewiseLinearFunction};
use conversion::speed_profile_to_tt_profile;
use std::ops::Range;
use crate::graph::ModifiableWeight;
use std::cmp::min;

pub type Capacity = u32;

pub const MAX_BUCKETS: u32 = 86400;

pub struct TDCapacityGraph {
    num_buckets: u32,

    first_out: Vec<EdgeId>,
    head: Vec<NodeId>,
    //weight: Vec<Vec<(Timestamp, Weight)>>,

    departure: Vec<Vec<Timestamp>>,
    travel_time: Vec<Vec<Weight>>,

    distance: Vec<Weight>,
    freeflow_time: Vec<Weight>,
    max_capacity: Vec<Capacity>,
    used_capacity: Vec<Vec<Weight>>,
    speed: Vec<Vec<Weight>>,

    speed_function: fn(Weight, Capacity, Capacity) -> Weight,
}

impl TDCapacityGraph {
    /// Create a new `TDCapacityGraph` from the given containers and a weight function
    pub fn new(
        num_buckets: u32,
        first_out: Vec<EdgeId>,
        head: Vec<NodeId>,
        distance: Vec<Weight>,
        freeflow_time: Vec<Weight>,
        max_capacity: Vec<Capacity>,
        speed_function: fn(Weight, Capacity, Capacity) -> Weight,
    ) -> TDCapacityGraph {
        assert!(num_buckets > 0 && MAX_BUCKETS % num_buckets == 0); // avoid rounding when accessing buckets!

        assert!(first_out.len() > 0 && first_out.len() < <NodeId>::MAX as usize);
        assert!(head.len() > 0 && head.len() < <EdgeId>::MAX as usize);
        assert_eq!(first_out.first(), Some(&0));
        assert_eq!(first_out.last(), Some(&(head.len() as u32)));
        assert_eq!(freeflow_time.len(), head.len());
        assert_eq!(max_capacity.len(), head.len());

        let speed = freeflow_time
            .iter()
            .zip(distance.iter())
            .map(|(&time, &dist)| {
                // TODO add proper units here!
                let speed = dist / time;
                vec![min(speed, 1); num_buckets as usize]
            })
            .collect::<Vec<Vec<Weight>>>();

        let departure = vec![vec![0]; max_capacity.len()];

        let travel_time = freeflow_time
            .iter()
            .map(|&time| vec![time])
            .collect::<Vec<Vec<Weight>>>();

        let used_capacity = vec![vec![0; num_buckets as usize]; max_capacity.len()];

        /*let weight = freeflow_time
            .iter()
            .map(|&time| vec![(0, time)])
            .collect::<Vec<Vec<(Timestamp, Weight)>>>();*/

        TDCapacityGraph {
            num_buckets,
            first_out,
            head,
            departure,
            travel_time,
            distance,
            freeflow_time,
            max_capacity,
            used_capacity,
            speed,
            speed_function,
        }
    }

    /// Decompose the graph into its three seperate data containers
    pub fn decompose(self) -> (Vec<EdgeId>, Vec<NodeId>, Vec<Vec<Timestamp>>, Vec<Vec<Weight>>) {
        (self.first_out, self.head, self.departure, self.travel_time)
    }

    /// Borrow an individual travel time function.
    #[inline(always)]
    pub fn travel_time_function(&self, edge_id: EdgeId) -> PiecewiseLinearFunction {
        let edge_id = edge_id as usize;
        PiecewiseLinearFunction::new(&self.departure[edge_id], &self.travel_time[edge_id])
    }

    fn timestamp_to_bucket_id(&self, timestamp: Timestamp) -> usize {
        let timestamp = timestamp % MAX_BUCKETS; // clip to single day

        // convert to u64 to avoid integer overflows (86400^2 > u32::MAX)
        ((timestamp as u64 * self.num_buckets as u64) / MAX_BUCKETS as u64) as usize
    }

    fn bucket_id_to_timestamp(&self, bucket_id: usize) -> Timestamp {
        (bucket_id as u32) * (MAX_BUCKETS / self.num_buckets)
    }

    fn update_travel_time_profile(&mut self, edge_id: usize) {
        if edge_id == 393270 {
            println!("stop!");
        }

        let profile = speed_profile_to_tt_profile(
            self
                .speed[edge_id]
                .iter()
                .enumerate()
                .map(|(idx, &val)| (self.bucket_id_to_timestamp(idx), val))
                .collect::<Vec<(Timestamp, Weight)>>()
                .as_ref(),
            self.distance[edge_id],
        );

        self.departure[edge_id] = Vec::with_capacity(profile.len());
        self.travel_time[edge_id] = Vec::with_capacity(profile.len());

        profile.iter().enumerate().for_each(|(idx, (timestamp, weight))| {
            self.departure[edge_id][idx] = *timestamp;
            self.travel_time[edge_id][idx] = *weight;
        });
    }
}

impl Graph for TDCapacityGraph {
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

impl<TDPathContainer> ModifiableWeight<TDPathContainer> for TDCapacityGraph
    where TDPathContainer: AsRef<[(EdgeId, Timestamp)]>
{
    fn increase_weights(&mut self, path: TDPathContainer) {
        path.as_ref().iter().cloned().for_each(|(edge_id, timestamp)| {
            let edge_id = edge_id as usize;
            let bucket_id = self.timestamp_to_bucket_id(timestamp);

            self.used_capacity[edge_id][bucket_id] += 1;

            self.speed[edge_id][bucket_id] = (self.speed_function)(
                self.freeflow_time[edge_id],
                self.max_capacity[edge_id],
                self.used_capacity[edge_id][bucket_id],
            );

            self.update_travel_time_profile(edge_id);
        });
    }

    fn decrease_weights(&mut self, path: TDPathContainer) {
        path.as_ref().iter().cloned().for_each(|(edge_id, timestamp)| {
            let edge_id = edge_id as usize;
            let bucket_id = self.timestamp_to_bucket_id(timestamp);

            self.used_capacity[edge_id][bucket_id] -= 1;

            self.speed[edge_id][bucket_id] = (self.speed_function)(
                self.freeflow_time[edge_id],
                self.max_capacity[edge_id],
                self.used_capacity[edge_id][bucket_id],
            );

            self.update_travel_time_profile(edge_id);
        });
    }

    fn reset_weights(&mut self) {
        self.used_capacity.clear();

        for edge_id in 0..self.num_arcs() {
            for bucket_id in 0..self.num_buckets as usize {
                self.speed[edge_id][bucket_id] = (self.speed_function)(
                    self.freeflow_time[edge_id],
                    self.max_capacity[edge_id],
                    self.used_capacity[edge_id][bucket_id],
                );
            }

            self.update_travel_time_profile(edge_id);
        }
    }
}

impl Deconstruct for TDCapacityGraph {
    fn store_each(&self, store: &dyn Fn(&str, &dyn Store) -> std::io::Result<()>) -> std::io::Result<()> {
        store("first_out", &self.first_out)?;
        store("head", &self.head)?;
        store("weights", &self.freeflow_time)?;
        store("capacity", &self.max_capacity)?;
        Ok(())
    }
}

impl RandomLinkAccessGraph for TDCapacityGraph {
    fn link(&self, edge_id: EdgeId) -> Link {
        Link {
            node: self.head[edge_id as usize],
            weight: 0,
        }
    }

    fn edge_index(&self, from: NodeId, to: NodeId) -> Option<EdgeId> {
        let first_out = self.first_out[from as usize];
        let range = self.neighbor_edge_indices_usize(from);
        self.head[range].iter().position(|&head| head == to).map(|pos| pos as EdgeId + first_out)
    }

    #[inline(always)]
    fn neighbor_edge_indices(&self, node: NodeId) -> Range<EdgeId> {
        (self.first_out[node as usize] as EdgeId)..(self.first_out[(node + 1) as usize] as EdgeId)
    }

    #[inline(always)]
    fn neighbor_edge_indices_usize(&self, node: NodeId) -> Range<usize> {
        let node = node as usize;
        (self.first_out[node] as usize)..(self.first_out[(node + 1)] as usize)
    }
}

impl LinkIterable<NodeId> for TDCapacityGraph {
    type Iter<'a> = std::iter::Cloned<std::slice::Iter<'a, NodeId>>;

    #[inline(always)]
    fn link_iter(&self, node: NodeId) -> Self::Iter<'_> {
        self.head[self.neighbor_edge_indices_usize(node)].iter().cloned()
    }
}

impl LinkIterable<(NodeId, EdgeId)> for TDCapacityGraph {
    type Iter<'a> = std::iter::Zip<std::iter::Cloned<std::slice::Iter<'a, NodeId>>, std::ops::Range<EdgeId>>;

    #[inline(always)]
    fn link_iter(&self, node: NodeId) -> Self::Iter<'_> {
        self
            .head[self.neighbor_edge_indices_usize(node)]
            .iter()
            .cloned()
            .zip(self.neighbor_edge_indices(node))
    }
}

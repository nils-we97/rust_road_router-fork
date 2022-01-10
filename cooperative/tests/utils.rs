use std::cmp::max;

use cooperative::dijkstra::elimination_tree::multi_level_buckets::bucket_tree::BucketTreeEntry;
use cooperative::graph::capacity_graph::CapacityGraph;
use cooperative::graph::traffic_functions::bpr_traffic_function;
use cooperative::graph::{Capacity, MAX_BUCKETS};
use rust_road_router::datastr::graph::{NodeId, Weight};

pub struct CapacityEdge {
    pub from: NodeId,
    pub to: NodeId,
    pub distance: Weight,
    pub travel_time: Weight,
    pub capacity: Capacity,
}

impl CapacityEdge {
    pub fn new(from: NodeId, to: NodeId, distance: Weight, travel_time: Weight, capacity: Capacity) -> Self {
        Self {
            from,
            to,
            distance,
            travel_time,
            capacity,
        }
    }
}

/// Creates a dummy graph from a given (unordered) edge list.
pub fn create_graph(num_buckets: u32, mut edges: Vec<CapacityEdge>) -> CapacityGraph {
    assert!(!edges.is_empty(), "graph must not be empty!");

    // bring edges in sorted order
    edges.sort_by_key(|a| (a.from, a.to));

    let max_node_id = edges.iter().map(|edge| max(edge.from, edge.to)).max().unwrap();

    let mut head = Vec::with_capacity(edges.len());
    let mut distance = Vec::with_capacity(edges.len());
    let mut freeflow_time = Vec::with_capacity(edges.len());
    let mut max_capacity = Vec::with_capacity(edges.len());

    let mut degree = vec![0u32; max_node_id as usize];
    let mut first_out = vec![0];

    edges.iter().for_each(|edge| {
        degree[edge.from as usize] += 1;
        head.push(edge.to);
        distance.push(edge.distance);
        freeflow_time.push(edge.travel_time);
        max_capacity.push(edge.capacity);
    });

    degree.iter().for_each(|&deg| first_out.push(*first_out.last().unwrap() + deg));
    CapacityGraph::new(num_buckets, first_out, head, distance, freeflow_time, max_capacity, bpr_traffic_function)
}

#[test]
fn build_dummy_graph() {
    let edges = vec![
        CapacityEdge::new(0, 1, 10, 10000, 10),
        CapacityEdge::new(1, 2, 10, 10000, 10),
        CapacityEdge::new(2, 3, 10, 10000, 10),
    ];

    let graph = create_graph(1, edges);
    dbg!(&graph);

    assert_eq!(graph.first_out(), &[0, 1, 2, 3]);
    assert_eq!(graph.head(), &[1, 2, 3]);
}

#[test]
fn bucket_tree() {
    let entry = BucketTreeEntry {
        interval_start: 0,
        interval_end: 86400000,
        metric_index: 0,
        children: vec![
            BucketTreeEntry {
                interval_start: 0,
                interval_end: 14400000,
                metric_index: 2,
                children: vec![
                    BucketTreeEntry {
                        interval_start: 0,
                        interval_end: 7200000,
                        metric_index: 8,
                        children: vec![
                            BucketTreeEntry {
                                interval_start: 0,
                                interval_end: 3600000,
                                metric_index: 20,
                                children: vec![],
                            },
                            BucketTreeEntry {
                                interval_start: 3600000,
                                interval_end: 7200000,
                                metric_index: 21,
                                children: vec![],
                            },
                        ],
                    },
                    BucketTreeEntry {
                        interval_start: 7200000,
                        interval_end: 14400000,
                        metric_index: 9,
                        children: vec![
                            BucketTreeEntry {
                                interval_start: 7200000,
                                interval_end: 10800000,
                                metric_index: 22,
                                children: vec![],
                            },
                            BucketTreeEntry {
                                interval_start: 10800000,
                                interval_end: 14400000,
                                metric_index: 23,
                                children: vec![],
                            },
                        ],
                    },
                ],
            },
            BucketTreeEntry {
                interval_start: 14400000,
                interval_end: 28800000,
                metric_index: 3,
                children: vec![
                    BucketTreeEntry {
                        interval_start: 14400000,
                        interval_end: 21600000,
                        metric_index: 10,
                        children: vec![
                            BucketTreeEntry {
                                interval_start: 14400000,
                                interval_end: 18000000,
                                metric_index: 24,
                                children: vec![],
                            },
                            BucketTreeEntry {
                                interval_start: 18000000,
                                interval_end: 21600000,
                                metric_index: 25,
                                children: vec![],
                            },
                        ],
                    },
                    BucketTreeEntry {
                        interval_start: 21600000,
                        interval_end: 28800000,
                        metric_index: 11,
                        children: vec![
                            BucketTreeEntry {
                                interval_start: 21600000,
                                interval_end: 25200000,
                                metric_index: 26,
                                children: vec![],
                            },
                            BucketTreeEntry {
                                interval_start: 25200000,
                                interval_end: 28800000,
                                metric_index: 27,
                                children: vec![],
                            },
                        ],
                    },
                ],
            },
            BucketTreeEntry {
                interval_start: 28800000,
                interval_end: 43200000,
                metric_index: 4,
                children: vec![
                    BucketTreeEntry {
                        interval_start: 28800000,
                        interval_end: 36000000,
                        metric_index: 12,
                        children: vec![
                            BucketTreeEntry {
                                interval_start: 28800000,
                                interval_end: 32400000,
                                metric_index: 28,
                                children: vec![],
                            },
                            BucketTreeEntry {
                                interval_start: 32400000,
                                interval_end: 36000000,
                                metric_index: 29,
                                children: vec![],
                            },
                        ],
                    },
                    BucketTreeEntry {
                        interval_start: 36000000,
                        interval_end: 43200000,
                        metric_index: 13,
                        children: vec![
                            BucketTreeEntry {
                                interval_start: 36000000,
                                interval_end: 39600000,
                                metric_index: 30,
                                children: vec![],
                            },
                            BucketTreeEntry {
                                interval_start: 39600000,
                                interval_end: 43200000,
                                metric_index: 31,
                                children: vec![],
                            },
                        ],
                    },
                ],
            },
            BucketTreeEntry {
                interval_start: 43200000,
                interval_end: 57600000,
                metric_index: 5,
                children: vec![
                    BucketTreeEntry {
                        interval_start: 43200000,
                        interval_end: 50400000,
                        metric_index: 14,
                        children: vec![
                            BucketTreeEntry {
                                interval_start: 43200000,
                                interval_end: 46800000,
                                metric_index: 32,
                                children: vec![],
                            },
                            BucketTreeEntry {
                                interval_start: 46800000,
                                interval_end: 50400000,
                                metric_index: 33,
                                children: vec![],
                            },
                        ],
                    },
                    BucketTreeEntry {
                        interval_start: 50400000,
                        interval_end: 57600000,
                        metric_index: 15,
                        children: vec![
                            BucketTreeEntry {
                                interval_start: 50400000,
                                interval_end: 54000000,
                                metric_index: 34,
                                children: vec![],
                            },
                            BucketTreeEntry {
                                interval_start: 54000000,
                                interval_end: 57600000,
                                metric_index: 35,
                                children: vec![],
                            },
                        ],
                    },
                ],
            },
            BucketTreeEntry {
                interval_start: 57600000,
                interval_end: 72000000,
                metric_index: 6,
                children: vec![
                    BucketTreeEntry {
                        interval_start: 57600000,
                        interval_end: 64800000,
                        metric_index: 16,
                        children: vec![
                            BucketTreeEntry {
                                interval_start: 57600000,
                                interval_end: 61200000,
                                metric_index: 36,
                                children: vec![],
                            },
                            BucketTreeEntry {
                                interval_start: 61200000,
                                interval_end: 64800000,
                                metric_index: 37,
                                children: vec![],
                            },
                        ],
                    },
                    BucketTreeEntry {
                        interval_start: 64800000,
                        interval_end: 72000000,
                        metric_index: 17,
                        children: vec![
                            BucketTreeEntry {
                                interval_start: 64800000,
                                interval_end: 68400000,
                                metric_index: 38,
                                children: vec![],
                            },
                            BucketTreeEntry {
                                interval_start: 68400000,
                                interval_end: 72000000,
                                metric_index: 39,
                                children: vec![],
                            },
                        ],
                    },
                ],
            },
            BucketTreeEntry {
                interval_start: 72000000,
                interval_end: 86400000,
                metric_index: 7,
                children: vec![
                    BucketTreeEntry {
                        interval_start: 72000000,
                        interval_end: 79200000,
                        metric_index: 18,
                        children: vec![
                            BucketTreeEntry {
                                interval_start: 72000000,
                                interval_end: 75600000,
                                metric_index: 40,
                                children: vec![],
                            },
                            BucketTreeEntry {
                                interval_start: 75600000,
                                interval_end: 79200000,
                                metric_index: 41,
                                children: vec![],
                            },
                        ],
                    },
                    BucketTreeEntry {
                        interval_start: 79200000,
                        interval_end: 86400000,
                        metric_index: 19,
                        children: vec![
                            BucketTreeEntry {
                                interval_start: 79200000,
                                interval_end: 82800000,
                                metric_index: 42,
                                children: vec![],
                            },
                            BucketTreeEntry {
                                interval_start: 82800000,
                                interval_end: 86400000,
                                metric_index: 43,
                                children: vec![],
                            },
                        ],
                    },
                ],
            },
        ],
    };

    println!("Params: {}, {}", 85816191 % MAX_BUCKETS, 86711777 % MAX_BUCKETS);

    let result = entry.find_interval(85816191 % MAX_BUCKETS, 86711777 % MAX_BUCKETS);
    debug_assert!(result.is_some());
    dbg!(&result);
    //debug_assert_eq!(result.unwrap().metric_index, 20);
}

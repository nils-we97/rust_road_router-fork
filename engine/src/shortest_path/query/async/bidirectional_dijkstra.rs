use super::*;
use std::cmp::min;
use std::ptr;
use std::sync::{Arc, Barrier, RwLock};
use std::ops::Deref;

use shortest_path::timestamped_vector::TimestampedVector;

#[derive(Debug)]
struct DistancesPointerWrapper {
    pointer: *const TimestampedVector<Weight>
}

unsafe impl Send for DistancesPointerWrapper {}
unsafe impl Sync for DistancesPointerWrapper {}

impl Deref for DistancesPointerWrapper {
    type Target = TimestampedVector<Weight>;

    fn deref(&self) -> &TimestampedVector<Weight> {
        unsafe { self.pointer.as_ref().unwrap() }
    }
}

#[derive(Debug)]
pub struct Server {
    forward_query_sender: Sender<(ServerControl, u32)>,
    backward_query_sender: Sender<(ServerControl, u32)>,
    forward_progress_receiver: Receiver<(QueryProgress, u32)>,
    backward_progress_receiver: Receiver<(QueryProgress, u32)>,

    forward_distances_pointer: Arc<RwLock<DistancesPointerWrapper>>,
    backward_distances_pointer: Arc<RwLock<DistancesPointerWrapper>>,
    tentative_distance: Weight,

    active_query_id: u32
}

impl Server {
    pub fn new(graph: Graph) -> Server {
        let (forward_query_sender, forward_query_receiver) = channel();
        let (forward_progress_sender, forward_progress_receiver) = channel();
        let (backward_query_sender, backward_query_receiver) = channel();
        let (backward_progress_sender, backward_progress_receiver) = channel();

        let forward_distances_pointer = Arc::new(RwLock::new(DistancesPointerWrapper { pointer: ptr::null() }));
        let backward_distances_pointer = Arc::new(RwLock::new(DistancesPointerWrapper { pointer: ptr::null() }));

        let query_barrier = Arc::new(Barrier::new(2));
        let init_barrier = Arc::new(Barrier::new(3));

        let reversed = graph.reverse();
        Server::spawn_direction_worker(graph, forward_query_receiver, forward_progress_sender, forward_distances_pointer.clone(), backward_distances_pointer.clone(), query_barrier.clone(), init_barrier.clone());
        Server::spawn_direction_worker(reversed, backward_query_receiver, backward_progress_sender, backward_distances_pointer.clone(), forward_distances_pointer.clone(), query_barrier, init_barrier.clone());

        init_barrier.wait();

        Server {
            forward_query_sender,
            backward_query_sender,
            forward_progress_receiver,
            backward_progress_receiver,

            forward_distances_pointer,
            backward_distances_pointer,
            tentative_distance: INFINITY,

            active_query_id: 0
        }
    }

    fn spawn_direction_worker(graph: Graph,
                              query_receiver: Receiver<(ServerControl, u32)>,
                              progress_sender: Sender<(QueryProgress, u32)>,
                              distances_pointer: Arc<RwLock<DistancesPointerWrapper>>,
                              reverse_distances_pointer: Arc<RwLock<DistancesPointerWrapper>>,
                              query_barrier: Arc<Barrier>,
                              init_barrier: Arc<Barrier>) {

        thread::spawn(move || {
            let mut dijkstra = SteppedDijkstra::new(graph);

            {
                let mut p = distances_pointer.write().unwrap();
                p.pointer = dijkstra.distances_pointer();
            }

            init_barrier.wait();

            let reverse_distances_pointer = reverse_distances_pointer.read().unwrap();

            loop {
                match query_receiver.recv() {
                    Ok((ServerControl::Query(query), active_query_id)) => {
                        dijkstra.initialize_query(query);
                        let mut tentative_distance = INFINITY;

                        query_barrier.wait();

                        let mut i = 0;
                        loop {
                            i += 1;
                            if i % 1024 == 0 {
                                match query_receiver.try_recv() {
                                    Ok((ServerControl::Break, query_id)) if active_query_id == query_id => break,
                                    Ok((ServerControl::Query(_), query_id)) => panic!("backward received new query {} while still processing {}", query_id, active_query_id),
                                    _ => ()
                                }
                            }

                            let progress = dijkstra.next_step();
                            match progress {
                                QueryProgress::Progress(State { distance, node }) => {
                                    let reverse_distance = reverse_distances_pointer[node as usize];
                                    if distance + reverse_distance <= tentative_distance {
                                        match progress_sender.send((progress.clone(), active_query_id)) {
                                            Ok(_) => (),
                                            Err(_) => break, // main thread gone
                                        }
                                        tentative_distance = distance + reverse_distance;
                                    }
                                },
                                QueryProgress::Done(_) => {
                                    match progress_sender.send((progress.clone(), active_query_id)) {
                                        Ok(_) => (),
                                        Err(_) => break, // main thread gone
                                    }
                                    break;
                                }
                            }
                        }
                    },
                    Ok((ServerControl::Break, _)) => (),
                    Ok((ServerControl::Shutdown, _)) | Err(_) => break
                }
            }
        });
    }

    pub fn distance(&mut self, from: NodeId, to: NodeId) -> Option<Weight> {
        // initialize
        self.tentative_distance = INFINITY;
        self.active_query_id += 1;

        self.forward_query_sender.send((ServerControl::Query(Query { from, to }), self.active_query_id)).unwrap();
        self.backward_query_sender.send((ServerControl::Query(Query { from: to, to: from }), self.active_query_id)).unwrap();

        let mut forward_progress = 0;
        let mut backward_progress = 0;

        let forward_distances_pointer = self.forward_distances_pointer.read().unwrap();
        let backward_distances_pointer = self.backward_distances_pointer.read().unwrap();

        let forward_receiver = &self.forward_progress_receiver;
        let backward_receiver = &self.backward_progress_receiver;

        while self.tentative_distance >= forward_progress + backward_progress {
            // this is an unstable API which will likely be removed at some point, so we gotta get rid of it
            // one possibility could be crossbeam-channel https://github.com/crossbeam-rs/rfcs/pull/22
            select! {
                forward_message = forward_receiver.recv() => {
                    match forward_message {
                        Ok((_, query_id)) if query_id != self.active_query_id => (),
                        Ok((QueryProgress::Done(result), _)) => {
                            self.backward_query_sender.send((ServerControl::Break, self.active_query_id)).unwrap();
                            return result
                        },
                        Ok((QueryProgress::Progress(State { distance, node }), _)) => {
                            forward_progress = distance;
                            let other_distance = backward_distances_pointer[node as usize];
                            self.tentative_distance = min(distance + other_distance, self.tentative_distance);
                        },
                        Err(e) => panic!("{:?}", e)
                    }
                },
                backward_message = backward_receiver.recv() => {
                    match backward_message {
                        Ok((_, query_id)) if query_id != self.active_query_id => (),
                        Ok((QueryProgress::Done(result), _)) => {
                            self.forward_query_sender.send((ServerControl::Break, self.active_query_id)).unwrap();
                            return result
                        },
                        Ok((QueryProgress::Progress(State { distance, node }), _)) => {
                            backward_progress = distance;
                            let other_distance = forward_distances_pointer[node as usize];
                            self.tentative_distance = min(distance + other_distance, self.tentative_distance);
                        },
                        Err(e) => panic!("{:?}", e)
                    }
                }
            }
        }

        self.forward_query_sender.send((ServerControl::Break, self.active_query_id)).unwrap();
        self.backward_query_sender.send((ServerControl::Break, self.active_query_id)).unwrap();

        match self.tentative_distance {
            INFINITY => None,
            dist => Some(dist)
        }
    }
}

impl Drop for Server {
    fn drop(&mut self) {
        self.forward_query_sender.send((ServerControl::Shutdown, 0)).unwrap();
        self.backward_query_sender.send((ServerControl::Shutdown, 0)).unwrap();
    }
}
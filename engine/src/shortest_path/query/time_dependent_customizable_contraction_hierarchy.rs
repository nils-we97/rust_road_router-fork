use graph::time_dependent::*;
use shortest_path::td_stepped_dijkstra::QueryProgress as OtherQueryProgress;
use std::cmp::min;
use shortest_path::td_stepped_dijkstra::TDSteppedDijkstra;
use super::*;
use super::td_stepped_elimination_tree::*;
use ::shortest_path::customizable_contraction_hierarchy::cch_graph::CCHGraph;
// use ::in_range_option::InRangeOption;
use self::td_stepped_elimination_tree::QueryProgress;
use rank_select_map::BitVec;

#[derive(Debug)]
pub struct Server<'a> {
    forward: TDSteppedEliminationTree<'a, 'a>,
    backward: TDSteppedEliminationTree<'a, 'a>,
    td_dijkstra: TDSteppedDijkstra,
    cch_graph: &'a CCHGraph,
    shortcut_graph: &'a ShortcutGraph<'a>,
    tentative_distance: (Weight, Weight),
    meeting_nodes: Vec<NodeId>,
    m_orig: usize,
    m_cch: usize
}

impl<'a> Server<'a> {
    pub fn new<Graph>(cch_graph: &'a CCHGraph, shortcut_graph: &'a ShortcutGraph<'a>) -> Self {
        Self {
            forward: TDSteppedEliminationTree::new(shortcut_graph.upward_graph(), cch_graph.elimination_tree()),
            backward: TDSteppedEliminationTree::new(shortcut_graph.downward_graph(), cch_graph.elimination_tree()),
            td_dijkstra: TDSteppedDijkstra::new(shortcut_graph.original_graph().clone()), // TODO fix clone
            cch_graph,
            meeting_nodes: Vec::new(),
            tentative_distance: (INFINITY, INFINITY),
            shortcut_graph,
            m_orig: shortcut_graph.original_graph().num_arcs(),
            m_cch: cch_graph.num_arcs()
        }
    }

    pub fn distance(&mut self, from: NodeId, to: NodeId, departure_time: Timestamp) -> Option<Weight> {
        let from = self.cch_graph.node_order().rank(from);
        let to = self.cch_graph.node_order().rank(to);

        // initialize
        self.tentative_distance = (INFINITY, INFINITY);
        self.meeting_nodes.clear();
        self.forward.initialize_query(from);
        self.backward.initialize_query(to);

        // forward up
        while self.forward.next().is_some() {
            self.forward.next_step();
        }

        // backward up
        while let QueryProgress::Progress(node) = self.backward.next_step() {
            if self.forward.node_data(node).lower_bound + self.backward.node_data(node).lower_bound < self.tentative_distance.0 ||
                self.forward.node_data(node).upper_bound + self.backward.node_data(node).upper_bound < self.tentative_distance.1
            {
                self.tentative_distance.0 = min(self.tentative_distance.0, self.forward.node_data(node).lower_bound + self.backward.node_data(node).lower_bound);
                self.tentative_distance.1 = min(self.tentative_distance.1, self.forward.node_data(node).upper_bound + self.backward.node_data(node).upper_bound);
                self.meeting_nodes.push(node);
            }
        }

        let mut original_edges = BitVec::new(self.m_orig);
        let mut shortcuts = BitVec::new(2 * self.m_cch);
        let mut forward_queue: Vec<NodeId> = Vec::new();
        forward_queue.extend(&self.meeting_nodes);
        let mut backward_queue: Vec<NodeId> = Vec::new();
        backward_queue.extend(&self.meeting_nodes);

        while let Some(node) = forward_queue.pop() {
            for label in &self.forward.node_data(node).labels {
                if !shortcuts.get(label.shortcut_id as usize * 2) {
                    shortcuts.set(label.shortcut_id as usize * 2);
                    self.shortcut_graph.get_upward(label.shortcut_id).unpack(self.shortcut_graph, &mut shortcuts, &mut original_edges);
                    forward_queue.push(label.parent);
                }
            }
        }

        while let Some(node) = backward_queue.pop() {
            for label in &self.backward.node_data(node).labels {
                if !shortcuts.get(label.shortcut_id as usize * 2 + 1) {
                    shortcuts.set(label.shortcut_id as usize * 2 + 1);
                    self.shortcut_graph.get_downward(label.shortcut_id).unpack(self.shortcut_graph, &mut shortcuts, &mut original_edges);
                    backward_queue.push(label.parent);
                }
            }
        }

        self.td_dijkstra.initialize_query(TDQuery { from, to, departure_time }, original_edges);
        loop {
            match self.td_dijkstra.next_step() {
                OtherQueryProgress::Progress(_) => continue,
                OtherQueryProgress::Done(result) => return result
            }
        }
    }
}

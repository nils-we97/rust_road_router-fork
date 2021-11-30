use rust_road_router::datastr::graph::floating_time_dependent::shortcut::Sources;
use rust_road_router::datastr::graph::floating_time_dependent::shortcut_source::ShortcutSource;
use rust_road_router::datastr::graph::floating_time_dependent::{
    FlWeight, MutTopPLF, PartialATTF, PeriodicATTF, ReusablePLFStorage, Shortcut, ShortcutGraphTrt, ShortcutId, TDGraph, Timestamp,
};
use rust_road_router::datastr::graph::EdgeId;

#[derive(Debug)]
pub struct ShortcutWrapper {
    pub shortcut: Shortcut,
    pub interval_minima: Vec<u32>,
    pub bounds: (u32, u32),
}

impl ShortcutWrapper {
    pub fn empty(shortcut: Shortcut) -> Self {
        Self {
            shortcut,
            interval_minima: vec![],
            bounds: (0, 0),
        }
    }

    pub fn reenable_required(&self, downward: &mut [ShortcutWrapper], upward: &mut [ShortcutWrapper]) {
        if self.shortcut.required {
            for (_, source) in self.shortcut.sources_iter() {
                if let ShortcutSource::Shortcut(down, up) = source.into() {
                    downward[down as usize].shortcut.required = true;
                    upward[up as usize].shortcut.required = true;
                }
            }
        }
    }

    /// If Shortcuts in skipped triangles are not required, the corresponding `Source` in this shortcut is also not required, so remove it
    pub fn disable_if_unnecessary(&mut self, shortcut_graph: &PartialShortcutWrapperGraph) {
        match &mut self.shortcut.sources {
            Sources::None => {}
            Sources::One(source) => match ShortcutSource::from(*source) {
                ShortcutSource::None => {
                    self.shortcut.required = false;
                }
                ShortcutSource::Shortcut(down, up) => {
                    if !shortcut_graph.get_incoming(down).required || !shortcut_graph.get_outgoing(up).required {
                        self.shortcut.required = false;
                    }
                }
                _ => (),
            },
            Sources::Multi(sources) => {
                let mut any_required = false;
                for (_, source) in &mut sources[..] {
                    match ShortcutSource::from(*source) {
                        ShortcutSource::OriginalEdge(_) => {
                            any_required = true;
                            break;
                        }
                        ShortcutSource::Shortcut(down, up) => {
                            if shortcut_graph.get_incoming(down).required && shortcut_graph.get_outgoing(up).required {
                                any_required = true;
                                break;
                            }
                        }
                        _ => (),
                    }
                }
                if !any_required {
                    self.shortcut.required = false;
                }
            }
        }
    }
}

pub struct PartialShortcutWrapperGraph<'a> {
    pub original_graph: &'a TDGraph,
    outgoing: &'a [ShortcutWrapper],
    incoming: &'a [ShortcutWrapper],
    offset: usize,
}

impl<'a> PartialShortcutWrapperGraph<'a> {
    /// Create `PartialShortcutGraph` from original graph, shortcut slices in both directions and an offset to map CCH edge ids to slice indices
    pub fn new(original_graph: &'a TDGraph, outgoing: &'a [ShortcutWrapper], incoming: &'a [ShortcutWrapper], offset: usize) -> Self {
        Self {
            original_graph,
            outgoing,
            incoming,
            offset,
        }
    }

    fn get(&self, shortcut_id: ShortcutId) -> &Shortcut {
        match shortcut_id {
            ShortcutId::Incoming(id) => self.get_incoming(id),
            ShortcutId::Outgoing(id) => self.get_outgoing(id),
        }
    }

    /// Borrow upward `Shortcut` with given CCH EdgeId
    pub fn get_outgoing(&self, edge_id: EdgeId) -> &Shortcut {
        &self.outgoing[edge_id as usize - self.offset].shortcut
    }

    /// Borrow downward `Shortcut` with given CCH EdgeId
    pub fn get_incoming(&self, edge_id: EdgeId) -> &Shortcut {
        &self.incoming[edge_id as usize - self.offset].shortcut
    }
}

impl<'a> ShortcutGraphTrt for PartialShortcutWrapperGraph<'a> {
    type OriginalGraph = TDGraph;

    fn periodic_ttf(&self, shortcut_id: ShortcutId) -> Option<PeriodicATTF> {
        self.get(shortcut_id).periodic_ttf(self)
    }
    fn partial_ttf(&self, shortcut_id: ShortcutId, start: Timestamp, end: Timestamp) -> Option<PartialATTF> {
        self.get(shortcut_id).partial_ttf(self, start, end)
    }
    fn is_valid_path(&self, shortcut_id: ShortcutId) -> bool {
        self.get(shortcut_id).is_valid_path()
    }
    fn lower_bound(&self, shortcut_id: ShortcutId) -> FlWeight {
        self.get(shortcut_id).lower_bound
    }
    fn upper_bound(&self, shortcut_id: ShortcutId) -> FlWeight {
        self.get(shortcut_id).upper_bound
    }
    fn original_graph(&self) -> &TDGraph {
        &self.original_graph
    }
    fn reconstruct_exact_ttf(&self, shortcut_id: ShortcutId, start: Timestamp, end: Timestamp, target: &mut MutTopPLF, tmp: &mut ReusablePLFStorage) {
        self.get(shortcut_id).reconstruct_exact_ttf(start, end, self, target, tmp)
    }
    fn get_switchpoints(&self, shortcut_id: ShortcutId, start: Timestamp, end: Timestamp) -> (Vec<(Timestamp, Vec<EdgeId>, FlWeight)>, FlWeight) {
        self.get(shortcut_id).get_switchpoints(start, end, self)
    }
    fn unpack_at(&self, shortcut_id: ShortcutId, t: Timestamp, result: &mut Vec<(EdgeId, Timestamp)>) {
        self.get(shortcut_id).unpack_at(t, self, result);
    }
    fn evaluate(&self, shortcut_id: ShortcutId, t: Timestamp) -> FlWeight {
        self.get(shortcut_id).evaluate(t, self)
    }
}

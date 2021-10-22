use rust_road_router::algo::dijkstra::{DijkstraOps, Label};
use rust_road_router::datastr::graph::floating_time_dependent::{PeriodicPiecewiseLinearFunction, TTFPoint};
use rust_road_router::datastr::graph::{NodeIdT, Reversed, ReversedGraphWithEdgeIds};

pub struct TDBackwardProfilePotentialOps<Profiles>(pub Profiles);

impl<Profiles: AsRef<Vec<Vec<TTFPoint>>>> DijkstraOps<ReversedGraphWithEdgeIds> for TDBackwardProfilePotentialOps<Profiles> {
    type Label = Vec<TTFPoint>;
    type Arc = (NodeIdT, Reversed);
    type LinkResult = Vec<TTFPoint>;
    type PredecessorLink = (); // no paths are calculated here => not needed

    // label = state at currently processed node
    // must be linked backward with (static) weight at previous edge
    fn link(&mut self, _graph: &ReversedGraphWithEdgeIds, label: &Self::Label, (_, prev_edge): &Self::Arc) -> Self::LinkResult {
        //1. obtain profile from `previous_node`
        let prev_profile = PeriodicPiecewiseLinearFunction::new(&self.0.as_ref()[prev_edge.0 .0 as usize]);
        let current_profile = PeriodicPiecewiseLinearFunction::new(label);

        //2. link (`prev_profile` and `label`)
        let link_result = prev_profile.link(&current_profile);

        PeriodicPiecewiseLinearFunction::new(&link_result).to_vec()

        //3. apply douglas peuker approximation -> accelerates calculation by factor ~10
        // approximation does currently not work! TODO fix this
        //PeriodicPiecewiseLinearFunction::new(&link_result).approximate(&mut Vec::new()).to_vec()
    }

    fn merge(&mut self, label: &mut Self::Label, linked: Self::LinkResult) -> bool {
        // easy case: label is empty -> simply uses recently linked profile
        if *label == Self::Label::neutral() {
            *label = linked;
            return true;
        }

        // more complex case requires actual merging
        let linked_profile = PeriodicPiecewiseLinearFunction::new(&linked);
        let current_profile = PeriodicPiecewiseLinearFunction::new(label);

        let (result, changes) = current_profile.merge(&linked_profile, &mut Vec::new());

        // merging takes place if there is any point where the linked profile is 'better',
        // i.e. the current profile does not dominate all the time
        if changes.iter().any(|&(_, b)| !b) {
            *label = result.to_vec();
            true
        } else {
            false
        }
    }

    fn predecessor_link(&self, _link: &Self::Arc) -> Self::PredecessorLink {
        ()
    }
}
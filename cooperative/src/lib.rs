#![feature(generic_associated_types)]
#![feature(min_type_alias_impl_trait)]

pub mod dijkstra;
pub mod experiments;
pub mod graph;
pub mod io;
pub mod visualization;

#[cfg(test)]
mod tests {
    use crate::io::parse_visum_graph::convert_coord;
    use conversion::speed_profile_to_tt_profile;
    use rust_road_router::datastr::graph::time_dependent::{period, Timestamp};
    use rust_road_router::datastr::graph::Weight;

    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }

    #[test]
    fn test_coord_conversion() {
        let coords = vec![3513149i32];
        let result = convert_coord(&coords);

        println!("{:?}", result);
    }

    #[test]
    fn test_profile() {
        // link dist = meter
        // speed = km/h

        let speed: Vec<(Timestamp, u32)> = vec![(0, 100), (10000 * 1000, 150), (52500 * 1000, 200), (86400 * 1000, 100)];

        dbg!(period());

        let _ = speed_profile_to_tt_profile(&speed, 1000)
            .iter()
            .cloned()
            .for_each(|(ts, val)| println!("{}: {}", ts, val));
    }
}

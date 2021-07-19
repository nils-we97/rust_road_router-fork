#![feature(generic_associated_types)]
#![feature(min_type_alias_impl_trait)]

pub mod io;
pub mod graph;
pub mod dijkstra;
pub mod visualization;
pub mod experiments;


#[cfg(test)]
mod tests {
    use rust_road_router::datastr::graph::Weight;
    use rust_road_router::datastr::graph::time_dependent::{Timestamp, period};
    use conversion::speed_profile_to_tt_profile;

    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }

    #[test]
    fn test_profile() {
        // link dist = meter
        // speed = km/h

        let speed: Vec<(Timestamp, u32)> = vec![
            (0, 100),
            (10000 * 1000, 150),
            (52500 * 1000, 200),
            (86400 * 1000, 100)
        ];

        dbg!(period());

        let _ = speed_profile_to_tt_profile(&speed, 1000).iter().cloned().for_each(|(ts, val)| {
            println!("{}: {}", ts, val)
        });

        assert_eq!(2+2, 4);
    }
}
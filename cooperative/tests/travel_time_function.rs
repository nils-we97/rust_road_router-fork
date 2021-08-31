use conversion::speed_profile_to_tt_profile;
use cooperative::graph::MAX_BUCKETS;

#[test]
fn test_ttf() {
    /*let speeds = [(0, 30), (1_000_000, 90), (1_050_000, 60), (1_100_000, 180), (MAX_BUCKETS, 30)];
    let ttf = speed_profile_to_tt_profile(&speeds, 1500);
    dbg!(&ttf);*/

    let speeds = [(0, 90), (1_000_000, 90), (2_000_000, 90), (MAX_BUCKETS, 90)];
    let ttf = speed_profile_to_tt_profile(&speeds, 1500);
    dbg!(&ttf);

    let speeds = [(0, 90), (1_000_000, 90), (2_000_000, 80), (MAX_BUCKETS, 90)];
    let ttf = speed_profile_to_tt_profile(&speeds, 1500);
    dbg!(&ttf);
}

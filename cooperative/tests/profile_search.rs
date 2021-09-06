use cooperative::graph::MAX_BUCKETS;
use cooperative::util::profile_search::find_profile_index;

#[test]
fn profile_search_test() {
    let profile = [
        (0, 100),
        (MAX_BUCKETS / 10, 10),
        (MAX_BUCKETS / 5, 200),
        (MAX_BUCKETS / 2, 100),
        (MAX_BUCKETS, 500),
    ];

    assert_eq!(find_profile_index(&profile, 0), Ok(0));
    assert_eq!(find_profile_index(&profile, MAX_BUCKETS), Ok(4));

    assert_eq!(find_profile_index(&profile, 5), Err(1));
    assert_eq!(find_profile_index(&profile, MAX_BUCKETS / 4), Err(3));
    assert_eq!(find_profile_index(&profile, MAX_BUCKETS / 2), Ok(3));
    assert_eq!(find_profile_index(&profile, MAX_BUCKETS - 1), Err(4));
}

pub fn to_velocity(dist: u32, time: u32) -> u32 {
    // assuming that time_in_hours = time / (3600 * 10) and dist_km = dist / 1000
    assert!(dist < u32::MAX / 36);

    if dist == 0 || time == 0 || time > dist * 36 {
        1 // avoid division by zero exceptions. Travel time will be 0 anyway
    } else {
        (dist * 36) / time
    }
}

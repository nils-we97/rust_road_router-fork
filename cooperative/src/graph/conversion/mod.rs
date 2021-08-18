/// determine the velocity in km_h for a given distance in meters and time in seconds
pub fn to_velocity(dist_m: u32, time_s: u32) -> u32 {
    assert!(dist_m < u32::MAX / 36, "integer overflow detected");
    if dist_m == 0 || time_s == 0 {
        1 // avoid division by zero exceptions. Travel time will be 0 anyway
    } else {
        (dist_m * 36) / (time_s * 10)
    }
}

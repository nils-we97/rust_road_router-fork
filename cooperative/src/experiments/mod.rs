pub mod compare_static_cooperative;
pub mod evaluation;
pub mod generate_queries;
pub mod run_server;
pub mod run_td_server;
pub mod evaluate_td_impact;
pub mod run_server_with_potentials;
pub mod evaluate_potential_quality;

#[derive(Copy, Clone, Debug)]
pub struct PathCompareResult {
    pub num_queries: u32,
    pub dist_baseline: u32,
    pub dist_algorithm: u32,
    pub improvement: f64,
}

impl PathCompareResult {
    pub fn new(num_queries: u32, dist_baseline: u32, dist_algorithm: u32) -> Self {
        Self {
            num_queries,
            dist_baseline,
            dist_algorithm,
            improvement: dist_baseline as f64 / dist_algorithm as f64
        }
    }
}
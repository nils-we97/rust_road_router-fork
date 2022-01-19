use rust_road_router::cli::CliErr;
use std::str::FromStr;

#[derive(Debug, Clone)]
pub enum PotentialType {
    CCHPot,
    CorridorLowerbound,
    MultiMetrics,
    MultiLevelBucket,
}

impl FromStr for PotentialType {
    type Err = CliErr;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_uppercase().as_str() {
            "CCH_POT" => Ok(Self::CCHPot),
            "CORRIDOR_LOWERBOUND" => Ok(Self::CorridorLowerbound),
            "MULTI_METRICS" => Ok(Self::MultiMetrics),
            "MULTI_LEVEL_BUCKET" => Ok(Self::MultiLevelBucket),
            _ => Err(CliErr("Invalid Graph Type [CORRIDOR_LOWERBOUND/MULTI_METRICS]")),
        }
    }
}

impl ToString for PotentialType {
    fn to_string(&self) -> String {
        match self {
            PotentialType::CCHPot => "CCH-Pot".to_string(),
            PotentialType::CorridorLowerbound => "Corridor-Lowerbound".to_string(),
            PotentialType::MultiMetrics => "Multi-Metric".to_string(),
            PotentialType::MultiLevelBucket => "Multi-Level-Bucket".to_string(),
        }
    }
}

use std::env::Args;
use std::error::Error;
use std::str::FromStr;

use rust_road_router::cli::CliErr;
use std::borrow::Borrow;

#[derive(Debug, Clone)]
pub enum QueryType {
    Uniform,
    UniformConstantDep,
    Geometric,
    GeometricConstantDep,
    PopulationUniform,
    PopulationUniformConstantDep,
    PopulationGeometric,
    PopulationGeometricConstantDep,
}

impl FromStr for QueryType {
    type Err = CliErr;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_uppercase().as_str() {
            "UNIFORM" => Ok(QueryType::Uniform),
            "UNIFORM_CONSTANT_DEPARTURE" => Ok(QueryType::UniformConstantDep),
            "GEOMETRIC" => Ok(QueryType::Geometric),
            "GEOMETRIC_CONSTANT_DEPARTURE" => Ok(QueryType::GeometricConstantDep),
            /*"POPULATION_UNIFORM" => Ok(QueryType::PopulationUniform),
            "POPULATION_UNIFORM_CONSTANT_DEPARTURE" => Ok(QueryType::PopulationUniformConstantDep),
            "POPULATION_GEOMETRIC" => Ok(QueryType::PopulationGeometric),
            "POPULATION_GEOMETRIC_CONSTANT_DEPARTURE" => Ok(QueryType::PopulationGeometricConstantDep),*/
            _ => Err(CliErr("Unknown Query Type!"))
        }
    }
}

pub fn parse_arg_required<T: FromStr>(
    args: &mut impl Iterator<Item=String>,
    field_name: & str
) -> Result<T, Box<dyn Error>> {
    let next = args.next();

    if next.is_some() {
        let val = T::from_str(&next.unwrap());
        if val.is_ok() {
            Ok(val.ok().unwrap())
        } else {
            dbg!("Invalid argument type for `{}`", field_name);
            Err(Box::new(CliErr("Invalid argument!")))
        }
    } else {
        dbg!("Missing value for argument `{}`", field_name);
        Err(Box::new(CliErr("Missing arguments!")))
    }
}

pub fn parse_arg_optional<T: FromStr + Clone>(args: &mut impl Iterator<Item=String>, default: T) -> T {
    args.next().map(|s| T::from_str(&s).unwrap_or(default.clone())).unwrap_or(default)
}
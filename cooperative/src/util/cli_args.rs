use std::error::Error;
use std::str::FromStr;

use rust_road_router::cli::CliErr;

pub fn parse_arg_required<T: FromStr>(args: &mut impl Iterator<Item = String>, field_name: &str) -> Result<T, Box<dyn Error>> {
    let next = args.next();

    if next.is_some() {
        let val = T::from_str(&next.unwrap());
        if val.is_ok() {
            Ok(val.ok().unwrap())
        } else {
            println!("Invalid argument type for `{}`", field_name);
            Err(Box::new(CliErr("Invalid argument!")))
        }
    } else {
        println!("Missing value for argument `{}`", field_name);
        Err(Box::new(CliErr("Missing arguments!")))
    }
}

pub fn parse_arg_optional<T: FromStr + Clone>(args: &mut impl Iterator<Item = String>, default: T) -> T {
    args.next().map(|s| T::from_str(&s).unwrap_or(default.clone())).unwrap_or(default)
}

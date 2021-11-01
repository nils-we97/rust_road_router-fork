#![feature(generic_associated_types)]
#![feature(type_alias_impl_trait)]

pub mod dijkstra;
pub mod experiments;
pub mod graph;
pub mod io;
pub mod util;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}

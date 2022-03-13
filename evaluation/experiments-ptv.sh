#### Experiment 1: Potential Preprocessing on PTV Graphs - Space and Time
# Lux-20
cargo +nightly run --package cooperative --bin ptv_customize_graph --release -- ../graphs/ptv/ptv20-lux-car/ CCH_POT
cargo +nightly run --package cooperative --bin ptv_customize_graph --release -- ../graphs/ptv/ptv20-lux-car/ MULTI_METRICS 20_multi_metrics 20
cargo +nightly run --package cooperative --bin ptv_customize_graph --release -- ../graphs/ptv/ptv20-lux-car/ CORRIDOR_LOWERBOUND 72_corridor_lowerbound 72

# Ger-06
cargo +nightly run --package cooperative --bin ptv_customize_graph --release -- ../graphs/ptv/de/ CCH_POT
cargo +nightly run --package cooperative --bin ptv_customize_graph --release -- ../graphs/ptv/de/ MULTI_METRICS 20_multi_metrics 20
cargo +nightly run --package cooperative --bin ptv_customize_graph --release -- ../graphs/ptv/de/ CORRIDOR_LOWERBOUND 72_corridor_lowerbound 72

# Eur-17
cargo +nightly run --package cooperative --bin ptv_customize_graph --release -- ../graphs/ptv/ptv17-eur-car/ CCH_POT
cargo +nightly run --package cooperative --bin ptv_customize_graph --release -- ../graphs/ptv/ptv17-eur-car/ MULTI_METRICS 20_multi_metrics 20
cargo +nightly run --package cooperative --bin ptv_customize_graph --release -- ../graphs/ptv/ptv17-eur-car/ CORRIDOR_LOWERBOUND 72_corridor_lowerbound 72

# Eur-20
cargo +nightly run --package cooperative --bin ptv_customize_graph --release -- ../graphs/ptv/ptv20-eur-car/ CCH_POT
cargo +nightly run --package cooperative --bin ptv_customize_graph --release -- ../graphs/ptv/ptv20-eur-car/ MULTI_METRICS 20_multi_metrics 20
cargo +nightly run --package cooperative --bin ptv_customize_graph --release -- ../graphs/ptv/ptv20-eur-car/ CORRIDOR_LOWERBOUND 72_corridor_lowerbound 72

#---------------------------------------------------------------------------------

#### Experiment 2: Potential Runtimes on PTV Graphs
# run graph-wise, generate queries in advance

# Lux-20
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/ptv20-lux-car/ PTV 10000 UNIFORM q_10000_uni
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/ptv20-lux-car/ PTV 10000 GEOMETRIC_RUSH_HOUR q_10000_geom
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/ptv20-lux-car/ PTV 10000 POPULATION_UNIFORM q_10000_pop_uni ../graphs/population_grid/europe/
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/ptv20-lux-car/ PTV 10000 POPULATION_GEOMETRIC q_10000_pop_geom ../graphs/population_grid/europe/

RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_compare_potentials --release -- ../graphs/ptv/ptv20-lux-car/ q_10000_uni 72_corridor_lowerbound 20_multi_metrics
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_compare_potentials --release -- ../graphs/ptv/ptv20-lux-car/ q_10000_geom 72_corridor_lowerbound 20_multi_metrics
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_compare_potentials --release -- ../graphs/ptv/ptv20-lux-car/ q_10000_pop_uni 72_corridor_lowerbound 20_multi_metrics
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_compare_potentials --release -- ../graphs/ptv/ptv20-lux-car/ q_10000_pop_geom 72_corridor_lowerbound 20_multi_metrics

# Ger-06
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/de/ PTV 10000 UNIFORM q_10000_uni
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/de/ PTV 10000 GEOMETRIC_RUSH_HOUR q_10000_geom
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/de/ PTV 10000 POPULATION_UNIFORM q_10000_pop_uni ../graphs/population_grid/europe/
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/de/ PTV 10000 POPULATION_GEOMETRIC q_10000_pop_geom ../graphs/population_grid/europe/

RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_compare_potentials --release -- ../graphs/ptv/de/ q_10000_uni 72_corridor_lowerbound 20_multi_metrics
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_compare_potentials --release -- ../graphs/ptv/de/ q_10000_geom 72_corridor_lowerbound 20_multi_metrics
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_compare_potentials --release -- ../graphs/ptv/de/ q_10000_pop_uni 72_corridor_lowerbound 20_multi_metrics
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_compare_potentials --release -- ../graphs/ptv/de/ q_10000_pop_geom 72_corridor_lowerbound 20_multi_metrics

# Eur-17
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/ptv17-eur-car/ PTV 10000 UNIFORM q_10000_uni
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/ptv17-eur-car/ PTV 10000 GEOMETRIC_RUSH_HOUR q_10000_geom
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/ptv17-eur-car/ PTV 10000 POPULATION_UNIFORM q_10000_pop_uni ../graphs/population_grid/europe/
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/ptv17-eur-car/ PTV 10000 POPULATION_GEOMETRIC q_10000_pop_geom ../graphs/population_grid/europe/

RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_compare_potentials --release -- ../graphs/ptv/ptv17-eur-car/ q_10000_uni 72_corridor_lowerbound 20_multi_metrics
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_compare_potentials --release -- ../graphs/ptv/ptv17-eur-car/ q_10000_geom 72_corridor_lowerbound 20_multi_metrics
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_compare_potentials --release -- ../graphs/ptv/ptv17-eur-car/ q_10000_pop_uni 72_corridor_lowerbound 20_multi_metrics
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_compare_potentials --release -- ../graphs/ptv/ptv17-eur-car/ q_10000_pop_geom 72_corridor_lowerbound 20_multi_metrics

# Eur-20
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/ptv20-eur-car/ PTV 10000 UNIFORM q_10000_uni
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/ptv20-eur-car/ PTV 10000 GEOMETRIC_RUSH_HOUR q_10000_geom
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/ptv20-eur-car/ PTV 10000 POPULATION_UNIFORM q_10000_pop_uni ../graphs/population_grid/europe/
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/ptv20-eur-car/ PTV 10000 POPULATION_GEOMETRIC q_10000_pop_geom ../graphs/population_grid/europe/

RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_compare_potentials --release -- ../graphs/ptv/ptv20-eur-car/ q_10000_uni 72_corridor_lowerbound 20_multi_metrics
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_compare_potentials --release -- ../graphs/ptv/ptv20-eur-car/ q_10000_geom 72_corridor_lowerbound 20_multi_metrics
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_compare_potentials --release -- ../graphs/ptv/ptv20-eur-car/ q_10000_pop_uni 72_corridor_lowerbound 20_multi_metrics
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_compare_potentials --release -- ../graphs/ptv/ptv20-eur-car/ q_10000_pop_geom 72_corridor_lowerbound 20_multi_metrics

#### Experiment 3: Potential Runtimes on PTV Graphs, Dijkstra Rank Visualization
# only conducted on Eur-20 Graph, choosing the starting point (1) by population-density and (2) uniformly, (3) -> (1) with rush hour departure distribution, (4) -> (2) with rush hour
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/ptv20-eur-car/ PTV 1000 DIJKSTRA_RANK q_1000_dijkstra_rank 24
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/ptv20-eur-car/ PTV 1000 POPULATION_DIJKSTRA_RANK q_1000_pop_dijkstra_rank ../graphs/population_grid/europe/ 24
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/ptv20-eur-car/ PTV 1000 DIJKSTRA_RANK_RUSH_HOUR q_1000_dijkstra_rank_rh 24
cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/ptv/ptv20-eur-car/ PTV 1000 POPULATION_DIJKSTRA_RANK_RUSH_HOUR q_1000_pop_dijkstra_rank_rh ../graphs/population_grid/europe/ 24

RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_dijkstra_ranks --release -- ../graphs/ptv/ptv20-eur-car/ q_1000_dijkstra_rank dijkstra_rank_uni 72_corridor_lowerbound 20_multi_metrics
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_dijkstra_ranks --release -- ../graphs/ptv/ptv20-eur-car/ q_1000_pop_dijkstra_rank dijkstra_rank_pop 72_corridor_lowerbound 20_multi_metrics
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_dijkstra_ranks --release -- ../graphs/ptv/ptv20-eur-car/ q_1000_dijkstra_rank_rh dijkstra_rank_rh 72_corridor_lowerbound 20_multi_metrics
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin ptv_dijkstra_ranks --release -- ../graphs/ptv/ptv20-eur-car/ q_1000_pop_dijkstra_rank_rh dijkstra_rank_pop_rh 72_corridor_lowerbound 20_multi_metrics
















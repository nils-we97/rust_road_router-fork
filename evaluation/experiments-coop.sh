#### COOPERATIVE EXPERIMENTS

routing_kit= # set value
rust_base= # set value
inertial_flow_cutter= # set value

# download latest OSM dumps from Geofabrik

# generate graphs with RoutingKit
cd $routing_kit/bin
./osm_export_graph ../../graphs/osm/berlin_raw berlin-latest.osm.pbf
./osm_export_graph ../../graphs/osm/baden_wuerttemberg_raw baden-wuerttemberg-latest.osm.pbf

# additional graph preprocessing
cd $rust_base
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin preprocess_osm_graph --release -- ../graphs/osm/berlin_raw/ ../graphs/osm/berlin
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin preprocess_osm_graph --release -- ../graphs/osm/baden_wuerttemberg_raw/ ../graphs/osm/baden_wuerttemberg

# create node order with InertialFlowCutter
cd $inertial_flow_cutter
python3 inertialflowcutter_order.py ../graphs/osm/berlin/ ../graphs/osm/berlin/order
python3 inertialflowcutter_order.py ../graphs/osm/baden_wuerttemberg/ ../graphs/osm/baden_wuerttemberg/order

# generate queries
cd $rust_base
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/osm/berlin/ CAPACITY 500000 POPULATION_UNIFORM q_500k_pop_uni ../graphs/population_grid/germany
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/osm/berlin/ CAPACITY 1000000 POPULATION_UNIFORM q_1M_pop_uni ../graphs/population_grid/germany
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/osm/berlin/ CAPACITY 3000000 POPULATION_UNIFORM q_3M_pop_uni ../graphs/population_grid/germany
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/osm/baden_wuerttemberg/ CAPACITY 1000000 POPULATION_UNIFORM q_1M_pop_uni ../graphs/population_grid/germany
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin generate_queries --release -- ../graphs/osm/baden_wuerttemberg/ CAPACITY 2000000 POPULATION_UNIFORM q_2M_pop_uni ../graphs/population_grid/germany

########################################################################################################################

#### Experiment 4: Evaluate Storage Consumption of Capacity Servers
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin evaluate_cooperative_storage --release -- ../graphs/osm/berlin q_3M_pop_uni 500000,1000000,1500000,2000000,2500000,3000000 1,50,200
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin evaluate_cooperative_storage --release -- ../graphs/osm/baden_wuerttemberg/ q_2M_pop_uni 500000,1000000,2000000 1,50,200

#### Experiment 5: Compare potentials on cooperative graphs
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin evaluate_potential_quality_unordered --release -- ../graphs/osm/baden_wuerttemberg/ q_1M_pop_uni 50 10000 20 50000 72 100000
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin evaluate_potential_quality_ordered --release -- ../graphs/osm/berlin/ 200 q_500k_pop_uni,q_1M_pop_uni 20 50000 72 100000

#### Experiment 6: Compare Static (CCH) vs Time-Dependent Cooperative Routing
RUSTFLAGS='-C target-cpu=native' cargo +nightly run --package cooperative --bin compare_static_cooperative --release -- ../graphs/osm/baden_wuerttemberg/ q_2M_pop_uni 100000 1,50,200 0,20000,100000
use rust_road_router::datastr::graph::floating_time_dependent::{FlWeight, MergeBuffers, PartialATTF, PartialPiecewiseLinearFunction, TTFPoint, Timestamp};
use std::cmp::{max, min};

#[test]
fn test_partial_backward_profile() {
    let prev_edge_profile = [
        TTFPoint {
            at: Timestamp(0.0),
            val: FlWeight(4.023),
        },
        TTFPoint {
            at: Timestamp(86400.0),
            val: FlWeight(6.023),
        },
    ];

    let current_profile = [
        TTFPoint {
            at: Timestamp(917.278),
            val: FlWeight(0.0),
        },
        TTFPoint {
            at: Timestamp(3632.2780000000002),
            val: FlWeight(0.0),
        },
    ];

    let first = PartialATTF::Exact(PartialPiecewiseLinearFunction::new(&prev_edge_profile));
    let second = PartialATTF::Exact(PartialPiecewiseLinearFunction::new(&current_profile));

    let start_ts = backward_link_interval(&prev_edge_profile, Timestamp(917.278));
    let end_ts = backward_link_interval(&prev_edge_profile, Timestamp(3632.2780000000002));

    dbg!(start_ts, end_ts);

    //let result = first.link(&second, Timestamp(917.278 - 4.023), Timestamp(3632.2780000000002 - 4.023));
    let result = first.link(&second, start_ts, end_ts);
    dbg!(&result);
}

#[test]
fn merge_partial_profiles() {
    let mut linked = vec![
        TTFPoint {
            at: Timestamp(1055.903),
            val: FlWeight(150.4949999999999),
        },
        TTFPoint {
            at: Timestamp(1657.6879999999999),
            val: FlWeight(139.49500000000035),
        },
        TTFPoint {
            at: Timestamp(1700.7189999999998),
            val: FlWeight(149.5490000000002),
        },
        TTFPoint {
            at: Timestamp(2561.6340000000005),
            val: FlWeight(149.54899999999998),
        },
        TTFPoint {
            at: Timestamp(2564.7190000000005),
            val: FlWeight(149.49500000000035),
        },
        TTFPoint {
            at: Timestamp(4500.902999999998),
            val: FlWeight(140.4950000000017),
        },
    ];

    let mut label = vec![
        TTFPoint {
            at: Timestamp(1052.8780000000002),
            val: FlWeight(149.51999999999975),
        },
        TTFPoint {
            at: Timestamp(1697.663),
            val: FlWeight(149.5200000000002),
        },
        TTFPoint {
            at: Timestamp(1700.694),
            val: FlWeight(149.57400000000007),
        },
        TTFPoint {
            at: Timestamp(2561.6090000000004),
            val: FlWeight(149.57400000000007),
        },
        TTFPoint {
            at: Timestamp(2564.6940000000004),
            val: FlWeight(149.52000000000044),
        },
        TTFPoint {
            at: Timestamp(4523.877999999998),
            val: FlWeight(149.52000000000226),
        },
    ];

    // adjust start/end points
    let start_ts = min(label[0].at, linked[0].at);
    if label[0].at.fuzzy_lt(linked[0].at) {
        // add FIFO-conform weight at beginning, equal to "waiting"
        linked.insert(
            0,
            TTFPoint {
                at: label[0].at,
                val: linked[0].val + linked[0].at - label[0].at,
            },
        );
    } else if linked[0].at.fuzzy_lt(label[0].at) {
        label.insert(
            0,
            TTFPoint {
                at: linked[0].at,
                val: label[0].val + label[0].at - linked[0].at,
            },
        );
    }

    let end_ts = min(label.last().unwrap().at, linked.last().unwrap().at);
    /*let label_last = label.last().unwrap();
    let linked_last = linked.last().unwrap();

    if label_last.at.fuzzy_lt(linked_last.at) {
        label.push(TTFPoint {
            at: linked_last.at,
            val: label_last.val + linked_last.at - label_last.at,
        });
    } else if linked_last.at.fuzzy_lt(label_last.at) {
        linked.push(TTFPoint {
            at: label_last.at,
            val: linked_last.val + label_last.at - linked_last.at,
        });
    }*/

    let first = PartialPiecewiseLinearFunction::new(&label);
    let second = PartialPiecewiseLinearFunction::new(&linked);

    /*let start_ts = max(linked[0].at, label[0].at);
    let end_ts = min(linked.last().unwrap().at, label.last().unwrap().at);*/

    let result = first.merge(&second, start_ts, end_ts.at, &mut Vec::new());

    dbg!(&result);
}

fn backward_link_interval(prev_edge_ipps: &[TTFPoint], reference_ts: Timestamp) -> Timestamp {
    // find profile breakpoints in relevant interval
    let idx_start = prev_edge_ipps.binary_search_by_key(&reference_ts, |p| p.at + p.val);
    if idx_start.is_ok() {
        // easy, but very rare case: previous edge contains TTF point that directly links with start of current profile
        prev_edge_ipps[idx_start.unwrap()].at
    } else {
        // more complex, general case: interpolate between the two point that are in between the desired solution
        // goal: for interpolation function f, find timestamp t such that: t + f(t) = reference_ts
        let ipp1 = &prev_edge_ipps[idx_start.unwrap_err() - 1];
        let ipp2 = &prev_edge_ipps[idx_start.unwrap_err()];

        // fortunately, there is a closed solution formula to obtain the desired point
        // get slope of TTF interval
        let delta = (ipp2.val.0 - ipp1.val.0) / (ipp2.at.0 - ipp1.at.0);
        assert_ne!(delta, -1.0, "slope must not be -1 here!");

        Timestamp((reference_ts.0 - ipp1.val.0 + ipp1.at.0 * delta) / (1.0 + delta))
    }
}

#[test]
fn test_broken_link() {
    let prev_edge_ipps = vec![
        TTFPoint {
            at: Timestamp(0.0),
            val: FlWeight(1314.45),
        },
        TTFPoint {
            at: Timestamp(86400.0),
            val: FlWeight(1314.45),
        },
    ];

    let label = vec![
        TTFPoint {
            at: Timestamp(26852.35500000001),
            val: FlWeight(1438.880999999994),
        },
        TTFPoint {
            at: Timestamp(26853.555000000008),
            val: FlWeight(1438.760999999995),
        },
        TTFPoint {
            at: Timestamp(26854.719000000012),
            val: FlWeight(1438.638999999992),
        },
        TTFPoint {
            at: Timestamp(26855.76400000001),
            val: FlWeight(1438.6059999999925),
        },
        TTFPoint {
            at: Timestamp(26856.93100000001),
            val: FlWeight(1438.491999999991),
        },
        TTFPoint {
            at: Timestamp(26858.098000000013),
            val: FlWeight(1438.3649999999907),
        },
        TTFPoint {
            at: Timestamp(26861.63100000001),
            val: FlWeight(1437.9589999999953),
        },
        TTFPoint {
            at: Timestamp(26862.78500000001),
            val: FlWeight(1437.8419999999933),
        },
        TTFPoint {
            at: Timestamp(26863.82300000001),
            val: FlWeight(1437.8039999999928),
        },
        TTFPoint {
            at: Timestamp(26865.911000000007),
            val: FlWeight(1437.7229999999945),
        },
        TTFPoint {
            at: Timestamp(26867.11100000001),
            val: FlWeight(1437.6029999999919),
        },
        TTFPoint {
            at: Timestamp(26868.286000000007),
            val: FlWeight(1437.474999999995),
        },
        TTFPoint {
            at: Timestamp(26870.380000000012),
            val: FlWeight(1437.437999999991),
        },
        TTFPoint {
            at: Timestamp(26872.76000000001),
            val: FlWeight(1437.1659999999938),
        },
        TTFPoint {
            at: Timestamp(26876.918000000012),
            val: FlWeight(1437.0959999999905),
        },
        TTFPoint {
            at: Timestamp(26878.100000000013),
            val: FlWeight(1436.955999999991),
        },
        TTFPoint {
            at: Timestamp(26880.18600000001),
            val: FlWeight(1436.8979999999974),
        },
        TTFPoint {
            at: Timestamp(26881.214000000007),
            val: FlWeight(1436.8699999999953),
        },
        TTFPoint {
            at: Timestamp(26885.392000000007),
            val: FlWeight(1436.7969999999987),
        },
        TTFPoint {
            at: Timestamp(26886.437000000005),
            val: FlWeight(1436.7639999999992),
        },
        TTFPoint {
            at: Timestamp(26889.503000000004),
            val: FlWeight(1436.7639999999992),
        },
        TTFPoint {
            at: Timestamp(26893.552000000003),
            val: FlWeight(1436.7639999999992),
        },
        TTFPoint {
            at: Timestamp(26895.601000000002),
            val: FlWeight(1436.7330000000002),
        },
        TTFPoint {
            at: Timestamp(26899.626000000004),
            val: FlWeight(1436.7330000000002),
        },
        TTFPoint {
            at: Timestamp(26900.637000000002),
            val: FlWeight(1436.7220000000016),
        },
        TTFPoint {
            at: Timestamp(26902.649000000005),
            val: FlWeight(1436.7220000000016),
        },
        TTFPoint {
            at: Timestamp(26904.681000000004),
            val: FlWeight(1436.6900000000023),
        },
        TTFPoint {
            at: Timestamp(26905.70100000001),
            val: FlWeight(1436.6729999999952),
        },
        TTFPoint {
            at: Timestamp(26908.797000000006),
            val: FlWeight(1436.612000000001),
        },
        TTFPoint {
            at: Timestamp(26936.995000000003),
            val: FlWeight(1436.612000000001),
        },
        TTFPoint {
            at: Timestamp(26943.102),
            val: FlWeight(1436.5050000000047),
        },
        TTFPoint {
            at: Timestamp(27122.340000000004),
            val: FlWeight(1436.5049999999974),
        },
        TTFPoint {
            at: Timestamp(27123.420000000006),
            val: FlWeight(1436.5049999999974),
        },
        TTFPoint {
            at: Timestamp(27124.448000000004),
            val: FlWeight(1436.5049999999974),
        },
        TTFPoint {
            at: Timestamp(27125.52300000001),
            val: FlWeight(1436.5049999999937),
        },
        TTFPoint {
            at: Timestamp(27126.581000000006),
            val: FlWeight(1436.5369999999966),
        },
        TTFPoint {
            at: Timestamp(27127.67300000001),
            val: FlWeight(1436.5369999999893),
        },
        TTFPoint {
            at: Timestamp(27128.75300000001),
            val: FlWeight(1436.5369999999893),
        },
        TTFPoint {
            at: Timestamp(27129.841000000008),
            val: FlWeight(1436.536999999993),
        },
        TTFPoint {
            at: Timestamp(27130.92100000001),
            val: FlWeight(1436.536999999993),
        },
        TTFPoint {
            at: Timestamp(27132.005000000012),
            val: FlWeight(1436.5369999999893),
        },
        TTFPoint {
            at: Timestamp(27138.45700000001),
            val: FlWeight(1436.536999999993),
        },
        TTFPoint {
            at: Timestamp(27139.485000000008),
            val: FlWeight(1436.536999999993),
        },
        TTFPoint {
            at: Timestamp(27140.49500000001),
            val: FlWeight(1436.5369999999893),
        },
        TTFPoint {
            at: Timestamp(27142.56700000001),
            val: FlWeight(1436.5689999999922),
        },
        TTFPoint {
            at: Timestamp(27149.202000000005),
            val: FlWeight(1436.983999999993),
        },
        TTFPoint {
            at: Timestamp(27150.225000000006),
            val: FlWeight(1437.005999999994),
        },
        TTFPoint {
            at: Timestamp(27151.26200000001),
            val: FlWeight(1437.0239999999867),
        },
        TTFPoint {
            at: Timestamp(27154.286000000007),
            val: FlWeight(1437.0849999999919),
        },
        TTFPoint {
            at: Timestamp(27158.33600000001),
            val: FlWeight(1437.1579999999885),
        },
        TTFPoint {
            at: Timestamp(27160.426000000007),
            val: FlWeight(1437.1919999999918),
        },
        TTFPoint {
            at: Timestamp(27161.430000000008),
            val: FlWeight(1437.215999999993),
        },
        TTFPoint {
            at: Timestamp(27162.447000000007),
            val: FlWeight(1437.2389999999905),
        },
        TTFPoint {
            at: Timestamp(27170.84700000001),
            val: FlWeight(1437.406999999992),
        },
        TTFPoint {
            at: Timestamp(27171.84700000001),
            val: FlWeight(1437.4249999999884),
        },
        TTFPoint {
            at: Timestamp(27173.92100000001),
            val: FlWeight(1437.4609999999884),
        },
        TTFPoint {
            at: Timestamp(27175.95000000001),
            val: FlWeight(1437.4609999999848),
        },
        TTFPoint {
            at: Timestamp(27176.97800000001),
            val: FlWeight(1437.4799999999886),
        },
        TTFPoint {
            at: Timestamp(27181.420000000006),
            val: FlWeight(1437.6779999999926),
        },
        TTFPoint {
            at: Timestamp(27183.441000000006),
            val: FlWeight(1437.6779999999926),
        },
        TTFPoint {
            at: Timestamp(27184.486000000004),
            val: FlWeight(1437.6779999999962),
        },
        TTFPoint {
            at: Timestamp(27185.521000000008),
            val: FlWeight(1437.6909999999916),
        },
        TTFPoint {
            at: Timestamp(27186.549000000006),
            val: FlWeight(1437.6909999999916),
        },
        TTFPoint {
            at: Timestamp(27188.598000000005),
            val: FlWeight(1437.6909999999916),
        },
        TTFPoint {
            at: Timestamp(27188.958000000006),
            val: FlWeight(1437.6909999999953),
        },
        TTFPoint {
            at: Timestamp(27197.558000000005),
            val: FlWeight(1437.8939999999966),
        },
        TTFPoint {
            at: Timestamp(27198.624000000003),
            val: FlWeight(1437.8939999999966),
        },
        TTFPoint {
            at: Timestamp(27203.088000000003),
            val: FlWeight(1438.079999999998),
        },
        TTFPoint {
            at: Timestamp(27204.116),
            val: FlWeight(1438.0949999999975),
        },
        TTFPoint {
            at: Timestamp(27205.205),
            val: FlWeight(1438.109999999997),
        },
        TTFPoint {
            at: Timestamp(27206.271),
            val: FlWeight(1438.1299999999974),
        },
        TTFPoint {
            at: Timestamp(27207.502),
            val: FlWeight(1438.1990000000005),
        },
        TTFPoint {
            at: Timestamp(27210.727),
            val: FlWeight(1438.1990000000005),
        },
        TTFPoint {
            at: Timestamp(27215.977),
            val: FlWeight(1438.3100000000013),
        },
        TTFPoint {
            at: Timestamp(27219.201999999997),
            val: FlWeight(1438.3780000000006),
        },
        TTFPoint {
            at: Timestamp(27220.221999999994),
            val: FlWeight(1438.3780000000042),
        },
        TTFPoint {
            at: Timestamp(27221.276999999995),
            val: FlWeight(1438.4150000000045),
        },
        TTFPoint {
            at: Timestamp(27222.333),
            val: FlWeight(1438.4509999999973),
        },
        TTFPoint {
            at: Timestamp(27224.535000000003),
            val: FlWeight(1438.4509999999973),
        },
        TTFPoint {
            at: Timestamp(27225.409),
            val: FlWeight(1438.4509999999973),
        },
        TTFPoint {
            at: Timestamp(27225.575000000004),
            val: FlWeight(1438.4615122873329),
        },
        TTFPoint {
            at: Timestamp(27226.484512015755),
            val: FlWeight(1438.5004879842454),
        },
        TTFPoint {
            at: Timestamp(27227.519438804986),
            val: FlWeight(1438.4935611950132),
        },
        TTFPoint {
            at: Timestamp(27228.59820978332),
            val: FlWeight(1438.5117902166821),
        },
        TTFPoint {
            at: Timestamp(27229.668823703214),
            val: FlWeight(1438.536176296784),
        },
        TTFPoint {
            at: Timestamp(27230.708848653972),
            val: FlWeight(1438.5161513460262),
        },
        TTFPoint {
            at: Timestamp(27235.022912934997),
            val: FlWeight(1438.5870870650033),
        },
        TTFPoint {
            at: Timestamp(27238.151144845695),
            val: FlWeight(1438.5268551543013),
        },
        TTFPoint {
            at: Timestamp(27241.104),
            val: FlWeight(1438.5287027027043),
        },
        TTFPoint {
            at: Timestamp(27243.536),
            val: FlWeight(1438.5779999999977),
        },
        TTFPoint {
            at: Timestamp(27250.735999999997),
            val: FlWeight(1438.5780000000013),
        },
        TTFPoint {
            at: Timestamp(27252.896),
            val: FlWeight(1438.5779999999977),
        },
        TTFPoint {
            at: Timestamp(27256.979999999996),
            val: FlWeight(1438.5780000000013),
        },
        TTFPoint {
            at: Timestamp(27258.015),
            val: FlWeight(1438.5929999999971),
        },
        TTFPoint {
            at: Timestamp(27265.072999999997),
            val: FlWeight(1438.5930000000008),
        },
        TTFPoint {
            at: Timestamp(27269.1),
            val: FlWeight(1438.6620000000003),
        },
        TTFPoint {
            at: Timestamp(27270.117),
            val: FlWeight(1438.6620000000003),
        },
        TTFPoint {
            at: Timestamp(27272.182999999997),
            val: FlWeight(1438.701000000001),
        },
        TTFPoint {
            at: Timestamp(27296.320999999996),
            val: FlWeight(1438.7010000000046),
        },
        TTFPoint {
            at: Timestamp(27302.657),
            val: FlWeight(1438.701000000001),
        },
        TTFPoint {
            at: Timestamp(27313.032999999996),
            val: FlWeight(1438.5010000000075),
        },
        TTFPoint {
            at: Timestamp(27316.078999999998),
            val: FlWeight(1438.5010000000038),
        },
        TTFPoint {
            at: Timestamp(27324.266999999993),
            val: FlWeight(1438.343000000008),
        },
        TTFPoint {
            at: Timestamp(27325.274999999994),
            val: FlWeight(1438.343000000008),
        },
        TTFPoint {
            at: Timestamp(27326.285999999993),
            val: FlWeight(1438.3320000000094),
        },
        TTFPoint {
            at: Timestamp(27361.739999999998),
            val: FlWeight(1438.3320000000058),
        },
        TTFPoint {
            at: Timestamp(27367.244999999995),
            val: FlWeight(1438.2270000000062),
        },
        TTFPoint {
            at: Timestamp(27377.621),
            val: FlWeight(1438.2270000000026),
        },
        TTFPoint {
            at: Timestamp(27385.858999999997),
            val: FlWeight(1438.2270000000062),
        },
        TTFPoint {
            at: Timestamp(27394.117),
            val: FlWeight(1438.2270000000062),
        },
        TTFPoint {
            at: Timestamp(27399.627),
            val: FlWeight(1438.3420000000006),
        },
        TTFPoint {
            at: Timestamp(27402.686999999998),
            val: FlWeight(1438.3930000000073),
        },
        TTFPoint {
            at: Timestamp(27403.752999999997),
            val: FlWeight(1438.3930000000073),
        },
        TTFPoint {
            at: Timestamp(27404.752999999997),
            val: FlWeight(1438.4210000000057),
        },
        TTFPoint {
            at: Timestamp(27407.879999999997),
            val: FlWeight(1438.4210000000057),
        },
        TTFPoint {
            at: Timestamp(27408.907999999996),
            val: FlWeight(1438.4210000000057),
        },
        TTFPoint {
            at: Timestamp(27409.949999999997),
            val: FlWeight(1438.4210000000057),
        },
        TTFPoint {
            at: Timestamp(27413.1),
            val: FlWeight(1438.421000000002),
        },
        TTFPoint {
            at: Timestamp(27415.157),
            val: FlWeight(1438.4580000000024),
        },
        TTFPoint {
            at: Timestamp(27416.237),
            val: FlWeight(1438.4580000000024),
        },
        TTFPoint {
            at: Timestamp(27419.429000000004),
            val: FlWeight(1438.5830000000024),
        },
        TTFPoint {
            at: Timestamp(27422.654000000002),
            val: FlWeight(1438.7230000000018),
        },
        TTFPoint {
            at: Timestamp(27422.879),
            val: FlWeight(1438.7230000000018),
        },
        TTFPoint {
            at: Timestamp(27434.629),
            val: FlWeight(1438.7230000000018),
        },
        TTFPoint {
            at: Timestamp(27436.686),
            val: FlWeight(1438.7230000000018),
        },
        TTFPoint {
            at: Timestamp(27437.714),
            val: FlWeight(1438.7400000000016),
        },
        TTFPoint {
            at: Timestamp(27438.803999999996),
            val: FlWeight(1438.7400000000052),
        },
        TTFPoint {
            at: Timestamp(27440.843999999997),
            val: FlWeight(1438.7740000000085),
        },
        TTFPoint {
            at: Timestamp(27442.875999999997),
            val: FlWeight(1438.8070000000043),
        },
        TTFPoint {
            at: Timestamp(27448.311),
            val: FlWeight(1438.916000000001),
        },
        TTFPoint {
            at: Timestamp(27449.398999999998),
            val: FlWeight(1438.9290000000074),
        },
        TTFPoint {
            at: Timestamp(27451.595),
            val: FlWeight(1438.929),
        },
        TTFPoint {
            at: Timestamp(27452.675000000003),
            val: FlWeight(1438.956000000002),
        },
        TTFPoint {
            at: Timestamp(27454.722),
            val: FlWeight(1438.956000000002),
        },
        TTFPoint {
            at: Timestamp(27455.822),
            val: FlWeight(1438.956000000002),
        },
        TTFPoint {
            at: Timestamp(27459.106),
            val: FlWeight(1438.956000000002),
        },
        TTFPoint {
            at: Timestamp(27460.305999999997),
            val: FlWeight(1438.9560000000056),
        },
        TTFPoint {
            at: Timestamp(27463.366),
            val: FlWeight(1438.956000000002),
        },
        TTFPoint {
            at: Timestamp(27464.394),
            val: FlWeight(1438.956000000002),
        },
        TTFPoint {
            at: Timestamp(27471.593999999997),
            val: FlWeight(1438.9560000000056),
        },
        TTFPoint {
            at: Timestamp(27472.674),
            val: FlWeight(1438.9780000000064),
        },
        TTFPoint {
            at: Timestamp(27474.722),
            val: FlWeight(1438.9780000000028),
        },
        TTFPoint {
            at: Timestamp(27476.913),
            val: FlWeight(1439.0270000000019),
        },
        TTFPoint {
            at: Timestamp(27478.019999999997),
            val: FlWeight(1439.0720000000074),
        },
        TTFPoint {
            at: Timestamp(27481.104999999996),
            val: FlWeight(1439.0720000000074),
        },
        TTFPoint {
            at: Timestamp(27483.305),
            val: FlWeight(1439.0720000000038),
        },
        TTFPoint {
            at: Timestamp(27485.352),
            val: FlWeight(1439.0720000000038),
        },
        TTFPoint {
            at: Timestamp(27486.432),
            val: FlWeight(1439.1090000000004),
        },
        TTFPoint {
            at: Timestamp(27487.525),
            val: FlWeight(1439.1090000000004),
        },
        TTFPoint {
            at: Timestamp(27488.631999999998),
            val: FlWeight(1439.109000000004),
        },
        TTFPoint {
            at: Timestamp(27489.712),
            val: FlWeight(1439.1310000000012),
        },
        TTFPoint {
            at: Timestamp(27490.732000000004),
            val: FlWeight(1439.1470000000008),
        },
        TTFPoint {
            at: Timestamp(27491.817000000003),
            val: FlWeight(1439.1650000000009),
        },
        TTFPoint {
            at: Timestamp(27495.034),
            val: FlWeight(1439.2340000000004),
        },
        TTFPoint {
            at: Timestamp(27498.327000000005),
            val: FlWeight(1439.3060000000005),
        },
        TTFPoint {
            at: Timestamp(27500.462),
            val: FlWeight(1439.3810000000012),
        },
        TTFPoint {
            at: Timestamp(27501.542),
            val: FlWeight(1439.4079999999994),
        },
        TTFPoint {
            at: Timestamp(27503.560000000005),
            val: FlWeight(1439.4079999999958),
        },
        TTFPoint {
            at: Timestamp(27504.618000000002),
            val: FlWeight(1439.439999999995),
        },
        TTFPoint {
            at: Timestamp(27505.698000000004),
            val: FlWeight(1439.4769999999953),
        },
        TTFPoint {
            at: Timestamp(27506.786000000007),
            val: FlWeight(1439.5029999999933),
        },
        TTFPoint {
            at: Timestamp(27507.82600000001),
            val: FlWeight(1439.5029999999933),
        },
        TTFPoint {
            at: Timestamp(27508.84700000001),
            val: FlWeight(1439.5169999999925),
        },
        TTFPoint {
            at: Timestamp(27509.894000000008),
            val: FlWeight(1439.5559999999932),
        },
        TTFPoint {
            at: Timestamp(27510.93400000001),
            val: FlWeight(1439.603999999992),
        },
        TTFPoint {
            at: Timestamp(27511.962000000007),
            val: FlWeight(1439.6829999999936),
        },
        TTFPoint {
            at: Timestamp(27513.002000000008),
            val: FlWeight(1439.7359999999935),
        },
        TTFPoint {
            at: Timestamp(27515.02300000001),
            val: FlWeight(1439.7359999999935),
        },
        TTFPoint {
            at: Timestamp(27516.085000000006),
            val: FlWeight(1439.7539999999972),
        },
        TTFPoint {
            at: Timestamp(27517.13500000001),
            val: FlWeight(1439.7759999999907),
        },
        TTFPoint {
            at: Timestamp(27518.155000000006),
            val: FlWeight(1439.775999999998),
        },
        TTFPoint {
            at: Timestamp(27519.183000000005),
            val: FlWeight(1439.7909999999974),
        },
        TTFPoint {
            at: Timestamp(27520.217000000004),
            val: FlWeight(1439.8029999999962),
        },
        TTFPoint {
            at: Timestamp(27521.257000000005),
            val: FlWeight(1439.8259999999973),
        },
        TTFPoint {
            at: Timestamp(27523.36200000001),
            val: FlWeight(1439.8669999999947),
        },
        TTFPoint {
            at: Timestamp(27524.409000000007),
            val: FlWeight(1439.885999999995),
        },
        TTFPoint {
            at: Timestamp(27525.443000000007),
            val: FlWeight(1439.9079999999958),
        },
        TTFPoint {
            at: Timestamp(27526.471000000005),
            val: FlWeight(1439.9869999999974),
        },
        TTFPoint {
            at: Timestamp(27527.511000000006),
            val: FlWeight(1439.997999999996),
        },
        TTFPoint {
            at: Timestamp(27528.546000000002),
            val: FlWeight(1440.0269999999982),
        },
        TTFPoint {
            at: Timestamp(27529.584000000003),
            val: FlWeight(1440.0470000000023),
        },
        TTFPoint {
            at: Timestamp(27530.629),
            val: FlWeight(1440.0640000000021),
        },
        TTFPoint {
            at: Timestamp(27531.657),
            val: FlWeight(1440.0920000000006),
        },
        TTFPoint {
            at: Timestamp(27532.677000000003),
            val: FlWeight(1440.091999999997),
        },
        TTFPoint {
            at: Timestamp(27533.712),
            val: FlWeight(1440.1180000000022),
        },
        TTFPoint {
            at: Timestamp(27537.876000000004),
            val: FlWeight(1440.2019999999975),
        },
        TTFPoint {
            at: Timestamp(27538.876000000004),
            val: FlWeight(1440.2159999999967),
        },
        TTFPoint {
            at: Timestamp(27539.919),
            val: FlWeight(1440.2160000000003),
        },
        TTFPoint {
            at: Timestamp(27540.947),
            val: FlWeight(1440.2160000000003),
        },
        TTFPoint {
            at: Timestamp(27541.990000000005),
            val: FlWeight(1440.2159999999967),
        },
        TTFPoint {
            at: Timestamp(27543.070000000007),
            val: FlWeight(1440.2159999999967),
        },
        TTFPoint {
            at: Timestamp(27544.15000000001),
            val: FlWeight(1440.2159999999967),
        },
        TTFPoint {
            at: Timestamp(27546.236000000004),
            val: FlWeight(1440.2159999999967),
        },
        TTFPoint {
            at: Timestamp(27547.278000000006),
            val: FlWeight(1440.2439999999951),
        },
        TTFPoint {
            at: Timestamp(27548.32100000001),
            val: FlWeight(1440.2439999999915),
        },
        TTFPoint {
            at: Timestamp(27549.363000000012),
            val: FlWeight(1440.2439999999915),
        },
        TTFPoint {
            at: Timestamp(27550.413000000008),
            val: FlWeight(1440.2439999999988),
        },
        TTFPoint {
            at: Timestamp(27551.479000000007),
            val: FlWeight(1440.2439999999951),
        },
        TTFPoint {
            at: Timestamp(27552.517000000007),
            val: FlWeight(1440.2439999999988),
        },
        TTFPoint {
            at: Timestamp(27553.564000000006),
            val: FlWeight(1440.2439999999988),
        },
        TTFPoint {
            at: Timestamp(27554.592000000004),
            val: FlWeight(1440.2439999999988),
        },
        TTFPoint {
            at: Timestamp(27559.636000000006),
            val: FlWeight(1440.2439999999988),
        },
        TTFPoint {
            at: Timestamp(27562.75600000001),
            val: FlWeight(1440.2439999999951),
        },
        TTFPoint {
            at: Timestamp(27577.92700000001),
            val: FlWeight(1440.2439999999951),
        },
        TTFPoint {
            at: Timestamp(27578.95500000001),
            val: FlWeight(1440.2259999999987),
        },
        TTFPoint {
            at: Timestamp(27580.005000000005),
            val: FlWeight(1440.184000000001),
        },
        TTFPoint {
            at: Timestamp(27581.20500000001),
            val: FlWeight(1440.1839999999975),
        },
        TTFPoint {
            at: Timestamp(27582.405000000006),
            val: FlWeight(1440.0119999999988),
        },
        TTFPoint {
            at: Timestamp(27590.41700000001),
            val: FlWeight(1440.0119999999952),
        },
        TTFPoint {
            at: Timestamp(27599.754000000008),
            val: FlWeight(1440.0119999999952),
        },
        TTFPoint {
            at: Timestamp(27603.804000000004),
            val: FlWeight(1440.0119999999988),
        },
        TTFPoint {
            at: Timestamp(27604.849000000002),
            val: FlWeight(1440.0469999999987),
        },
        TTFPoint {
            at: Timestamp(27607.877),
            val: FlWeight(1440.0469999999987),
        },
        TTFPoint {
            at: Timestamp(27617.237),
            val: FlWeight(1440.3689999999988),
        },
        TTFPoint {
            at: Timestamp(27619.331000000006),
            val: FlWeight(1440.447999999993),
        },
        TTFPoint {
            at: Timestamp(27620.351000000002),
            val: FlWeight(1440.4479999999967),
        },
        TTFPoint {
            at: Timestamp(27621.439000000006),
            val: FlWeight(1440.447999999993),
        },
        TTFPoint {
            at: Timestamp(27623.566000000006),
            val: FlWeight(1440.4809999999925),
        },
        TTFPoint {
            at: Timestamp(27624.646000000008),
            val: FlWeight(1440.4809999999925),
        },
        TTFPoint {
            at: Timestamp(27625.661000000007),
            val: FlWeight(1440.4809999999889),
        },
        TTFPoint {
            at: Timestamp(27629.74100000001),
            val: FlWeight(1440.4809999999889),
        },
        TTFPoint {
            at: Timestamp(27630.78100000001),
            val: FlWeight(1440.4809999999925),
        },
        TTFPoint {
            at: Timestamp(27631.80900000001),
            val: FlWeight(1440.4809999999925),
        },
        TTFPoint {
            at: Timestamp(27632.897000000004),
            val: FlWeight(1440.4809999999961),
        },
        TTFPoint {
            at: Timestamp(27633.960000000006),
            val: FlWeight(1440.4929999999913),
        },
        TTFPoint {
            at: Timestamp(27635.015000000007),
            val: FlWeight(1440.5109999999913),
        },
        TTFPoint {
            at: Timestamp(27636.073000000004),
            val: FlWeight(1440.510999999995),
        },
        TTFPoint {
            at: Timestamp(27637.101000000002),
            val: FlWeight(1440.510999999995),
        },
        TTFPoint {
            at: Timestamp(27638.121000000006),
            val: FlWeight(1440.5109999999913),
        },
        TTFPoint {
            at: Timestamp(27639.143000000004),
            val: FlWeight(1440.510999999995),
        },
        TTFPoint {
            at: Timestamp(27640.223000000005),
            val: FlWeight(1440.510999999995),
        },
        TTFPoint {
            at: Timestamp(27644.51200000001),
            val: FlWeight(1440.5109999999913),
        },
        TTFPoint {
            at: Timestamp(27645.57800000001),
            val: FlWeight(1440.5109999999913),
        },
        TTFPoint {
            at: Timestamp(27647.61200000001),
            val: FlWeight(1440.5109999999913),
        },
        TTFPoint {
            at: Timestamp(27648.670000000006),
            val: FlWeight(1440.510999999995),
        },
        TTFPoint {
            at: Timestamp(27650.68800000001),
            val: FlWeight(1440.5109999999913),
        },
        TTFPoint {
            at: Timestamp(27654.96300000001),
            val: FlWeight(1440.5109999999913),
        },
        TTFPoint {
            at: Timestamp(27659.043000000012),
            val: FlWeight(1440.5109999999913),
        },
        TTFPoint {
            at: Timestamp(27660.10100000001),
            val: FlWeight(1440.510999999995),
        },
        TTFPoint {
            at: Timestamp(27661.15400000001),
            val: FlWeight(1440.5379999999932),
        },
        TTFPoint {
            at: Timestamp(27662.204000000012),
            val: FlWeight(1440.5599999999904),
        },
        TTFPoint {
            at: Timestamp(27664.36400000001),
            val: FlWeight(1440.6229999999923),
        },
        TTFPoint {
            at: Timestamp(27666.38900000001),
            val: FlWeight(1440.6879999999946),
        },
        TTFPoint {
            at: Timestamp(27667.439000000013),
            val: FlWeight(1440.7329999999893),
        },
        TTFPoint {
            at: Timestamp(27668.481000000014),
            val: FlWeight(1440.7749999999905),
        },
        TTFPoint {
            at: Timestamp(27669.509000000013),
            val: FlWeight(1440.7749999999905),
        },
        TTFPoint {
            at: Timestamp(27670.55900000001),
            val: FlWeight(1440.8199999999924),
        },
        TTFPoint {
            at: Timestamp(27671.60100000001),
            val: FlWeight(1440.8619999999937),
        },
        TTFPoint {
            at: Timestamp(27672.653000000013),
            val: FlWeight(1440.8949999999895),
        },
        TTFPoint {
            at: Timestamp(27674.70900000001),
            val: FlWeight(1440.8949999999895),
        },
        TTFPoint {
            at: Timestamp(27675.76400000001),
            val: FlWeight(1440.9129999999896),
        },
        TTFPoint {
            at: Timestamp(27677.83000000001),
            val: FlWeight(1440.9129999999896),
        },
        TTFPoint {
            at: Timestamp(27678.858000000007),
            val: FlWeight(1440.9129999999896),
        },
        TTFPoint {
            at: Timestamp(27680.982000000004),
            val: FlWeight(1440.9489999999932),
        },
        TTFPoint {
            at: Timestamp(27682.052000000003),
            val: FlWeight(1440.9789999999957),
        },
        TTFPoint {
            at: Timestamp(27683.118000000002),
            val: FlWeight(1440.9989999999962),
        },
        TTFPoint {
            at: Timestamp(27684.138000000006),
            val: FlWeight(1440.9989999999889),
        },
        TTFPoint {
            at: Timestamp(27685.178000000007),
            val: FlWeight(1440.9989999999889),
        },
        TTFPoint {
            at: Timestamp(27687.218000000008),
            val: FlWeight(1440.9989999999889),
        },
        TTFPoint {
            at: Timestamp(27688.29800000001),
            val: FlWeight(1440.9989999999889),
        },
        TTFPoint {
            at: Timestamp(27689.310000000005),
            val: FlWeight(1441.031999999992),
        },
        TTFPoint {
            at: Timestamp(27692.34500000001),
            val: FlWeight(1441.0929999999898),
        },
        TTFPoint {
            at: Timestamp(27693.39800000001),
            val: FlWeight(1441.119999999988),
        },
        TTFPoint {
            at: Timestamp(27694.416000000005),
            val: FlWeight(1441.1199999999917),
        },
        TTFPoint {
            at: Timestamp(27695.496000000006),
            val: FlWeight(1441.1199999999917),
        },
        TTFPoint {
            at: Timestamp(27698.52700000001),
            val: FlWeight(1441.119999999988),
        },
        TTFPoint {
            at: Timestamp(27699.541000000005),
            val: FlWeight(1441.1199999999953),
        },
        TTFPoint {
            at: Timestamp(27700.569000000003),
            val: FlWeight(1441.1499999999942),
        },
        TTFPoint {
            at: Timestamp(27701.603000000003),
            val: FlWeight(1441.171999999995),
        },
        TTFPoint {
            at: Timestamp(27702.739),
            val: FlWeight(1441.2699999999968),
        },
        TTFPoint {
            at: Timestamp(27703.767),
            val: FlWeight(1441.295000000002),
        },
        TTFPoint {
            at: Timestamp(27704.795),
            val: FlWeight(1441.3199999999997),
        },
        TTFPoint {
            at: Timestamp(27705.936),
            val: FlWeight(1441.4429999999957),
        },
        TTFPoint {
            at: Timestamp(27706.978000000003),
            val: FlWeight(1441.4709999999977),
        },
        TTFPoint {
            at: Timestamp(27708.114),
            val: FlWeight(1441.5689999999959),
        },
        TTFPoint {
            at: Timestamp(27709.314),
            val: FlWeight(1441.5689999999995),
        },
        TTFPoint {
            at: Timestamp(27710.334000000003),
            val: FlWeight(1441.5859999999993),
        },
        TTFPoint {
            at: Timestamp(27711.491),
            val: FlWeight(1441.6749999999993),
        },
        TTFPoint {
            at: Timestamp(27712.557),
            val: FlWeight(1441.6749999999993),
        },
        TTFPoint {
            at: Timestamp(27714.862999999998),
            val: FlWeight(1441.8700000000063),
        },
        TTFPoint {
            at: Timestamp(27715.907999999996),
            val: FlWeight(1441.8870000000024),
        },
        TTFPoint {
            at: Timestamp(27716.947999999997),
            val: FlWeight(1441.9690000000046),
        },
        TTFPoint {
            at: Timestamp(27717.949999999997),
            val: FlWeight(1441.9820000000036),
        },
        TTFPoint {
            at: Timestamp(27719.03),
            val: FlWeight(1442.0380000000005),
        },
        TTFPoint {
            at: Timestamp(27720.11),
            val: FlWeight(1442.1579999999994),
        },
        TTFPoint {
            at: Timestamp(27721.152000000002),
            val: FlWeight(1442.2309999999998),
        },
        TTFPoint {
            at: Timestamp(27722.164000000004),
            val: FlWeight(1442.2639999999956),
        },
        TTFPoint {
            at: Timestamp(27723.217000000004),
            val: FlWeight(1442.3179999999957),
        },
        TTFPoint {
            at: Timestamp(27724.257000000005),
            val: FlWeight(1442.3999999999978),
        },
        TTFPoint {
            at: Timestamp(27727.384000000005),
            val: FlWeight(1442.6199999999953),
        },
        TTFPoint {
            at: Timestamp(27728.421000000002),
            val: FlWeight(1442.6949999999997),
        },
        TTFPoint {
            at: Timestamp(27729.421000000002),
            val: FlWeight(1442.7129999999997),
        },
        TTFPoint {
            at: Timestamp(27731.428000000007),
            val: FlWeight(1442.7939999999944),
        },
        TTFPoint {
            at: Timestamp(27732.50800000001),
            val: FlWeight(1442.849999999995),
        },
        TTFPoint {
            at: Timestamp(27733.555000000008),
            val: FlWeight(1442.9319999999934),
        },
        TTFPoint {
            at: Timestamp(27735.61200000001),
            val: FlWeight(1442.9689999999937),
        },
        TTFPoint {
            at: Timestamp(27737.72000000001),
            val: FlWeight(1443.0969999999943),
        },
        TTFPoint {
            at: Timestamp(27741.808000000005),
            val: FlWeight(1443.1669999999976),
        },
        TTFPoint {
            at: Timestamp(27742.850000000006),
            val: FlWeight(1443.2400000000016),
        },
        TTFPoint {
            at: Timestamp(27744.878000000004),
            val: FlWeight(1443.2690000000002),
        },
        TTFPoint {
            at: Timestamp(27745.878000000004),
            val: FlWeight(1443.2970000000023),
        },
        TTFPoint {
            at: Timestamp(27749.983000000007),
            val: FlWeight(1443.369999999999),
        },
        TTFPoint {
            at: Timestamp(27750.995000000003),
            val: FlWeight(1443.4030000000057),
        },
        TTFPoint {
            at: Timestamp(27754.061),
            val: FlWeight(1443.4030000000057),
        },
        TTFPoint {
            at: Timestamp(27765.244),
            val: FlWeight(1443.403000000002),
        },
    ];
}

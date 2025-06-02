/// Copyright 2024 Lucas WAlter
/// BSD 3-Clause
///
/// show a text version of the ros tf tree as received on /tf and /tf_static
use chrono::TimeDelta;
use clap::{arg, command};
use mcap_tools::{bins_text, get_bins, get_sorted_indices};
use tf_mcap::mcaps_to_tf_buffer;
use tf_roslibrust::tf_util;
use tf_util::duration_to_f64;

fn main() -> Result<(), anyhow::Error> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Info)
        .init()
        .unwrap();

    let args = std::env::args();

    // TODO(lucasw) write out a toml of parent child relationships
    // like the transforms node does?  It could be a similar format,
    // static transforms would be the same but everything else could show
    // a min/max value for each xyz rpy value.
    // Also like the mcap_tools mcap_rates tool it could look for gaps in the
    // any parent-child relationship, have stats about gaps between updates
    let matches = command!()
        .arg(
            arg!(
                <mcaps> ... "mcaps to load"
            )
            .trailing_var_arg(true),
        )
        .get_matches_from(args);

    let mcap_names: Vec<_> = matches.get_many::<String>("mcaps").unwrap().collect();
    let mcap_names: Vec<String> = mcap_names.iter().map(|s| (**s).clone()).collect();
    log::info!("mcaps: {mcap_names:?}");

    let tf_buffer = mcaps_to_tf_buffer(&mcap_names)?;

    fn get_gap_bins(gaps: &[TimeDelta], sorted_indices: &Vec<usize>) -> (Vec<f64>, f64, f64) {
        let num = gaps.len();
        let mut gap_dvec = nalgebra::DVector::zeros(num);
        let mut gap_vec = Vec::with_capacity(num);
        for index in sorted_indices {
            let gap = duration_to_f64(gaps[*index]);
            gap_vec.push(gap);
            gap_dvec[*index] = gap;
        }

        // it's already sorted but need the indices for get_bins
        let sort_indices = get_sorted_indices(&gap_vec);
        let bins = get_bins(&gap_vec, &sort_indices, 7);
        (bins, gap_dvec.mean(), gap_dvec.variance().sqrt())
    }

    // TODO(lucasw) make this optional
    // look at each parent-child transform and look for gaps or other irregularities in the stamp of each
    let gap_datas = tf_buffer.get_gaps();
    println!("gaps in seconds between tf timestamps per-pair (with >= 2 transforms):");
    for (tf_graph_node, gap_data) in gap_datas.into_iter() {
        if let Some((_times, gaps, sort_indices)) = gap_data {
            // TODO(lucasw) sort by std_dev
            let (bins, mean, std_dev) = get_gap_bins(&gaps, &sort_indices);

            let shortest_gap = duration_to_f64(gaps[*sort_indices.first().unwrap()]);
            let longest_gap = duration_to_f64(gaps[*sort_indices.last().unwrap()]);

            // TODO(lucasw) instead of printing, put all the stats into a struct for outputting
            // into a toml file
            print!("  {:>8}    mean: {mean:.3}, std dev {std_dev:0.3}, min {shortest_gap:0.3}, max {longest_gap:0.3},", gaps.len());
            print!("    bins: {}", bins_text(&bins));
            println!(",  {} -> {}", tf_graph_node.parent, tf_graph_node.child);
        }
    }

    let _ = tf_util::print_tree(&tf_buffer);

    Ok(())
}

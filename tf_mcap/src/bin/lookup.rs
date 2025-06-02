/*!
Copyright 2024 Lucas WAlter
BSD 3-Clause

Look up a tf parent-child transform in a set of input mcaps, print transform to stdout at specified interval
TODO(lucasw) support multiple pairs of lookups
*/

use clap::{arg, command};
use tf_mcap::mcaps_to_tf_buffer;
use tf_roslibrust::{tf_util, LookupTransform};
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
                -p --parent <PARENT> "parent frame"
            )
            .required(true),
        )
        .arg(
            arg!(
                -c --child <CHILD> "child frame"
            )
            .required(true),
        )
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

    let parent = matches.get_one::<String>("parent").unwrap();
    let child = matches.get_one::<String>("child").unwrap();

    // TODO(lucasw) make clap arg
    let resolution = 0.1;

    let tf_buffer = mcaps_to_tf_buffer(&mcap_names)?;

    let start_sec = duration_to_f64(tf_buffer.start_time());
    let end_sec = duration_to_f64(tf_buffer.end_time());
    let steps = ((end_sec - start_sec) / resolution) as usize;

    println!("# {parent} {child}");
    println!("stamp, x, y, z, roll, pitch, yaw");
    for ind in 0..steps {
        let offset_sec = ind as f64 * resolution;
        let stamp = tf_util::f64_to_stamp(start_sec + offset_sec);

        if let Ok(tf) = tf_buffer.lookup_transform(parent, child, Some(stamp)) {
            let lookup_time = tf_util::stamp_to_f64(&tf.header.stamp);
            let xyz = tf.transform.translation;
            let (x, y, z) = (xyz.x, xyz.y, xyz.z);
            let quat = tf.transform.rotation;
            let unit_quat = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                quat.w, quat.x, quat.y, quat.z,
            ));
            let (roll, pitch, yaw) = unit_quat.euler_angles();
            println!("{lookup_time:.3}, {x}, {y}, {z}, {roll}, {pitch}, {yaw}");
        }
    }

    Ok(())
}

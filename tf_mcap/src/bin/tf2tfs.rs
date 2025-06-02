/*!
Copyright 2024 Lucas Walter
BSD 3-Clause

Look up any number of transforms in tf topics in a set of input mcaps, output them
into a new mcap
*/

use clap::{arg, command};
use roslibrust::RosMessageType;
use roslibrust_util::{std_msgs::Header, tf2_msgs::TFMessage};
use std::collections::BTreeMap;
use std::fs::File;
use std::io::BufWriter;
use tf_mcap::mcaps_to_tf_buffer;
use tf_roslibrust::{
    tf_util,
    tf_util::{duration_to_f64, get_tf2tf_from_toml, tf2tf_to_tfm},
};

fn main() -> Result<(), anyhow::Error> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Info)
        .init()
        .unwrap();

    let args = std::env::args();

    let matches = command!()
        .arg(
            arg!(
                -i --input <INPUT> "input toml file with transforms to look up and save to a new mcap with optional modifications"
            )
            .required(true),
        )
        .arg(
            arg!(
                -o --output <OUTPUT> "output mcap file to store TFMessages in"
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

    let config_file = matches.get_one::<String>("input").unwrap();
    println!("# loading {config_file}");
    let tf2tf_config = get_tf2tf_from_toml(config_file)?;

    let output_mcap_name = matches.get_one::<String>("output").unwrap();
    let mut output_mcap =
        mcap::Writer::new(BufWriter::new(File::create(output_mcap_name).unwrap())).unwrap();

    let channel_id = {
        let schema_id = output_mcap.add_schema(
            TFMessage::ROS_TYPE_NAME,
            "ros1msg",
            TFMessage::DEFINITION.as_bytes(),
        )?;
        let mut connection_summary = BTreeMap::new();
        let topic = "/tf".to_string();
        connection_summary.insert("topic".to_string(), topic.clone());
        output_mcap
            .add_channel(schema_id, &topic, "ros1", &connection_summary)
            .unwrap()
    };

    let mcap_names: Vec<_> = matches.get_many::<String>("mcaps").unwrap().collect();
    let mcap_names: Vec<String> = mcap_names.iter().map(|s| (**s).clone()).collect();
    log::info!("mcaps: {mcap_names:?}");

    // TODO(lucasw) make clap arg
    let resolution = 0.1;

    let tf_buffer = mcaps_to_tf_buffer(&mcap_names)?;

    let start_sec = duration_to_f64(tf_buffer.start_time());
    let end_sec = duration_to_f64(tf_buffer.end_time());
    let steps = ((end_sec - start_sec) / resolution) as usize;

    let mut count = 0;
    for ind in 0..steps {
        let offset_sec = ind as f64 * resolution;
        let stamp = tf_util::f64_to_stamp(start_sec + offset_sec);
        let (tfm, _tf_errors) = tf2tf_to_tfm(&tf_buffer, &tf2tf_config, Some(stamp.clone()));
        mcap_tools::mcap_write::<TFMessage>(
            &mut output_mcap,
            &tfm,
            Header {
                seq: 0,
                stamp,
                frame_id: "".to_string(),
            },
            channel_id,
        )?;
        count += 1;
    }
    println!("{count} transform sets saved");

    Ok(())
}

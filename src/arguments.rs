use std::{
    fs,
    env,
    path::Path,
    io::{BufReader, Read},
};
use crate::config::{parse_config, Config};
use clap::{Arg, App};

pub fn parse_arguments() -> Config {
    let matches = App::new("Detour Graph")
        .version("0.1.0")
        .author("Erik Andersen <3rik4ndersen@gmail.com>")
        .about("Builds a Detour graph")
        .arg(Arg::new("out_path")
                 .short('o')
                 .long("output")
                 .takes_value(true)
                 .help("Folder for output"))
        .arg(Arg::new("config_file")
                 .short('c')
                 .long("config")
                 .takes_value(true)
                 .help("Configuration file"))
        .get_matches();

    let config = match matches.value_of("config") {
    Some(config) =>
    {println!("reading configuration from {}", config);
        parse_config(std::fs::read_to_string(config).unwrap())
    }
    None => {
        println!("using standard configuration (config.cfg)");
        parse_config(std::fs::read_to_string("config.cfg").unwrap())
    }};

    // NOTE: We set the working path here!
    if let Some(out_path) = matches.value_of("out_path") {
    fs::create_dir_all(&out_path).expect("Cant write to specified output folder");
    println!("Writing output to: {}", &out_path);
    let out_path = Path::new(&out_path);
    assert!(env::set_current_dir(&out_path).is_ok());
    } else {
    let out_path = Path::new(&"Output");
    assert!(env::set_current_dir(&out_path).is_ok());
    }
    config
}

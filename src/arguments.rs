use crate::{
    config::{parse_config, Config},
    CLUSTERINGARGS, OUTPUT,
};
use clap::{App, Arg};
use std::{env, fs, path::Path};

pub fn parse_arguments() -> Config {
    let matches = App::new("Detour Graph")
        .version("0.1.0")
        .author("Erik Andersen <3rik4ndersen@gmail.com>")
        .about("Builds a Detour graph. Input is read from stdin. Currently supports GPX, PLT (Geolife Trajectories 1.3), AIS (AIS Brest 2009 at chorochronos), and some csv files.")
        .arg(
            Arg::new("out_path")
                .short('o')
                .long("output")
                .takes_value(true)
                .help("Folder for output"),
        )
        .arg(
            Arg::new("config_file")
                .short('c')
                .long("config")
                .takes_value(true)
                .help("Configuration file"),
        )
        .arg(
            Arg::new("temporal_slack")
                .short('t')
                .long("temporal-slack")
                .takes_value(true)
                .help("Amount of temporal slack. Should be at most 24 hours and always be postfixed with either 'h' for hours or 'm' for minutes. E.g. 4h or 15m. Defaults to 24h i.e. only the spatial dimensions are used in the clustering process."),
        ).arg(
            Arg::new("clustering_threshold")
                .short('c')
                .long("clustering-threshold")
                .takes_value(true)
                .help("Threshold used in clustering. When temporal slack is 24h the threshold corresponds to the maximal distance in meters between two points. Defaults to 50."),
        )
        .arg(
            Arg::new("edges")
                .short('e')
                .long("edges")
                .takes_value(false)
                .help("Write edges to csv"),
        )
        .arg(
            Arg::new("nodes")
                .short('n')
                .long("nodes")
                .takes_value(false)
                .help("Write nodes to csv"),
        )
        .arg(
            Arg::new("graph_dot")
                .short('d')
                .long("graph")
                .takes_value(false)
                .help("Write graph to dot"),
        )
        .arg(
            Arg::new("graph_json")
                .short('j')
                .long("json")
                .takes_value(false)
                .help("Write graph to json"),
        ).arg(
            Arg::new("use_centroids")
                .short('C')
                .long("use-centroids")
                .takes_value(false)
                .help("Use centroids as edge clusters representatives"),
        ).arg(
            Arg::new("use_medoids")
                .short('M')
                .long("use-medoids")
                .takes_value(false)
		.help("Use medoids as edge clusters representatives"),
        )
        .get_matches();

    let config = match matches.value_of("config_file") {
        Some(config) => {
            //println!("reading configuration from {}", config);
            parse_config(std::fs::read_to_string(config).unwrap())
        }
        None => {
            //println!("Using standard configuration (config.json)");
            parse_config(std::fs::read_to_string("config.json").unwrap())
        }
    };

    if let Some(threshold) = matches.value_of("clustering_threshold") {
        let mut clustering_args = CLUSTERINGARGS.lock().unwrap();
        clustering_args.threshold = threshold.parse().unwrap();
    }

    if let Some(temporal_slack) = matches.value_of("temporal_slack") {
        let mut clustering_args = CLUSTERINGARGS.lock().unwrap();
        let time_unit: char = temporal_slack
            .chars()
            .nth(temporal_slack.len() - 1)
            .unwrap();
        let mut time_value: f64 = temporal_slack[0..temporal_slack.len() - 1].parse().unwrap();
        if time_unit == 'm' {
            time_value *= 60.0 * 1000.0;
        } else if time_unit == 'h' {
            time_value *= 60.0 * 60.0 * 1000.0;
        }
        clustering_args.temporal_slack = time_value;
    }

    if matches.is_present("use_medoids") {
        let mut clustering_args = CLUSTERINGARGS.lock().unwrap();
        clustering_args.cluster_strategy = EdgeRepresentative::Medoid;
    }
    if matches.is_present("use_centroids") {
        let mut clustering_args = CLUSTERINGARGS.lock().unwrap();
        clustering_args.cluster_strategy = EdgeRepresentative::Centroid;
    }

    if matches.is_present("edges") {
        let mut output = OUTPUT.lock().unwrap();
        output.edges_csv = true;
    }
    if matches.is_present("nodes") {
        let mut output = OUTPUT.lock().unwrap();
        output.nodes_csv = true;
    }
    if matches.is_present("graph_dot") {
        let mut output = OUTPUT.lock().unwrap();
        output.graph_dot = true;
    }
    if matches.is_present("graph_json") {
        let mut output = OUTPUT.lock().unwrap();
        output.graph_json = true;
    }

    // NOTE: We set the working path here!
    if let Some(out_path) = matches.value_of("out_path") {
        fs::create_dir_all(&out_path).expect("Cant write to specified output folder");
        //println!("Setting output directory: {}", &out_path);
        let out_path = Path::new(&out_path);
        assert!(env::set_current_dir(&out_path).is_ok());
    } else {
        let output = OUTPUT.lock().unwrap();
        if output.edges_csv | output.graph_dot | output.graph_json | output.nodes_csv {
            panic!("Please specify an output directory using -o <output_directory>");
        }
        println!("Not writing any output. Use --help to get information on usage.");
    }
    config
}

pub struct Output {
    pub edges_csv: bool,
    pub nodes_csv: bool,
    pub graph_json: bool,
    pub graph_dot: bool,
}

impl Default for Output {
    fn default() -> Self {
        Output {
            edges_csv: false,
            nodes_csv: false,
            graph_json: false,
            graph_dot: false,
        }
    }
}

pub enum EdgeRepresentative {
    Centroid,
    Medoid,
}

pub struct ClusteringArgs {
    pub threshold: f64,
    pub temporal_slack: f64,
    pub cluster_strategy: EdgeRepresentative,
}

impl Default for ClusteringArgs {
    fn default() -> Self {
        ClusteringArgs {
            threshold: 50.0,
            temporal_slack: 24.0 * 60.0 * 60.0 * 100.0,
            cluster_strategy: EdgeRepresentative::Medoid,
        }
    }
}

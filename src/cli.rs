use std::path::PathBuf;
use std::fs;
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputMethod {
    Direct,
    Interpolated,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputFormat {
    Vtk,
    Png,
}

pub struct CliArgs {
    pub method: Option<InputMethod>, 
    pub input_path: PathBuf,
    pub output_path: Option<PathBuf>,
    pub mesh_u: usize,
    pub mesh_v: usize,
}

impl CliArgs {
    pub fn output_format(&self) -> Option<OutputFormat> {
        let path = self.output_path.as_ref()?;
        let ext = path.extension()?.to_str()?.to_ascii_lowercase();
        match ext.as_str() {
            "vtk" => Some(OutputFormat::Vtk),
            "png" => Some(OutputFormat::Png),
            _ => None,
        }
    }
}

pub fn parse() -> CliArgs {
    let mut args = std::env::args().skip(1);

    let method = match args.next().as_deref() {
        Some("direct") => Some(InputMethod::Direct),
        Some("interpolated") => Some(InputMethod::Interpolated),
        Some(other) => panic!("unknown method '{other}'; use 'direct' or 'interpolated'"),
        None => None, 
    };

    let input_path = args
        .next()
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("inputs/examples/default_script.toml"));

    let output_path = match args.next() {
        Some(path_str) => Some(PathBuf::from(path_str)),
        None => {
            let dir = PathBuf::from("outputs");
            if !dir.exists() {
                let _ = fs::create_dir_all(&dir);
            }
            let timestamp = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_secs())
                .unwrap_or(0);
                
            let filename = format!("surface_{}.vtk", timestamp);
            Some(dir.join(filename))
        }
    };

    let mesh_u = args
        .next()
        .map(|s| parse_usize(&s, "mesh_u"))
        .unwrap_or(32);

    let mesh_v = args
        .next()
        .map(|s| parse_usize(&s, "mesh_v"))
        .unwrap_or(32);

    CliArgs {
        method,
        input_path,
        output_path,
        mesh_u,
        mesh_v,
    }
}

fn parse_usize(text: &str, name: &str) -> usize {
    text.parse::<usize>()
        .unwrap_or_else(|_| panic!("invalid {name} '{text}'; expected a positive integer"))
}

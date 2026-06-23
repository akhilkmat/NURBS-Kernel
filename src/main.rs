mod cli;
mod input;
mod mesh;
mod nurbs;
mod output;

use crate::cli::OutputFormat;

fn main() {
    // 1. Gather what configuration details we can from the CLI arguments
    let args = cli::parse();

    // 2. Load the raw geometric data structure configuration first
    let raw = input::reader::load(&args.input_path)
        .unwrap_or_else(|e| panic!("failed to read input from {}: {e}", args.input_path.display()));

    // 3. Resolve the execution strategy: 
    //    If CLI passed a method option, use it. Otherwise, look inside the TOML file!
    let determined_method = args.method.unwrap_or_else(|| {
        // .trim_matches strips away both literal double and single outer quotes cleanly
        match raw.get("method").map(|v| v.as_str().trim_matches(|c| c == '"' || c == '\'')) {
            Some("direct") => cli::InputMethod::Direct,
            Some("interpolated") => cli::InputMethod::Interpolated,
            Some(unknown) => panic!("unknown method '{}' found in TOML configuration file", unknown),
            None => {
                println!("--- WARNING: No method specified in CLI or TOML. Defaulting to Direct. ---");
                cli::InputMethod::Direct
            }
        }
    });

    // 4. Resolve the structural input method parsing pattern based on resolved strategy
    let surface = match determined_method {
        cli::InputMethod::Direct => {
            println!("--- Pipeline: Running Direct Control Net Extraction ---");
            let spec = input::direct::parse(&raw)
                .unwrap_or_else(|e| panic!("failed to parse direct input: {e}"));
            nurbs::surface::from_direct(spec)
        }
        cli::InputMethod::Interpolated => {
            println!("--- Pipeline: Running Global Surface Interpolation ---");
            let spec = input::interpolated::parse(&raw)
                .unwrap_or_else(|e| panic!("failed to parse interpolated input: {e}"));
            let net = nurbs::interpolate::build_surface(&spec)
                .unwrap_or_else(|e| panic!("failed to interpolate surface: {e}"));
            nurbs::surface::from_control_net(net)
        }
    };

    // 5. Evaluate midpoint spatial derivatives for terminal verification
    let u = 0.5;
    let v = 0.5;
    let frame = surface.frame(u, v);

    println!("method={:?}", determined_method);
    println!(
        "S({u:.1}, {v:.1}) = ({:.4}, {:.4}, {:.4})",
        frame.position.x, frame.position.y, frame.position.z
    );
    println!(
        "∂S/∂u = ({:.4}, {:.4}, {:.4})",
        frame.tangent_u.x, frame.tangent_u.y, frame.tangent_u.z
    );
    println!(
        "∂S/∂v = ({:.4}, {:.4}, {:.4})",
        frame.tangent_v.x, frame.tangent_v.y, frame.tangent_v.z
    );
    println!(
        "normal = ({:.4}, {:.4}, {:.4})",
        frame.normal.x, frame.normal.y, frame.normal.z
    );

    if let Some(curvature) = nurbs::differential::curvature(&frame) {
        println!(
            "curvature: mean={:.6}, gaussian={:.6}",
            curvature.mean, curvature.gaussian
        );
    }

    // 6. Discretize continuous geometry into an explicit triangle mesh
    let triangle_mesh = mesh::tessellate::from_surface(&surface, args.mesh_u, args.mesh_v);
    println!(
        "mesh: {} vertices, {} triangles (grid {}x{})",
        triangle_mesh.vertex_count(),
        triangle_mesh.triangle_count(),
        args.mesh_u,
        args.mesh_v
    );

    // 7. Output file generation pipeline mapping
    if let Some(output_path) = &args.output_path {
        match args.output_format() {
            Some(OutputFormat::Vtk) => {
                let mut mean_curvatures = Vec::with_capacity(triangle_mesh.vertex_count());
                let mut gaussian_curvatures = Vec::with_capacity(triangle_mesh.vertex_count());

                // Calculate exact analytic differential properties at every discrete uv node
                for vertex in &triangle_mesh.vertices {
                    let vertex_frame = surface.frame(vertex.uv.0, vertex.uv.1);
                    
                    if let Some(curv) = nurbs::differential::curvature(&vertex_frame) {
                        mean_curvatures.push(curv.mean);
                        gaussian_curvatures.push(curv.gaussian);
                    } else {
                        mean_curvatures.push(0.0);
                        gaussian_curvatures.push(0.0);
                    }
                }

                // Write full geometric data blocks to file
                output::vtk::write_polydata(
                    &triangle_mesh,
                    &mean_curvatures,
                    &gaussian_curvatures,
                    output_path,
                )
                .unwrap_or_else(|e| panic!("failed to write VTK: {e}"));
                
                println!("saved ParaView mesh {}", output_path.display());
            }
            Some(OutputFormat::Png) => {
                output::render::save_wireframe(&surface, output_path)
                    .unwrap_or_else(|e| panic!("failed to write PNG: {e}"));
                println!("saved wireframe visualization {}", output_path.display());
            }
            None => {
                panic!("unknown or missing output format extension!");
            }
        }
    }
}
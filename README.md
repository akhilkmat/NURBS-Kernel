# NURBS-Kernel

A NURBS (Non-Uniform Rational B-Spline) surface kernel written in Rust, built from first principles to understand the mathematics underlying CAD geometry kernels rather than to wrap an existing library.

Implements Cox-de Boor basis function evaluation, analytic partial derivatives of rational surfaces (not finite differences), and differential geometry quantities (tangents, normals, first and second fundamental forms, Gaussian and mean curvature) needed to assess surface quality. Geometry is exported to VTK for inspection in ParaView, keeping geometric correctness fully separate from rendering concerns.

## Why analytic derivatives, not finite differences

For geometry/CAD applications, derivatives that feed into optimisation, sensitivity analysis, or downstream solvers need to be exact, not approximated. This kernel computes surface derivatives in closed form using the rational-surface quotient rule (Piegl & Tiller, Eq. 4.9), rather than perturbing parameters and re-evaluating.

## Pipeline

```
Input (TOML)
    |
NurbsSurface (ControlNet: control points, weights, knot vectors, degrees)
    |
nurbs/basis.rs        Cox-de Boor basis functions + derivatives
nurbs/derivatives.rs   analytic dS/du, dS/dv, d2S/du2, d2S/dudv, d2S/dv2
nurbs/differential.rs  tangents, normal, first/second fundamental forms, curvature
    |
mesh/tessellate.rs     parametric grid sampling -> triangle mesh, analytic normals
    |
output/vtk.rs          VTK export (positions, normals, curvature, UV) -> ParaView
```

## Project structure

```
src/
├── main.rs              pipeline entry point
├── cli.rs               argument parsing
├── input/               TOML parsing (direct control points / interpolated data points)
├── nurbs/
│   ├── types.rs         Point3, ControlNet, NurbsSurface
│   ├── basis.rs         Cox-de Boor basis values + derivative basis functions
│   ├── surface.rs       S(u,v) evaluation
│   ├── derivatives.rs   analytic partial derivatives (rational quotient rule)
│   ├── differential.rs  tangents, normal, curvature (fundamental forms)
│   ├── curve.rs         1D NURBS curve evaluation
│   └── interpolate.rs   global surface interpolation through data points
├── mesh/
│   ├── types.rs         MeshVertex, TriangleMesh
│   └── tessellate.rs    grid sampling -> triangle mesh with analytic normals
└── output/
    ├── vtk.rs           ParaView-compatible VTK export
    └── render.rs        PNG export (basic)
```

## Running it

```powershell
cargo run direct inputs/examples/default_script.toml outputs/surface.vtk
cargo run direct inputs/examples/default_script.toml outputs/surface.vtk 64 48
```

Open the resulting `.vtk` file in ParaView (File > Open > Apply). Enable normals in display properties for smooth shading, or colour by curvature to inspect surface quality.

## What's verified

- Basis function evaluation and first-order analytic derivatives checked against central finite differences across non-uniform control point weights.
- Second-order derivatives checked the same way; verified correct away from repeated/interior knots. Near a simple interior knot on a degree-2 patch, the analytic second derivative shows a genuine jump rather than matching a central finite difference straddling the knot, this is mathematically expected (a degree-*p* B-spline only guarantees C^(p-1) continuity at a simple knot, so degree 2 guarantees C1, not C2), not a bug.

## Known limitations

- Surface interpolation (`nurbs/interpolate.rs`) uses chord-length parameterisation and Piegl & Tiller's knot-averaging technique, solved via two sequential 1D linear systems (one pass per parametric direction). It is implemented and runs but has lighter test coverage than the core evaluation and derivative code; treat results from this path with appropriate scrutiny.
- Meshing uses uniform parametric grid sampling; adaptive refinement based on curvature is not yet implemented.
- PNG output is a minimal fallback; VTK/ParaView is the primary visualisation path.

## Dependencies

- [`nalgebra`](https://crates.io/crates/nalgebra) for linear solves in surface interpolation

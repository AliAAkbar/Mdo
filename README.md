# MDO - Electric Amphibious Aircraft Optimization

## Overview
Multidisciplinary Design Optimization (MDO) framework for an **electric amphibious aircraft** using OpenVSP for aerodynamics and MATLAB for all other disciplines.

## Project Structure

```
Mdo/
├── main.m                          % Entry point (run this)
├── config/
│   ├── problem_config.m            % Design variables, bounds, mission params
│   ├── ga_config.m                 % Genetic Algorithm hyperparameters
│   ├── moo_config.m                % Multi-objective (NSGA-II) settings
│   ├── propulsion_config.m         % Battery, motor, propeller specs
│   └── amphibious_config.m         % Hull, hydrodynamics, takeoff params
├── optimizer/
│   ├── ga_single.m                 % Single-objective GA with elitism
│   ├── ga_multi.m                  % NSGA-II multi-objective GA
│   ├── selection.m                 % Tournament selection
│   ├── crossover.m                 % Simulated Binary Crossover (SBX)
│   └── mutation.m                  % Polynomial mutation
├── aero/
│   ├── run_openvsp.m               % OpenVSP batch execution + VSPAero
│   ├── compute_aero.m              % Aero discipline wrapper (VSP + analytical)
│   └── parse_vsp_results.m         % VSPAero polar file parser
├── propulsion/
│   ├── battery_model.m             % Li-ion pack sizing + energy
│   ├── motor_model.m               % PMSM efficiency + mass
│   ├── propeller_model.m           % Momentum theory + tip Mach
│   └── propulsion_analysis.m       % Integrated propulsion system
├── amphibious/
│   ├── hull_hydro.m                % Savitsky planing resistance
│   ├── water_takeoff.m             % Takeoff distance simulation
│   ├── float_penalties.m           % Drag/weight penalties
│   └── amphibious_analysis.m       % Integrated amphibious analysis
├── objectives/
│   ├── objective_clcd.m            % Maximize CL/CD
│   ├── objective_weight.m          % Minimize weight
│   ├── objective_range.m           % Maximize range (electric Breguet)
│   ├── constraint_stall.m          % Stall speed constraint
│   ├── constraint_takeoff.m        % Water takeoff constraints
│   └── evaluate_objectives.m       % Master evaluation (all disciplines)
├── utils/
│   ├── init_population.m           % Latin Hypercube Sampling
│   ├── convergence_tracker.m       % Track + plot convergence
│   ├── pareto_front.m              % Extract, plot, save Pareto front
│   └── results_struct.m            % Build + save structured results
└── output/                         % Results directory (auto-created)
```

## Disciplines

| Discipline | Module | Method |
|------------|--------|--------|
| Aerodynamics | `aero/` | OpenVSP VLM/Panel + analytical fallback |
| Propulsion | `propulsion/` | Battery/Motor/Propeller component models |
| Hydrodynamics | `amphibious/` | Savitsky planing method |
| Structures | `objectives/objective_weight.m` | Raymer-style empirical weight |
| Optimization | `optimizer/` | GA (single) + NSGA-II (multi-objective) |

## Objectives (Multi-Objective)
1. **Maximize CL/CD** — Aerodynamic efficiency at cruise
2. **Minimize Weight** — Total aircraft weight from component buildup
3. **Maximize Range** — Electric Breguet range equation

## Constraints
- Stall speed ≤ 30 m/s (≈58 knots, CS-23 compliance)
- Water takeoff distance ≤ 500 m
- Thrust margin at hump ≥ 1.0
- Porpoising stability
- Geometric limits (4 < AR < 14, 0.2 < taper < 1.0)
- Power/energy feasibility

## Design Variables (11)
| Variable | Unit | Range |
|----------|------|-------|
| Wing span | m | 8–16 |
| Root chord | m | 1.0–2.5 |
| Tip chord | m | 0.4–1.2 |
| Sweep | deg | 0–20 |
| Twist | deg | -5–0 |
| Dihedral | deg | 0–10 |
| Thickness ratio | — | 0.10–0.18 |
| Hull beam | m | 1.0–2.5 |
| Hull deadrise | deg | 15–30 |
| Battery mass fraction | — | 0.20–0.45 |
| Motor power | kW | 50–200 |

## Execution
```matlab
>> main
```

## Key Outputs
- Pareto front (3D + 2D projections)
- Knee-point compromise solution
- Convergence history (hypervolume, feasibility, diversity)
- Design variable parallel coordinates
- Structured `.mat` results + text report + CSV export

## Configuration
All parameters are config-driven (no hardcoded values in modules).
Edit files in `config/` to tune the optimizer or change the aircraft.

## Reproducibility
RNG seed is set in `ga_config.m` (default: 42). Same seed → identical results.

## Requirements
- MATLAB R2020a+ (no toolboxes required)
- OpenVSP 3.x+ (optional — analytical fallback available)
- VSPAero (bundled with OpenVSP)

# Aerial Manipulator Simulink

This repository contains a MATLAB/Simulink simulation project for an aerial manipulator system. It includes the main Simulink model, controller S-functions, experiment runners, evaluation utilities, plotting scripts, and a Codex skill for assisted automatic parameter tuning.

## Overview

The project models and evaluates an aerial manipulator with coupled quadrotor and arm dynamics. The simulation workflow is built around `AerialManipulatorSystem.slx` and MATLAB scripts that run repeatable experiment scenarios, collect logged signals, and compute tracking metrics.

The current tuning target is to reduce the average spatial position tracking error:

```matlab
mean(vecnorm(p_actual - p_desired, 2, 2))
```

The closed-loop autotuning goal is to drive both mode 3 and mode 5 fresh simulation runs below `0.05 m` average position error while preserving stability and arm tracking guardrails.

## Repository Contents

- `AerialManipulatorSystem.slx` - main Simulink model.
- `common_functions.m` - shared physical parameters, geometry, and utility functions.
- `sfunc_*` files - controller, observer, planner, input, and dynamics S-functions.
- `run_aerialmanipulator_experiment.m` - runs one configured simulation and saves metrics.
- `evaluate_aerialmanipulator_results.m` - computes position, attitude, arm, saturation, and divergence metrics.
- `run_aerialmanipulator_tuning.m` - evaluates hand-crafted candidate controller settings.
- `run_aerialmanipulator_acceptance_suite.m` - runs representative validation scenarios.
- `plot_mode*_data.m` - plotting and analysis helpers.
- `simulink-uam-autotune/` - Codex skill and MATLAB scripts for algorithm-assisted tuning.

Generated tuning outputs are intentionally ignored by Git under `tuning_results/`.

## Autotuning Skill

The `simulink-uam-autotune` skill is designed for Codex-led parameter tuning. The tuning scripts propose candidates using random, annealing, coordinate, or hybrid search strategies, while Codex reviews the metrics, decides the next search direction, and only persists parameters after fresh validation passes.

Useful entrypoints:

```powershell
matlab -batch "addpath('simulink-uam-autotune/scripts'); simulink_uam_autotune('ProjectRoot', pwd, 'Algorithm', 'hybrid', 'Trials', 20)"
```

Focused mode 5 search:

```powershell
matlab -batch "addpath('simulink-uam-autotune/scripts'); simulink_uam_autotune('ProjectRoot', pwd, 'Scenario', 'mode5', 'Algorithm', 'hybrid', 'Trials', 12)"
```

Summarize historical autotune artifacts:

```powershell
matlab -batch "addpath('simulink-uam-autotune/scripts'); summarize_simulink_uam_trials('ProjectRoot', pwd)"
```

## Basic Usage

Open MATLAB in the repository root, then run:

```matlab
result = run_aerialmanipulator_experiment();
```

Run the acceptance suite:

```matlab
summary = run_aerialmanipulator_acceptance_suite();
```

Run the existing candidate tuning script:

```matlab
summary = run_aerialmanipulator_tuning();
```

## Notes

- MATLAB/Simulink is required.
- Large generated simulation artifacts should stay out of Git.
- The current autotuning workflow has already reduced mode 5 instability substantially, but the final `0.05 m` target still requires further tuning before parameter defaults should be persisted.
- The included paper PDF provides background for the ESO-based aerial manipulation controller design.

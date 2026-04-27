# Aerial Manipulator Simulink

This repository contains a MATLAB/Simulink simulation project for an aerial manipulator system. It includes the main Simulink model, controller S-functions, experiment runners, evaluation utilities, plotting scripts, and validation utilities for controller tuning experiments.

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

Generated tuning outputs are intentionally ignored by Git under `tuning_results/`. Local Codex skills and autotuning helper memory are also kept out of the repository.

## Tuning Workflow

The repository keeps the reproducible simulation and evaluation code in Git. Local-only Codex skills or scratch autotuning helpers can be used during development, but they are intentionally excluded from this GitHub repository so generated memory, prompts, and trial-specific helper code do not become part of the public project history.

For tracked tuning experiments, use the MATLAB entrypoints in this repository, inspect the fresh metrics, and only persist controller defaults after validation passes.

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
- Local Codex skill folders are ignored and should be kept on the local machine unless they are intentionally prepared for publication.
- The current autotuning workflow has already reduced mode 5 instability substantially, but the final `0.05 m` target still requires further tuning before parameter defaults should be persisted.
- The included paper PDF provides background for the ESO-based aerial manipulation controller design.

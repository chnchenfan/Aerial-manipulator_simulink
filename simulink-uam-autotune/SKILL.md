---
name: simulink-uam-autotune
description: Codex-led automatic tuning workflow for this MATLAB/Simulink aerial manipulator project. Use when Codex needs to tune AerialManipulatorSystem.slx, run mode 3/mode 5 Simulink experiments, use random/annealing/coordinate/hybrid algorithms as assistants, judge fresh simulation metrics, persist tuned controller defaults, or drive average position tracking error below 0.05 m.
---

# Simulink UAM Autotune

Use this skill to let Codex complete the closed loop for the local Simulink aerial manipulator model. The tuning algorithms propose candidates; Codex owns judgment, search refinement, persistence, and final validation.

## Non-negotiables

- Tune this workspace's `AerialManipulatorSystem.slx`, not the old PX4/Gazebo workflow.
- Use only fresh Simulink runs from `run_aerialmanipulator_experiment` as pass evidence.
- Treat `position_mean_error_m = mean(vecnorm(p_actual - p_desired, 2, 2))`.
- Require fresh mode 3 and mode 5 runs with `position_mean_error_m < 0.05 m`.
- Keep the guardrails: no divergent run and `arm_axis_max < 0.10` for every axis.
- Do not stop when a script reports "best so far"; inspect the metrics and keep tuning until the acceptance contract is met or a real blocker is found.
- After a fresh passing candidate is found, patch the tuned defaults into the project parameter sources and re-run fresh validation.

## Quick Start

From the repo root:

```bash
matlab -batch "addpath('simulink-uam-autotune/scripts'); summarize_simulink_uam_trials('ProjectRoot', pwd)"
matlab -batch "addpath('simulink-uam-autotune/scripts'); simulink_uam_autotune('ProjectRoot', pwd, 'Algorithm', 'hybrid', 'Trials', 20)"
matlab -batch "addpath('simulink-uam-autotune/scripts'); simulink_uam_autotune('ProjectRoot', pwd, 'Scenario', 'mode5', 'Algorithm', 'random', 'Trials', 10)"
matlab -batch "addpath('simulink-uam-autotune/scripts'); simulink_uam_autotune('ProjectRoot', pwd, 'Scenario', 'mode5', 'Algorithm', 'anneal', 'InitialTrialJson', 'tuning_results/autotune_mode5_lowz8/trial_0006.json', 'Trials', 10)"
```

Use fewer trials for a smoke test, then increase trials or switch algorithms based on the metrics.

## Workflow

1. Read `references/simulink-uam-tuning.md` for metric, scoring, search bounds, and persistence rules.
2. Run the summarizer to understand existing `tuning_results/autotune_*` artifacts.
3. Run a short `hybrid` or `random` batch if the current best is unknown.
4. Inspect mode 3 and mode 5 metrics. If one mode lags, use `Scenario` to search that mode faster, then return to `Scenario='both'` for acceptance.
5. Continue with `anneal` around the best candidate, then `coordinate` refinement near promising values.
6. Accept only a fresh candidate where both modes pass the threshold and guardrails.
7. Patch tuned defaults into the project parameter sources, then run final fresh validation.

## Scripts

- `scripts/simulink_uam_autotune.m`: MATLAB entrypoint. Supports name-value options including `ProjectRoot`, `Algorithm`, `Trials`, `Threshold`, `Seed`, `OutputRoot`, `Scenario`, and `InitialTrialJson`.
- `scripts/summarize_simulink_uam_trials.m`: Reads historical `summary.json` and `trial_*.json` artifacts, then prints rankings and pass status.

## Output Locations

- Batch artifacts: `tuning_results/autotune_<timestamp>/...`
- Per-trial artifacts: `trial_####.mat` and `trial_####.json`
- Batch summary: `summary.mat` and `summary.json`
- Passing parameters: `final_params.json`

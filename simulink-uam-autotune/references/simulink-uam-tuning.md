# Simulink UAM Tuning Reference

## Acceptance Contract

- Project root: the directory containing `AerialManipulatorSystem.slx`.
- Simulation entrypoint: `run_aerialmanipulator_experiment(config)`.
- Required scenarios:
  - Mode 3 hover with arm motion.
  - Mode 5 tracking with arm motion, `T = 20`, `pos_target = [0.2; 0.2; 5]`.
- Primary metric: `position_mean_error_m = mean(vecnorm(p_actual - p_desired, 2, 2))`.
- Pass threshold: both scenarios must have `position_mean_error_m < 0.05`.
- Guardrails: no divergent run, every `arm_axis_max < 0.10`, and no missing metrics.
- Valid evidence: fresh `trial_*.json` / `trial_*.mat` artifacts produced by the tuner from new Simulink runs.

## Codex Loop

The script proposes and evaluates candidates, but Codex makes the tuning decisions.

1. Summarize previous runs.
2. Run a small `hybrid` batch to establish the current best.
3. If both modes pass, patch defaults and revalidate.
4. If not, inspect which mode and metric dominates the score.
5. Use `Scenario='mode3'` or `Scenario='mode5'` to search the failing mode faster, then return to `Scenario='both'` for acceptance.
6. Continue with `anneal` near the best candidate or `coordinate` if one parameter group appears limiting.
7. Widen search bounds only when repeated batches converge above the threshold.
8. Never accept stale summaries alone.

## Search Surface

Tune only runtime override fields that already exist in the project:

- `system_params`: `Omega_p`, `K_v`, `Omega_q`, `K_omega`, `w_p`, `w_o`
- `arm_params`: `K_p`, `K_d`, `K_i`
- `position_limits`: `max_lateral_force_ratio`, `integral_limit`, `integral_feedback_gain`, `position_error_limit`, `velocity_error_limit`
- `attitude_limits`: `tau_max`

The tuner stores vectors as structured controller configs, so accepted values can be copied into `common_functions.m`, `sfunc_arm_controller.m`, `sfunc_position_controller.m`, and `sfunc_attitude_controller.m`.

## Score

Use the pass/fail contract for acceptance. Use score only for ranking:

```text
score = mode3_mean + mode5_mean
      + 0.15 * (mode3_position_rms + mode5_position_rms)
      + arm_penalty
      + max_position_penalty
      + saturation_penalty
      + divergence_penalty
```

Penalties:

- `arm_penalty = 10 * sum(max(0, arm_axis_max - 0.10))`
- `max_position_penalty = 2 * max(0, position_max - 0.25)`
- `saturation_penalty = 0.5 * (thrust_saturation_ratio + torque_saturation_ratio + arm_torque_saturation_ratio)`, ignoring NaN values.
- `divergence_penalty = 1e6` for divergent or failed runs.

## Algorithm Guidance

- `hybrid`: default first choice. It uses seeded candidates, random exploration, annealed local perturbation, and coordinate refinement.
- `random`: use after changing bounds or when the search appears stuck in a local pocket.
- `anneal`: use when one candidate is clearly best but still above threshold.
- `coordinate`: use when values are close to passing and small one-dimensional adjustments are likely to matter.
- `Scenario='mode5'`: use when mode 5 diverges badly; it halves the cost of screening candidates, but cannot produce final acceptance by itself.
- `InitialTrialJson`: use a fresh historical `trial_####.json` as the first seed when continuing a search around a promising candidate.

## Persistence Rule

Patch defaults only after a fresh passing candidate. Keep edits minimal:

- `common_functions.m`: system gains and ESO bandwidths.
- `sfunc_arm_controller.m`: arm PID gains and torque limit if tuned.
- `sfunc_position_controller.m`: position limits and integral feedback.
- `sfunc_attitude_controller.m`: attitude torque limit.

After patching, run fresh mode 3 and mode 5 validation with no controller overrides to prove the persisted defaults pass.

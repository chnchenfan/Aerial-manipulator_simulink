# Aerial Manipulator Simulink S-Function Controllers

This repository implements and evaluates two aerial-manipulator control baselines in MATLAB/Simulink using S-functions:

- An **ESO-based aerial manipulator controller**, including extended state observers, decoupled aerial-manipulator dynamics construction and compensation, and nonlinear backstepping-style control laws.
- A **PX4-like comparison controller**, including cascaded position/velocity PID, thrust-vector attitude reference generation, and an SO(3)-based attitude/rate PID path adapted to the current plant interface.

The project uses Simulink S-functions to keep the controller, observer, planner, manipulator dynamics, quadrotor dynamics, logging, and evaluation logic reproducible inside one local simulation framework. Controller parameters were tuned with local Codex-based skills that automate repeated Simulink trials, metric extraction, and parameter search. Those skills are still under testing and are not included in this repository yet; they may be cleaned up and open-sourced later.

The PX4-like controller in this repository is not PX4 stock firmware, and it is not a PX4 SITL/HITL/Gazebo result. It is an in-project baseline designed to use the same plant, references, measurement perturbations, logging, and metrics as the ESO controller.

## Project Structure

```text
.
+-- AerialManipulatorSystem.slx
|   +-- sfunc_input.m                         # mode/reference generator
|   +-- sfunc_position_eso.m                  # position ESO
|   +-- sfunc_position_controller.m           # ESO-based position/backstepping controller
|   +-- sfunc_Tranformation.m                 # thrust-vector to attitude reference conversion
|   +-- sfunc_attitude_eso.m                  # attitude ESO
|   +-- sfunc_attitude_controller.m           # ESO branch and PX4-like attitude/rate branch
|   +-- sfunc_quadrotor_dynamics.m            # quadrotor dynamics with manipulator coupling terms
|   +-- sfunc_arm_controller.m                # manipulator controller
|   +-- sfunc_arm_dynamics.m                  # Delta-arm dynamics model
|   +-- sfunc_cooperative_planner.m           # cooperative planning helpers
|   +-- sfunc_tracking_differentiator.m       # tracking differentiator blocks
|
+-- AerialManipulatorSystem_PX4Like.slx
|   +-- sfunc_input.m                         # same input/reference generator
|   +-- sfunc_px4_like_controller.m           # PX4-like position/velocity/thrust stage
|   +-- sfunc_Tranformation.m                 # same thrust-vector attitude reference conversion
|   +-- sfunc_attitude_controller.m           # PX4-like SO(3) attitude/rate PID branch
|   +-- sfunc_quadrotor_dynamics.m            # same quadrotor plant
|   +-- sfunc_arm_controller.m                # same arm controller
|   +-- sfunc_arm_dynamics.m                  # same arm plant
|
+-- Experiment and evaluation scripts
|   +-- run_aerialmanipulator_experiment.m    # run one configured simulation
|   +-- run_aerialmanipulator_acceptance_suite.m
|   +-- run_aerialmanipulator_tuning.m
|   +-- run_px4_like_comparison.m             # fresh ESO vs PX4-like comparison runs
|   +-- evaluate_aerialmanipulator_results.m  # metrics and pass/fail flags
|   +-- sim_tuning_runtime.m                  # runtime config and signal logging
|   +-- merge_structs.m
|
+-- Plotting and reporting
|   +-- plot_mode1_data.m
|   +-- plot_mode2_data.m
|   +-- plot_mode3_data.m
|   +-- plot_mode5_data.m
|   +-- plot_mode3_mode5_report.m
|   +-- plot_px4_like_comparison.m
|   +-- figures/
|       +-- px4_like_comparison/
|
+-- Shared assets and reference data
|   +-- common_functions.m                    # parameters, geometry, rotations, Delta kinematics
|   +-- baseline_data.mat
|   +-- baseline_vars.mat
|   +-- ESO-Based_Robust_and_High-Precision_Tracking_Control_for_Aerial_Manipulation.pdf
|
+-- Generated results
    +-- tuning_results/                       # ignored by Git; local MAT/JSON trial outputs
```

`AerialManipulatorSystem.slx` is the main ESO-based model. It keeps both position and attitude ESO subsystems active and uses the ESO-based UAV controller chain.

`AerialManipulatorSystem_PX4Like.slx` is a copied comparison model. Its position and attitude ESO subsystems are removed, legacy observer input ports are fed with zero vectors, and the UAV control path is replaced by the PX4-like baseline. The plant, manipulator, input modes, measurement model, logging, and evaluation scripts remain shared.

## Workflow and Usage

Open MATLAB in the repository root and add the project to the path if needed:

```matlab
addpath(pwd)
```

Run a default ESO-based experiment:

```matlab
result = run_aerialmanipulator_experiment();
```

Run a configured experiment:

```matlab
config = struct();
config.input = struct('mode', 3);
config.sim = struct('model_name', 'AerialManipulatorSystem', 'stop_time', 100);
config.output = struct('label', 'paper_eso_mode3', ...
    'save_dir', fullfile(pwd, 'tuning_results'), ...
    'save_results', true);
result = run_aerialmanipulator_experiment(config);
```

Run the fresh ESO vs PX4-like comparison:

```matlab
outputs = run_px4_like_comparison();
```

This runs `paper_eso` and `px4_like` in mode 3 and mode 5, writes result files under `tuning_results/px4_like_comparison/`, and generates comparison figures plus `px4_like_metrics_summary.txt` under `figures/px4_like_comparison/`.

Generate the current mode 3/mode 5 ESO report figures:

```matlab
plot_mode3_mode5_report
```

The local Codex autotuning skill used during development is not uploaded yet. For now, reproducible entrypoints are the MATLAB scripts in this repository. Generated trial data under `tuning_results/` is intentionally ignored by Git.

## Validation Scenarios

The validation scenarios are designed to test whether the UAV can maintain accurate position control while the manipulator changes the coupled system dynamics. Both controller families use the same plant, same manipulator motion, same references, same measurement perturbations, same simulation time, and same metric pipeline.

The main position metrics are evaluated per translation axis:

```matlab
position_axis_mean = mean(abs(p_actual - p_desired), 1)
position_axis_max  = max(abs(p_actual - p_desired), [], 1)
```

The current strict target is:

```matlab
position_axis_mean < [0.02 0.02 0.02]  % m
position_axis_max  < [0.04 0.04 0.04]  % m
arm_axis_max       < [0.10 0.10 0.10]  % rad
is_divergent == false
```

For the ESO controller, an additional observer guardrail is used:

```matlab
max(abs(p_hat - p_true), [], 1) < [0.01 0.01 0.01]  % m
```

The logged `h_v_true` / `h_v_est` channel is a disturbance-acceleration estimate in `m/s^2`, not a meter-valued position error.

### Mode 3: Hover With Arm Motion

Mode 3 verifies whether the UAV can hold a stable hover while the manipulator moves periodically. This scenario stresses disturbance rejection because arm motion changes the mass distribution and introduces coupling forces and torques. The UAV reference is near `[0, 0, 5] m`, and the three manipulator joints follow sinusoidal references.

Enabled measurement perturbations:

- Quadrotor measurement delay: `1` sample, approximately `0.010 s`.
- Arm measurement delay: `0` samples in the current validation run.
- Position noise standard deviation: `0.000825 m`.
- Velocity noise standard deviation: `0.001875 m/s`.
- Attitude noise standard deviation: `0.000375 rad`.
- Angular velocity noise standard deviation: `0.0013125 rad/s`.
- Arm position noise standard deviation: `0.001875 rad`.
- Arm velocity noise standard deviation: `0.00525 rad/s`.
- Arm acceleration noise standard deviation: `0.00750 rad/s^2`.
- Bias random walk, quantization, and colored-noise shaping are enabled by the empirical measurement model.

Not enabled in the current validation:

- Delay jitter.
- Packet dropout.
- Wind gusts.
- Actuator faults.
- Motor saturation fault injection.
- Payload mass variation.
- Sensor outages.
- Contact or collision disturbances.

Latest fresh comparison results:

| Controller | Axis mean position error (m) | Axis max position error (m) | Position RMS (m) | Position max (m) | Arm axis max (rad) | Divergent |
| --- | --- | --- | ---: | ---: | --- | --- |
| `paper_eso` | `[0.000588 0.000749 0.001213]` | `[0.002810 0.003369 0.004052]` | `0.001904` | `0.004532` | `[0.042269 0.039801 0.041402]` | `false` |
| `px4_like` | `[0.001502 0.001915 0.001923]` | `[0.005971 0.006147 0.006930]` | `0.003861` | `0.008115` | `[0.040316 0.041182 0.039375]` | `false` |

### Mode 5: Square Tracking With Arm Motion

Mode 5 verifies whether the UAV can track a horizontal square-like trajectory while the manipulator moves. Compared with mode 3, it adds translational motion and corner transitions, so it tests tracking performance and disturbance rejection at the same time. The reference starts at the hover height (`z = 5 m`) to avoid an artificial initial-condition mismatch.

Mode 5 uses the same empirical measurement perturbations as mode 3. The same unmodeled robustness cases, such as wind, dropout, actuator faults, payload changes, and collision/contact disturbances, are not enabled.

Latest fresh comparison results:

| Controller | Axis mean position error (m) | Axis max position error (m) | Position RMS (m) | Position max (m) | Arm axis max (rad) | Divergent |
| --- | --- | --- | ---: | ---: | --- | --- |
| `paper_eso` | `[0.000614 0.000758 0.001214]` | `[0.002841 0.003350 0.004006]` | `0.001919` | `0.004537` | `[0.042257 0.040552 0.042022]` | `false` |
| `px4_like` | `[0.001435 0.001925 0.001922]` | `[0.004944 0.006132 0.006899]` | `0.003829` | `0.008174` | `[0.040492 0.039502 0.040065]` | `false` |

The PX4-like comparison also reports zero thrust saturation ratio and zero torque saturation ratio in the latest fresh runs.

## Dynamics Model Notes

The quadrotor translational and rotational dynamics in `sfunc_quadrotor_dynamics.m` follow the paper-level rigid-body structure and include manipulator coupling terms derived from center-of-mass motion and arm inertia.

The Delta-arm plant in `sfunc_arm_dynamics.m` is a project-specific simulation model rather than an exact first-principles reproduction of every paper detail. It includes simplified/empirical terms such as a diagonal-dominant arm mass matrix, heuristic Coriolis/damping terms, base-coupling saturation, acceleration saturation, measurement noise/delay, and fallback simplified dynamics. Treat it as a simulation plant for controller validation.

## Future Work

- Replace or augment the current hand-coded plant with a Simscape-based controlled object model for higher-fidelity multibody dynamics.
- Explore MATLAB/Simulink PX4 HITL/SITL workflows if true PX4 firmware-in-the-loop validation is required.
- Add Gazebo co-simulation when scene-level sensors, contact, collision, or environment interaction become important.
- Extend robustness validation to include wind gusts, delay jitter, packet dropout, payload variation, motor/actuator faults, and sensor outages.
- Clean up, document, and eventually open-source the Codex-based autotuning skills after the workflow is more stable.

## Notes

- MATLAB/Simulink is required.
- Large generated simulation artifacts should stay out of Git.
- `tuning_results/` is local output and is ignored by Git.
- The included paper PDF provides background for the ESO-based aerial manipulation controller design.

function summary = simulink_uam_autotune(varargin)
%SIMULINK_UAM_AUTOTUNE Codex-assisted closed-loop tuning for this project.
%
% Name-value options:
%   ProjectRoot  - directory containing AerialManipulatorSystem.slx
%   Algorithm    - hybrid, random, anneal, or coordinate
%   Trials       - number of candidates to evaluate
%   Threshold    - required position_mean_error_m for both modes
%   Seed         - random seed
%   OutputRoot   - output directory for fresh trial artifacts
%   Scenario     - both, mode3, or mode5
%   InitialTrialJson - optional trial_####.json to seed continued search

opts = parse_options(varargin{:});
addpath(opts.ProjectRoot);
rng(opts.Seed);

if ~exist(opts.OutputRoot, 'dir')
    mkdir(opts.OutputRoot);
end

space = tuning_space();
seeds = seed_vectors(space);
initial_vector = load_initial_vector(opts.InitialTrialJson, space);
if ~isempty(initial_vector)
    if strcmp(opts.Algorithm, 'anneal') || strcmp(opts.Algorithm, 'coordinate')
        seeds = initial_vector;
    else
        seeds = [initial_vector; seeds];
    end
end
best = struct('score', inf, 'vector', [], 'trial', []);
trials = cell(opts.Trials, 1);

for idx = 1:opts.Trials
    vector = propose_vector(opts.Algorithm, idx, opts.Trials, space, seeds, best);
    candidate = vector_to_candidate(vector, space, sprintf('trial_%04d', idx));
    trial = evaluate_candidate(candidate, opts, idx);
    trials{idx} = trial;

    if trial.score < best.score
        best.score = trial.score;
        best.vector = vector;
        best.trial = trial;
    end

    write_trial(opts.OutputRoot, idx, trial);
    summary = build_summary(opts, trials, best);
    write_summary(opts.OutputRoot, summary);
end

summary = build_summary(opts, trials, best);
write_summary(opts.OutputRoot, summary);

if isfield(summary.best, 'passed') && summary.best.passed
    write_text(fullfile(opts.OutputRoot, 'final_params.json'), ...
        jsonencode(summary.best.candidate, 'PrettyPrint', true));
end

disp(jsonencode(compact_console_summary(summary), 'PrettyPrint', true));

function opts = parse_options(varargin)
script_dir = fileparts(mfilename('fullpath'));
default_project_root = fileparts(fileparts(script_dir));
opts = struct( ...
    'ProjectRoot', default_project_root, ...
    'Algorithm', 'hybrid', ...
    'Trials', 20, ...
    'Threshold', 0.05, ...
    'Seed', 20260426, ...
    'OutputRoot', '', ...
    'Scenario', 'both', ...
    'InitialTrialJson', '');

if mod(numel(varargin), 2) ~= 0
    error('simulink_uam_autotune: options must be name-value pairs.');
end

for i = 1:2:numel(varargin)
    name = varargin{i};
    if ~ischar(name) && ~isstring(name)
        error('simulink_uam_autotune: option names must be strings.');
    end
    name = char(name);
    if ~isfield(opts, name)
        error('simulink_uam_autotune: unknown option %s.', name);
    end
    opts.(name) = varargin{i + 1};
end

opts.ProjectRoot = char(opts.ProjectRoot);
opts.Algorithm = lower(char(opts.Algorithm));
opts.Scenario = lower(char(opts.Scenario));
opts.InitialTrialJson = char(opts.InitialTrialJson);
opts.Trials = double(opts.Trials);
opts.Threshold = double(opts.Threshold);
opts.Seed = double(opts.Seed);

if isempty(opts.OutputRoot)
    stamp = datestr(now, 'yyyymmdd_HHMMSS');
    opts.OutputRoot = fullfile(opts.ProjectRoot, 'tuning_results', ['autotune_' stamp]);
else
    opts.OutputRoot = char(opts.OutputRoot);
end

valid_algorithms = {'hybrid', 'random', 'anneal', 'coordinate'};
if ~any(strcmp(opts.Algorithm, valid_algorithms))
    error('simulink_uam_autotune: Algorithm must be hybrid, random, anneal, or coordinate.');
end

valid_scenarios = {'both', 'mode3', 'mode5'};
if ~any(strcmp(opts.Scenario, valid_scenarios))
    error('simulink_uam_autotune: Scenario must be both, mode3, or mode5.');
end

function space = tuning_space()
space.names = { ...
    'Omega_p_x', 'Omega_p_y', 'Omega_p_z', ...
    'K_v_x', 'K_v_y', 'K_v_z', ...
    'Omega_q_x', 'Omega_q_y', 'Omega_q_z', ...
    'K_omega_x', 'K_omega_y', 'K_omega_z', ...
    'w_p_x', 'w_p_y', 'w_p_z', ...
    'w_o_x', 'w_o_y', 'w_o_z', ...
    'arm_K_p_1', 'arm_K_p_2', 'arm_K_p_3', ...
    'arm_K_d_1', 'arm_K_d_2', 'arm_K_d_3', ...
    'arm_K_i_1', 'arm_K_i_2', 'arm_K_i_3', ...
    'max_lateral_force_ratio', ...
    'integral_limit_x', 'integral_limit_y', 'integral_limit_z', ...
    'integral_feedback_gain_x', 'integral_feedback_gain_y', 'integral_feedback_gain_z', ...
    'tau_max', ...
    'position_error_limit_x', 'position_error_limit_y', 'position_error_limit_z', ...
    'velocity_error_limit_x', 'velocity_error_limit_y', 'velocity_error_limit_z'};

space.base = [ ...
    1.65, 1.65, 16.5, ...
    6.2, 6.2, 16.5, ...
    0.5, 1.5, 1.5, ...
    0.5, 1.5, 1.4, ...
    3.8, 3.4, 3.0, ...
    4.8, 6.2, 6.2, ...
    11, 13, 11, ...
    1.1, 1.4, 1.1, ...
    7, 6, 7, ...
    0.38, ...
    0.30, 0.30, 0.50, ...
    0.0, 0.0, 1.60, ...
    22, ...
    1.0, 1.0, 1.0, ...
    2.0, 2.0, 2.0];

space.lo = [ ...
    1.0, 1.0, 4.0, ...
    4.5, 4.5, 3.0, ...
    0.3, 1.0, 1.0, ...
    0.3, 1.0, 1.0, ...
    2.5, 2.3, 0.8, ...
    3.0, 4.0, 4.0, ...
    8, 10, 8, ...
    0.7, 0.9, 0.7, ...
    4, 3, 4, ...
    0.25, ...
    0.15, 0.15, 0.25, ...
    0.0, 0.0, 0.6, ...
    14, ...
    0.5, 0.5, 0.5, ...
    1.0, 1.0, 1.0];

space.hi = [ ...
    2.4, 2.4, 18.0, ...
    8.0, 8.0, 18.0, ...
    0.9, 2.2, 2.2, ...
    1.0, 2.4, 2.4, ...
    5.2, 5.0, 4.2, ...
    6.5, 8.0, 8.0, ...
    15, 17, 15, ...
    1.7, 2.0, 1.7, ...
    10, 9, 10, ...
    0.50, ...
    0.55, 0.55, 0.80, ...
    0.25, 0.25, 2.4, ...
    30, ...
    3.0, 3.0, 5.0, ...
    6.0, 6.0, 8.0];

space.step = 0.18 * (space.hi - space.lo);

function seeds = seed_vectors(space)
seeds = [
    space.base;
    replace_values(space.base, [4 5 6 13 14 15 16 17 18], [5.8 5.8 9.0 3.6 3.2 2.9 4.5 6.0 6.0]);
    replace_values(space.base, [4 5 6 19 20 21 22 23 24 25 26 27], [6.5 6.5 9.4 12 14 12 1.2 1.5 1.2 6 5 6]);
    replace_values(space.base, [4 5 6 13 14 15 16 17 18 28 35], [5.5 5.5 8.7 3.4 3.0 2.8 4.2 5.8 5.8 0.35 18]);
    replace_values(space.base, [1 2 4 5 13 14 16 17 28 32 34 35], [1.85 1.85 6.8 6.8 4.2 3.8 5.3 6.8 0.42 0.0 1.85 24]);
    replace_values(space.base, [3 6 15 18 31 34 35], [7.0 7.0 1.2 5.2 0.35 1.2 18]);
    replace_values(space.base, [3 6 15 18 31 34 35], [5.0 5.0 0.9 4.8 0.25 0.8 16]);
    replace_values(space.base, [3 6 15 18 19 20 21 22 23 24 25 26 27 31 34 35], [7.0 7.0 1.2 5.2 13 16 13 1.6 1.9 1.6 8 8 8 0.70 2.2 22]);
    replace_values(space.base, [3 6 15 18 19 20 21 22 23 24 25 26 27 31 34 35], [8.5 8.5 1.5 5.6 14 17 14 1.7 2.0 1.7 6 6 6 0.80 2.4 24]);
    replace_values(space.base, [3 6 15 18 19 20 21 22 23 24 25 26 27 31 34 35], [6.0 8.0 1.1 5.0 12 15 12 1.5 1.8 1.5 9 9 9 0.65 2.0 20]);
    replace_values(space.base, [3 6 15 18 31 34 35 38 41], [7.0 7.0 1.2 5.2 0.70 2.2 22 3.0 6.0]);
    replace_values(space.base, [3 6 15 18 31 34 35 38 41], [8.0 9.0 1.4 5.5 0.80 2.4 24 5.0 8.0])
    ];

for i = 1:size(seeds, 1)
    seeds(i, :) = clamp_vector(seeds(i, :), space);
end

function vector = load_initial_vector(path, space)
vector = [];
if isempty(path)
    return;
end

try
    data = jsondecode(fileread(path));
    if isfield(data, 'candidate') && isfield(data.candidate, 'vector')
        vector = double(data.candidate.vector(:))';
    end
catch
    vector = [];
end

if numel(vector) < numel(space.base)
    vector = [vector, space.base(numel(vector) + 1:end)];
elseif numel(vector) > numel(space.base)
    vector = vector(1:numel(space.base));
end

vector = clamp_vector(vector, space);

function out = replace_values(base, indexes, values)
out = base;
out(indexes) = values;

function vector = propose_vector(algorithm, idx, total_trials, space, seeds, best)
if idx <= size(seeds, 1)
    vector = seeds(idx, :);
    return;
end

switch algorithm
    case 'random'
        vector = random_vector(space);
    case 'anneal'
        vector = anneal_vector(space, best, idx, total_trials);
    case 'coordinate'
        vector = coordinate_vector(space, best, idx);
    otherwise
        if isempty(best.vector)
            vector = random_vector(space);
        elseif idx <= max(size(seeds, 1) + 4, round(total_trials * 0.45))
            vector = random_vector(space);
        elseif idx <= round(total_trials * 0.80)
            vector = anneal_vector(space, best, idx, total_trials);
        else
            vector = coordinate_vector(space, best, idx);
        end
end

vector = clamp_vector(vector, space);

function vector = random_vector(space)
vector = space.lo + rand(size(space.base)) .* (space.hi - space.lo);

function vector = anneal_vector(space, best, idx, total_trials)
center = space.base;
if ~isempty(best.vector)
    center = best.vector;
end
temperature = max(0.12, 1.0 - idx / max(total_trials, 1));
vector = center + randn(size(space.base)) .* space.step * temperature;

function vector = coordinate_vector(space, best, idx)
center = space.base;
if ~isempty(best.vector)
    center = best.vector;
end
vector = center;
param_idx = mod(idx - 1, numel(center)) + 1;
direction = 1;
if mod(floor((idx - 1) / numel(center)), 2) == 1
    direction = -1;
end
vector(param_idx) = vector(param_idx) + direction * 0.35 * space.step(param_idx);

function vector = clamp_vector(vector, space)
vector = max(space.lo, min(space.hi, vector));

function candidate = vector_to_candidate(v, space, name)
candidate = struct();
candidate.name = name;
candidate.vector_names = space.names;
candidate.vector = v;
candidate.system_params = struct( ...
    'Omega_p', diag(v(1:3)), ...
    'K_v', diag(v(4:6)), ...
    'Omega_q', diag(v(7:9)), ...
    'K_omega', diag(v(10:12)), ...
    'w_p', v(13:15), ...
    'w_o', v(16:18));
candidate.arm_params = struct( ...
    'K_p', diag(v(19:21)), ...
    'K_d', diag(v(22:24)), ...
    'K_i', diag(v(25:27)), ...
    'tau_max', [v(35); v(35); v(35)]);
candidate.position_limits = struct( ...
    'max_lateral_force_ratio', v(28), ...
    'integral_limit', v(29:31)', ...
    'integral_feedback_gain', v(32:34)', ...
    'position_error_limit', v(36:38)', ...
    'velocity_error_limit', v(39:41)');
candidate.attitude_limits = struct('tau_max', v(35));

function trial = evaluate_candidate(candidate, opts, idx)
trial = struct();
trial.index = idx;
trial.candidate = candidate;
trial.created_at = datestr(now, 'yyyy-mm-dd HH:MM:SS');

try
    if strcmp(opts.Scenario, 'both') || strcmp(opts.Scenario, 'mode3')
        mode3 = run_mode(candidate, opts, 3, sprintf('%s_mode3', candidate.name));
        trial.mode3 = mode3.metrics;
        trial.mode3_file = get_output_file(mode3);
    end
    if strcmp(opts.Scenario, 'both') || strcmp(opts.Scenario, 'mode5')
        mode5 = run_mode(candidate, opts, 5, sprintf('%s_mode5', candidate.name));
        trial.mode5 = mode5.metrics;
        trial.mode5_file = get_output_file(mode5);
    end
    trial.score = score_trial(trial);
    trial.mode3_passed = isfield(trial, 'mode3') && mode_passes(trial.mode3, opts.Threshold);
    trial.mode5_passed = isfield(trial, 'mode5') && mode_passes(trial.mode5, opts.Threshold);
    trial.passed = isfield(trial, 'mode3') && isfield(trial, 'mode5') && ...
        trial.mode3_passed && trial.mode5_passed;
catch exc
    trial.error = exc.message;
    trial.score = 1e6;
    trial.passed = false;
end

function result = run_mode(candidate, opts, mode, label)
cfg = struct();
cfg.input = struct('mode', mode);
cfg.output = struct('label', label, 'save_dir', opts.OutputRoot, 'save_results', true);
cfg.measurement = measurement_config();
cfg.controller = struct( ...
    'system_params', candidate.system_params, ...
    'arm_params', candidate.arm_params, ...
    'position_limits', candidate.position_limits, ...
    'attitude_limits', candidate.attitude_limits);

if mode == 5
    cfg.input.T = 20;
    cfg.input.pos_target = [0.2; 0.2; 5];
end

result = run_aerialmanipulator_experiment(cfg);
result.metrics = add_position_mean_metric(result.metrics, result.signals);

function measurement = measurement_config()
measurement = struct( ...
    'model', 'empirical', ...
    'quad_delay_steps', 1, ...
    'arm_delay_steps', 0, ...
    'position_noise_std', 0.0013, ...
    'velocity_noise_std', 0.0032, ...
    'attitude_noise_std', 0.0007, ...
    'omega_noise_std', 0.0022, ...
    'hv_noise_std', 0.0035, ...
    'arm_position_noise_std', 0.0032, ...
    'arm_velocity_noise_std', 0.0090, ...
    'arm_acceleration_noise_std', 0.0130, ...
    'end_effector_position_noise_std', 0.0012, ...
    'end_effector_velocity_noise_std', 0.0035);

function metrics = add_position_mean_metric(metrics, signals)
actual = get_signal(signals, 'p_true');
if isempty(actual)
    actual = get_signal(signals, 'p_actual');
end
desired = get_signal(signals, 'p_desired');

if isempty(actual) || isempty(desired)
    metrics.position_mean_error_m = inf;
    return;
end

n = min(size(actual, 1), size(desired, 1));
err = actual(1:n, :) - desired(1:n, :);
metrics.position_mean_error_m = mean(vecnorm(err, 2, 2));

function data = get_signal(signals, name)
data = [];
if isfield(signals, name) && isfield(signals.(name), 'data')
    data = signals.(name).data;
end

function passed = mode_passes(metrics, threshold)
passed = isfield(metrics, 'position_mean_error_m') && ...
    metrics.position_mean_error_m < threshold && ...
    isfield(metrics, 'arm_axis_max') && all(metrics.arm_axis_max < 0.10) && ...
    isfield(metrics, 'is_divergent') && ~metrics.is_divergent;

function score = score_trial(trial)
if isfield(trial, 'error')
    score = 1e6;
    return;
end

score = 0;
if isfield(trial, 'mode3')
    score = score + safe_metric(trial.mode3, 'position_mean_error_m', 1e3) + ...
        0.15 * safe_metric(trial.mode3, 'position_rms', 1e3) + penalty_for_mode(trial.mode3);
end
if isfield(trial, 'mode5')
    score = score + safe_metric(trial.mode5, 'position_mean_error_m', 1e3) + ...
        0.15 * safe_metric(trial.mode5, 'position_rms', 1e3) + penalty_for_mode(trial.mode5);
end

function penalty = penalty_for_mode(metrics)
penalty = 0;
if safe_metric(metrics, 'is_divergent', 1) ~= 0
    penalty = penalty + 1e6;
end
if isfield(metrics, 'arm_axis_max')
    penalty = penalty + 10 * sum(max(0, metrics.arm_axis_max - 0.10));
else
    penalty = penalty + 1e3;
end
penalty = penalty + 2 * max(0, safe_metric(metrics, 'position_max', 1e3) - 0.25);
penalty = penalty + 0.5 * finite_sum([ ...
    safe_metric(metrics, 'thrust_saturation_ratio', NaN), ...
    safe_metric(metrics, 'torque_saturation_ratio', NaN), ...
    safe_metric(metrics, 'arm_torque_saturation_ratio', NaN)]);

function value = safe_metric(metrics, field, default_value)
if isfield(metrics, field) && ~isempty(metrics.(field))
    value = double(metrics.(field));
else
    value = default_value;
end

function total = finite_sum(values)
values = values(isfinite(values));
total = sum(values);

function output_file = get_output_file(result)
output_file = '';
if isfield(result, 'output_file')
    output_file = result.output_file;
end

function write_trial(output_root, idx, trial)
save(fullfile(output_root, sprintf('trial_%04d.mat', idx)), 'trial');
write_text(fullfile(output_root, sprintf('trial_%04d.json', idx)), ...
    jsonencode(strip_large_fields(trial), 'PrettyPrint', true));

function summary = build_summary(opts, trials, best)
summary = struct();
summary.options = opts;
summary.threshold_m = opts.Threshold;
summary.trials = trials(~cellfun('isempty', trials));
summary.best = struct();
if ~isempty(best.trial)
    summary.best = strip_large_fields(best.trial);
end
summary.passed = isfield(summary.best, 'passed') && summary.best.passed;

function write_summary(output_root, summary)
save(fullfile(output_root, 'summary.mat'), 'summary');
write_text(fullfile(output_root, 'summary.json'), ...
    jsonencode(summary, 'PrettyPrint', true));

function out = strip_large_fields(in)
out = in;
if isfield(out, 'candidate') && isfield(out.candidate, 'vector_names')
    out.candidate = rmfield(out.candidate, 'vector_names');
end

function console = compact_console_summary(summary)
console = struct();
console.output_root = summary.options.OutputRoot;
console.algorithm = summary.options.Algorithm;
console.scenario = summary.options.Scenario;
console.threshold_m = summary.threshold_m;
console.passed = summary.passed;
if isfield(summary.best, 'score')
    console.best_score = summary.best.score;
    console.best_passed = summary.best.passed;
    if isfield(summary.best, 'mode3')
        console.mode3_mean = summary.best.mode3.position_mean_error_m;
    end
    if isfield(summary.best, 'mode5')
        console.mode5_mean = summary.best.mode5.position_mean_error_m;
    end
end

function write_text(path, text)
fid = fopen(path, 'w');
if fid < 0
    error('simulink_uam_autotune: failed to open %s for writing.', path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s', text);

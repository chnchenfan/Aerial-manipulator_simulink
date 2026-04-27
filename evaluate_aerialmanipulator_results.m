function metrics = evaluate_aerialmanipulator_results(signals, config)
%EVALUATE_AERIALMANIPULATOR_RESULTS Compute quantitative metrics for tuning.

if nargin < 2
    config = struct();
end

[p_actual, p_desired] = pair_signal_prefer_true(signals, 'p_true', 'p_actual', 'p_desired');
[q_actual, q_desired] = pair_signal_prefer_true(signals, 'q_true', 'q_actual', 'q_desired');
[att, ~] = pair_signal_prefer_true(signals, 'att_true', 'att', 'att');
f = get_signal_data(signals, 'f');
tau = get_signal_data(signals, 'tau');
tau_arm = get_signal_data(signals, 'tau_arm');
[h_v_true, h_v_est] = pair_signal(signals, 'h_v_true', 'h_v_est');
[p_true_for_eso, p_hat] = pair_signal(signals, 'p_true', 'p_hat');

metrics = struct();

if ~isempty(p_actual) && ~isempty(p_desired)
    pos_error = p_actual - p_desired;
    pos_error_abs = abs(pos_error);
    pos_error_norm = vecnorm(pos_error, 2, 2);

    metrics.position_axis_mean = mean(pos_error_abs, 1);
    metrics.position_axis_max = max(pos_error_abs, [], 1);
    metrics.position_axis_rms = sqrt(mean(pos_error .^ 2, 1));
    metrics.position_rms = sqrt(mean(pos_error_norm .^ 2));
    metrics.position_max = max(pos_error_norm);
    metrics.position_steady = mean(pos_error_abs(end-min(99, size(pos_error_abs,1)-1):end, :), 1);
    metrics.height_rms = sqrt(mean(pos_error(:,3) .^ 2));
    metrics.height_max = max(pos_error_abs(:,3));
    metrics.disturbance_amp_position = amplitude_at_frequency(pos_error_norm, config);
    metrics.track_overshoot = overshoot_ratio(p_actual(:,3), p_desired(:,3));
    metrics.track_settling_time = settling_time(p_actual(:,3), p_desired(:,3), 0.02, get_sample_time(config));
end

if ~isempty(h_v_true) && ~isempty(h_v_est)
    eso_error = h_v_est - h_v_true;
    eso_error_abs = abs(eso_error);

    metrics.eso_disturbance_axis_mean_error = mean(eso_error_abs, 1);
    metrics.eso_disturbance_axis_max_error = max(eso_error_abs, [], 1);
    metrics.eso_disturbance_mean_error = mean(vecnorm(eso_error, 2, 2));
    metrics.eso_disturbance_max_error = max(vecnorm(eso_error, 2, 2));
end

if ~isempty(p_true_for_eso) && ~isempty(p_hat)
    eso_position_error = p_hat - p_true_for_eso;
    eso_position_error_abs = abs(eso_position_error);

    metrics.eso_position_axis_mean_error_m = mean(eso_position_error_abs, 1);
    metrics.eso_position_axis_max_error_m = max(eso_position_error_abs, [], 1);
    metrics.eso_position_mean_error_m = mean(vecnorm(eso_position_error, 2, 2));
    metrics.eso_position_max_error_m = max(vecnorm(eso_position_error, 2, 2));
end

if ~isempty(att)
    att_norm = vecnorm(att, 2, 2);
    metrics.attitude_rms = sqrt(mean(att_norm .^ 2));
    metrics.attitude_peak = max(max(abs(att), [], 2));
    metrics.disturbance_amp_attitude = amplitude_at_frequency(att_norm, config);
end

if ~isempty(q_actual) && ~isempty(q_desired)
    q_error = q_actual - q_desired;
    q_error_abs = abs(q_error);
    q_error_norm = vecnorm(q_error, 2, 2);

    metrics.arm_axis_max = max(q_error_abs, [], 1);
    metrics.arm_axis_rms = sqrt(mean(q_error .^ 2, 1));
    metrics.arm_rms = sqrt(mean(q_error_norm .^ 2));
    metrics.arm_max = max(q_error_norm);
end

params = common_functions('get_system_params');
thrust_bounds = [0.1 * (params.m_B + params.m_M) * params.g, ...
    2.0 * (params.m_B + params.m_M) * params.g];
if isfield(config, 'controller') && isstruct(config.controller) && ...
        isfield(config.controller, 'px4_like') && ...
        isfield(config.controller.px4_like, 'thrust_max_ratio')
    thrust_bounds(2) = config.controller.px4_like.thrust_max_ratio * ...
        (params.m_B + params.m_M) * params.g;
end
metrics.thrust_saturation_ratio = saturation_ratio(f, thrust_bounds);
torque_bounds = [-50, 50];
if isfield(config, 'controller') && isstruct(config.controller) && ...
        isfield(config.controller, 'px4_like_attitude') && ...
        isfield(config.controller.px4_like_attitude, 'tau_max')
    tau_max = max(abs(config.controller.px4_like_attitude.tau_max(:)));
    torque_bounds = [-tau_max, tau_max];
end
metrics.torque_saturation_ratio = saturation_ratio(tau, torque_bounds);

arm_params = get_arm_params_from_config(config);
metrics.arm_torque_saturation_ratio = saturation_ratio(tau_arm, ...
    [-max(abs(arm_params.tau_max)), max(abs(arm_params.tau_max))]);

metrics.is_divergent = false;
if isfield(metrics, 'position_axis_max')
    metrics.is_divergent = any(metrics.position_axis_max > 10) || ...
        (isfield(metrics, 'attitude_peak') && metrics.attitude_peak > pi/2);
end

metrics.mode1_pass = false;
metrics.mode2_pass = false;
if isfield(metrics, 'position_axis_max') && isfield(metrics, 'arm_axis_max')
    eso_pass = true;
    if isfield(metrics, 'eso_position_axis_max_error_m')
        eso_pass = all(metrics.eso_position_axis_max_error_m < 0.01);
    end
    pass_flag = all(metrics.position_axis_mean < 0.02) && ...
        all(metrics.position_axis_max < 0.04) && ...
        all(metrics.arm_axis_max < 0.10) && ...
        eso_pass && ...
        ~metrics.is_divergent;
    if isfield(config, 'input') && isfield(config.input, 'mode')
        if config.input.mode == 1
            metrics.mode1_pass = pass_flag;
        elseif config.input.mode == 2
            metrics.mode2_pass = pass_flag;
        end
    end
end

function [a, b] = pair_signal(signals, first_name, second_name)
a = get_signal_data(signals, first_name);
b = get_signal_data(signals, second_name);

if isempty(a) || isempty(b)
    return;
end

n = min(size(a,1), size(b,1));
a = a(1:n, :);
b = b(1:n, :);

function [a, b] = pair_signal_prefer_true(signals, primary_name, fallback_name, second_name)
a = get_signal_data(signals, primary_name);
if isempty(a)
    a = get_signal_data(signals, fallback_name);
end

b = get_signal_data(signals, second_name);

if isempty(a) || isempty(b)
    return;
end

n = min(size(a,1), size(b,1));
a = a(1:n, :);
b = b(1:n, :);

function data = get_signal_data(signals, name)
data = [];
if isfield(signals, name) && isfield(signals.(name), 'data')
    data = signals.(name).data;
end

function amp = amplitude_at_frequency(signal, config)
amp = NaN;
if isempty(signal) || numel(signal) < 8
    return;
end

freq = 0.5;
if isfield(config, 'input') && isfield(config.input, 'arm_frequency_hz')
    freq = config.input.arm_frequency_hz;
end

dt = get_sample_time(config);
signal = signal(:) - mean(signal(:));
N = numel(signal);
Y = fft(signal);
f_axis = (0:N-1)' / (N * dt);
[~, idx] = min(abs(f_axis - freq));
amp = 2 * abs(Y(idx)) / N;

function ratio = saturation_ratio(signal, bounds)
ratio = NaN;
if isempty(signal)
    return;
end

signal = signal(:);
ratio = mean(signal <= bounds(1) + 1e-6 | signal >= bounds(2) - 1e-6);

function ratio = overshoot_ratio(actual, desired)
ratio = NaN;
if isempty(actual) || isempty(desired)
    return;
end

target = mean(desired(end-min(49, numel(desired)-1):end));
if abs(target) < 1e-9
    ratio = 0;
    return;
end

peak = max(actual);
ratio = max(0, (peak - target) / abs(target));

function value = settling_time(actual, desired, tolerance_ratio, dt)
value = NaN;
if isempty(actual) || isempty(desired)
    return;
end

target = mean(desired(end-min(49, numel(desired)-1):end));
tolerance = max(0.01, tolerance_ratio * max(1, abs(target)));
idx = find(abs(actual - target) > tolerance, 1, 'last');
if isempty(idx)
    value = 0;
else
    value = idx * dt;
end

function dt = get_sample_time(config)
dt = 0.01;
if isfield(config, 'sim') && isfield(config.sim, 'sample_time')
    dt = config.sim.sample_time;
end

function params = get_arm_params_from_config(config)
params = struct('tau_max', [10; 10; 10]);

if isfield(config, 'controller') && isfield(config.controller, 'arm_params')
    params = merge_structs(params, config.controller.arm_params);
end

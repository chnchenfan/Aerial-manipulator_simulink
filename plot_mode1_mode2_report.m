function report = plot_mode1_mode2_report(mode1_file, mode2_file, output_dir)
%PLOT_MODE1_MODE2_REPORT Generate report figures for final Mode 1/2 runs.

if nargin < 1 || isempty(mode1_file)
    mode1_file = fullfile(pwd, 'tuning_results', 'strict_validation', ...
        'strict_mode1_final_20260427_152137.mat');
end
if nargin < 2 || isempty(mode2_file)
    mode2_file = fullfile(pwd, 'tuning_results', 'strict_validation', ...
        'strict_mode2_final_20260427_152156.mat');
end
if nargin < 3 || isempty(output_dir)
    output_dir = fullfile(pwd, 'figures');
end
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

report = struct();
report.mode1 = plot_one_mode(mode1_file, output_dir, 'mode1', 'Mode 1 Hover With Arm Motion');
report.mode2 = plot_one_mode(mode2_file, output_dir, 'mode2', 'Mode 2 Square Tracking With Arm Motion');
write_summary(report, output_dir);

function summary = plot_one_mode(mat_file, output_dir, prefix, title_prefix)
loaded = load(mat_file);
result = loaded.result;
data = extract_data(result);
metrics = compute_report_metrics(data, result.metrics);

summary = metrics;
summary.file = mat_file;

plot_position_tracking(data, output_dir, prefix, title_prefix);
plot_noise_delay_psd(data, result.config, output_dir, prefix, title_prefix);
plot_arm_tracking(data, output_dir, prefix, title_prefix);
plot_3d_error(data, output_dir, prefix, title_prefix);
plot_eso_estimates(data, output_dir, prefix, title_prefix);
plot_eso_errors(data, output_dir, prefix, title_prefix);

function data = extract_data(result)
signals = result.signals;
data.t = signal_time(signals, 'p_true');
data.p_true = signal_data(signals, 'p_true');
data.p_actual = signal_data(signals, 'p_actual');
data.p_desired = signal_data(signals, 'p_desired');
data.q_true = signal_data(signals, 'q_true');
data.q_actual = signal_data(signals, 'q_actual');
data.q_desired = signal_data(signals, 'q_desired');
data.h_v_est = optional_signal(signals, 'h_v_est', data.t, 3);
data.h_v_true = optional_signal(signals, 'h_v_true', data.t, 3);
data.p_hat = optional_signal(signals, 'p_hat', data.t, 3);
data.pE_true = optional_signal(signals, 'pE_true', data.t, 3);
data.h_omega_est = optional_signal(signals, 'h_omega_est', data.t, 3);

n = min([numel(data.t), size(data.p_true,1), size(data.p_actual,1), ...
    size(data.p_desired,1), size(data.q_true,1), size(data.q_actual,1), ...
    size(data.q_desired,1), size(data.h_v_est,1), size(data.h_v_true,1), ...
    size(data.p_hat,1), size(data.pE_true,1), size(data.h_omega_est,1)]);
fields = fieldnames(data);
for i = 1:numel(fields)
    value = data.(fields{i});
    if size(value, 1) >= n
        data.(fields{i}) = value(1:n, :);
    end
end

data.p_error = data.p_true - data.p_desired;
data.p_abs_error = abs(data.p_error);
data.q_error = data.q_true - data.q_desired;
data.position_error_norm = vecnorm(data.p_error, 2, 2);
data.eso_position_error = data.p_hat - data.p_true;
data.eso_disturbance_error = data.h_v_est - data.h_v_true;
data.p_noise_residual = data.p_actual - data.p_true;
data.p_delay_residual = delay_residual(data.t, data.p_true, 0.010);
data.q_noise_residual = data.q_actual - data.q_true;
data.q_delay_residual = delay_residual(data.t, data.q_true, 0.001);

function metrics = compute_report_metrics(data, base_metrics)
metrics = struct();
metrics.position_mean_error_m = mean(data.position_error_norm);
metrics.position_rms_m = sqrt(mean(data.position_error_norm .^ 2));
metrics.position_max_error_m = max(data.position_error_norm);
metrics.position_axis_mean_m = mean(abs(data.p_error), 1);
metrics.position_axis_max_m = max(abs(data.p_error), [], 1);
metrics.eso_position_axis_max_m = max(abs(data.eso_position_error), [], 1);
metrics.eso_disturbance_axis_max = max(abs(data.eso_disturbance_error), [], 1);
metrics.arm_axis_max_rad = max(abs(data.q_error), [], 1);
metrics.arm_max_error_rad = max(vecnorm(data.q_error, 2, 2));
metrics.is_divergent = base_metrics.is_divergent;

function plot_position_tracking(data, output_dir, prefix, title_prefix)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 780]);
labels = {'X (m)', 'Y (m)', 'Z (m)'};
for i = 1:3
    subplot(3, 1, i);
    plot(data.t, data.p_desired(:, i), 'r--', 'LineWidth', 1.2); hold on;
    plot(data.t, data.p_true(:, i), 'b-', 'LineWidth', 1.1);
    grid on;
    ylabel(labels{i});
    if i == 1
        title([title_prefix ' - UAV Position Tracking']);
        legend('Desired', 'Actual', 'Location', 'best');
    end
end
xlabel('Time (s)');
save_figure(fig, output_dir, [prefix '_uav_position_tracking.png']);

function plot_noise_delay_psd(data, config, output_dir, prefix, title_prefix)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 780]);
subplot(2, 1, 1);
plot_psd_lines(data.t, data.p_noise_residual, {'p_x noise/delay residual', 'p_y noise/delay residual', 'p_z noise/delay residual'});
title([title_prefix ' - UAV Measurement Residual PSD']);
subplot(2, 1, 2);
plot_psd_lines(data.t, data.p_delay_residual, {'p_x 10 ms delay residual', 'p_y 10 ms delay residual', 'p_z 10 ms delay residual'});
title(sprintf('%s - Delay Residual PSD (quad delay steps=%d, jitter=%d, dropout=%.3g)', ...
    title_prefix, config.measurement.quad_delay_steps, ...
    config.measurement.quad_delay_jitter_steps, config.measurement.quad_dropout_prob));
save_figure(fig, output_dir, [prefix '_noise_delay_psd.png']);

function plot_arm_tracking(data, output_dir, prefix, title_prefix)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 780]);
for i = 1:3
    subplot(3, 1, i);
    plot(data.t, data.q_desired(:, i), 'r--', 'LineWidth', 1.2); hold on;
    plot(data.t, data.q_true(:, i), 'b-', 'LineWidth', 1.1);
    grid on;
    ylabel(sprintf('q%d (rad)', i));
    if i == 1
        title([title_prefix ' - Arm Joint Tracking']);
        legend('Desired', 'Actual', 'Location', 'best');
    end
end
xlabel('Time (s)');
save_figure(fig, output_dir, [prefix '_arm_tracking.png']);

function plot_3d_error(data, output_dir, prefix, title_prefix)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1120 520]);
subplot(1, 2, 1);
running_mean = cumsum(data.position_error_norm) ./ (1:numel(data.position_error_norm))';
plot(data.t, data.position_error_norm, 'Color', [0.55 0.55 0.55], 'LineWidth', 0.8); hold on;
plot(data.t, running_mean, 'b-', 'LineWidth', 1.5);
yline(0.04, 'k--', '0.04 m max threshold');
grid on;
xlabel('Time (s)'); ylabel('3D position error (m)');
title(sprintf('%s - 3D Mean Error = %.4f m', title_prefix, mean(data.position_error_norm)));
legend('Instantaneous 3D error', 'Running mean', 'Location', 'best');

subplot(1, 2, 2);
plot3(data.p_desired(:,1), data.p_desired(:,2), data.p_desired(:,3), 'r--', 'LineWidth', 1.2); hold on;
plot3(data.p_true(:,1), data.p_true(:,2), data.p_true(:,3), 'b-', 'LineWidth', 1.1);
plot3(data.p_true(1,1), data.p_true(1,2), data.p_true(1,3), 'go', 'MarkerFaceColor', 'g');
plot3(data.p_true(end,1), data.p_true(end,2), data.p_true(end,3), 'ro', 'MarkerFaceColor', 'r');
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title([title_prefix ' - UAV 3D Position']);
legend('Desired UAV path', 'Actual UAV path', 'Start', 'End', 'Location', 'best');
save_figure(fig, output_dir, [prefix '_uav_3d_mean_error.png']);

function plot_eso_estimates(data, output_dir, prefix, title_prefix)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 780]);
labels = {'x', 'y', 'z'};
for i = 1:3
    subplot(3, 1, i);
    plot(data.t, data.h_v_est(:, i), 'b-', 'LineWidth', 1.0); hold on;
    plot(data.t, data.h_v_true(:, i), 'r--', 'LineWidth', 1.0);
    grid on;
    ylabel(sprintf('h_v %s (m/s^2)', labels{i}));
    if i == 1
        title([title_prefix ' - ESO Disturbance Estimate']);
        legend('ESO estimate', 'Model disturbance', 'Location', 'best');
    end
end
xlabel('Time (s)');
save_figure(fig, output_dir, [prefix '_eso_disturbance_estimate.png']);

function plot_eso_errors(data, output_dir, prefix, title_prefix)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 780]);
labels = {'x', 'y', 'z'};
for i = 1:3
    subplot(3, 1, i);
    plot(data.t, data.eso_disturbance_error(:, i), 'Color', [0.00 0.36 0.68], 'LineWidth', 1.0); hold on;
    plot(data.t, abs(data.eso_disturbance_error(:, i)), 'Color', [0.85 0.33 0.10], 'LineWidth', 0.9);
    grid on;
    ylabel(sprintf('e_{h,%s} (m/s^2)', labels{i}));
    if i == 1
        title([title_prefix ' - ESO Disturbance Estimation Error']);
        legend('Signed error', 'Absolute error', 'Location', 'best');
    end
end
xlabel('Time (s)');
save_figure(fig, output_dir, [prefix '_eso_disturbance_error.png']);

function plot_psd_lines(t, matrix, labels)
hold on;
for i = 1:size(matrix, 2)
    [freq, psd_val] = compute_psd(t, matrix(:, i));
    plot(freq, 10 * log10(psd_val + eps), 'LineWidth', 1.1);
end
grid on;
xlabel('Frequency (Hz)');
ylabel('PSD (dB/Hz)');
legend(labels, 'Location', 'best');
xlim([0 10]);

function residual = delay_residual(t, values, delay_seconds)
residual = zeros(size(values));
for i = 1:size(values, 2)
    delayed = interp1(t, values(:, i), max(t(1), t - delay_seconds), 'linear', 'extrap');
    residual(:, i) = values(:, i) - delayed;
end

function [freq, psd_val] = compute_psd(t, signal)
signal = signal(:) - mean(signal(:), 'omitnan');
dt = median(diff(t));
n = numel(signal);
window = hann(n);
nfft = max(256, 2^nextpow2(n));
[psd_val, freq] = periodogram(signal, window, nfft, 1 / dt);

function t = signal_time(signals, name)
if ~isfield(signals, name)
    error('Missing signal %s.', name);
end
t = signals.(name).time(:);

function data = signal_data(signals, name)
if ~isfield(signals, name)
    error('Missing signal %s.', name);
end
data = signals.(name).data;

function data = optional_signal(signals, name, t, width)
if isfield(signals, name) && isfield(signals.(name), 'data')
    data = signals.(name).data;
else
    data = nan(numel(t), width);
end

function save_figure(fig, output_dir, name)
path = fullfile(output_dir, name);
saveas(fig, path);
close(fig);

function write_summary(report, output_dir)
path = fullfile(output_dir, 'mode1_mode2_report_metrics.txt');
fid = fopen(path, 'w');
if fid < 0
    error('Unable to write %s.', path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'Mode 1 mean position error: %.6f m\n', report.mode1.position_mean_error_m);
fprintf(fid, 'Mode 1 axis mean position error: [%.6f %.6f %.6f] m\n', report.mode1.position_axis_mean_m);
fprintf(fid, 'Mode 1 axis max position error: [%.6f %.6f %.6f] m\n', report.mode1.position_axis_max_m);
fprintf(fid, 'Mode 1 ESO position max error: [%.6f %.6f %.6f] m\n', report.mode1.eso_position_axis_max_m);
fprintf(fid, 'Mode 1 ESO disturbance max error: [%.6f %.6f %.6f] m/s^2\n', report.mode1.eso_disturbance_axis_max);
fprintf(fid, 'Mode 1 max arm error: %.6f rad\n', report.mode1.arm_max_error_rad);
fprintf(fid, 'Mode 1 divergent: %d\n', report.mode1.is_divergent);
fprintf(fid, 'Mode 2 mean position error: %.6f m\n', report.mode2.position_mean_error_m);
fprintf(fid, 'Mode 2 axis mean position error: [%.6f %.6f %.6f] m\n', report.mode2.position_axis_mean_m);
fprintf(fid, 'Mode 2 axis max position error: [%.6f %.6f %.6f] m\n', report.mode2.position_axis_max_m);
fprintf(fid, 'Mode 2 ESO position max error: [%.6f %.6f %.6f] m\n', report.mode2.eso_position_axis_max_m);
fprintf(fid, 'Mode 2 ESO disturbance max error: [%.6f %.6f %.6f] m/s^2\n', report.mode2.eso_disturbance_axis_max);
fprintf(fid, 'Mode 2 max arm error: %.6f rad\n', report.mode2.arm_max_error_rad);
fprintf(fid, 'Mode 2 divergent: %d\n', report.mode2.is_divergent);




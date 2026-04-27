function report = plot_px4_like_comparison(mode3_eso_file, mode5_eso_file, mode3_px4_file, mode5_px4_file, output_dir)
%PLOT_PX4_LIKE_COMPARISON Plot paper ESO vs conservative PX4-like baseline.

if nargin < 5 || isempty(output_dir)
    output_dir = fullfile(pwd, 'figures', 'px4_like_comparison');
end
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

cases = {
    'mode3', 'paper_eso', mode3_eso_file;
    'mode3', 'px4_like', mode3_px4_file;
    'mode5', 'paper_eso', mode5_eso_file;
    'mode5', 'px4_like', mode5_px4_file};

report = struct();
for i = 1:size(cases, 1)
    loaded = load(cases{i, 3}, 'result');
    data = extract_case_data(loaded.result);
    report.(cases{i, 1}).(cases{i, 2}) = loaded.result.metrics;
    report.(cases{i, 1}).([cases{i, 2} '_data']) = data;
end

plot_mode_pair(report.mode3.paper_eso_data, report.mode3.px4_like_data, output_dir, 'mode3');
plot_mode_pair(report.mode5.paper_eso_data, report.mode5.px4_like_data, output_dir, 'mode5');
write_metric_table(report, output_dir);

function data = extract_case_data(result)
signals = result.signals;
data.p_true = signal_data(signals, 'p_true');
data.p_actual = fallback_signal(signals, 'p_actual', data.p_true);
data.p_desired = signal_data(signals, 'p_desired');
data.q_true = signal_data(signals, 'q_true');
data.q_desired = signal_data(signals, 'q_desired');
data.f = signal_data(signals, 'f');
data.tau = signal_data(signals, 'tau');
data.metrics = result.metrics;
data.t = signal_time(signals, 'p_true', size(data.p_true, 1));

n = min([numel(data.t), size(data.p_true,1), size(data.p_desired,1), ...
    size(data.q_true,1), size(data.q_desired,1)]);
fields = fieldnames(data);
for i = 1:numel(fields)
    value = data.(fields{i});
    if isnumeric(value) && size(value, 1) >= n
        data.(fields{i}) = value(1:n, :);
    end
end
data.p_error = data.p_true - data.p_desired;
data.q_error = data.q_true - data.q_desired;
data.position_error_norm = vecnorm(data.p_error, 2, 2);

function plot_mode_pair(eso, px4, output_dir, mode_label)
labels = {'x', 'y', 'z'};

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1160 780]);
for i = 1:3
    subplot(3, 1, i);
    plot(eso.t, eso.p_true(:, i), 'b-', 'LineWidth', 1.1); hold on;
    plot(px4.t, px4.p_true(:, i), 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
    plot(eso.t, eso.p_desired(:, i), 'k--', 'LineWidth', 0.9);
    grid on;
    ylabel(sprintf('%s (m)', labels{i}));
    if i == 1
        title(sprintf('%s UAV position tracking: paper ESO vs PX4-like', upper(mode_label)));
        legend('paper\_eso actual', 'px4\_like actual', 'desired', 'Location', 'best');
    end
end
xlabel('Time (s)');
save_figure(fig, output_dir, sprintf('%s_position_tracking_eso_vs_px4_like.png', mode_label));

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1160 780]);
for i = 1:3
    subplot(3, 1, i);
    plot(eso.t, eso.p_error(:, i), 'b-', 'LineWidth', 1.0); hold on;
    plot(px4.t, px4.p_error(:, i), 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
    grid on;
    ylabel(sprintf('e_%s (m)', labels{i}));
    if i == 1
        title(sprintf('%s axis position error', upper(mode_label)));
        legend('paper\_eso', 'px4\_like', 'Location', 'best');
    end
end
xlabel('Time (s)');
save_figure(fig, output_dir, sprintf('%s_position_error_eso_vs_px4_like.png', mode_label));

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1120 520]);
subplot(1, 2, 1);
plot3(eso.p_desired(:,1), eso.p_desired(:,2), eso.p_desired(:,3), 'k--', 'LineWidth', 1.0); hold on;
plot3(eso.p_true(:,1), eso.p_true(:,2), eso.p_true(:,3), 'b-', 'LineWidth', 1.1);
plot3(px4.p_true(:,1), px4.p_true(:,2), px4.p_true(:,3), 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('%s 3D trajectory', upper(mode_label)));
legend('desired', 'paper\_eso', 'px4\_like', 'Location', 'best');

subplot(1, 2, 2);
plot(eso.t, eso.position_error_norm, 'b-', 'LineWidth', 1.0); hold on;
plot(px4.t, px4.position_error_norm, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
grid on;
xlabel('Time (s)'); ylabel('3D position error (m)');
title(sprintf('Mean error: ESO %.4f m, PX4-like %.4f m', ...
    mean(eso.position_error_norm), mean(px4.position_error_norm)));
legend('paper\_eso', 'px4\_like', 'Location', 'best');
save_figure(fig, output_dir, sprintf('%s_3d_mean_error_eso_vs_px4_like.png', mode_label));

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1160 780]);
for i = 1:3
    subplot(3, 1, i);
    plot(eso.t, eso.q_error(:, i), 'b-', 'LineWidth', 1.0); hold on;
    plot(px4.t, px4.q_error(:, i), 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
    grid on;
    ylabel(sprintf('q%d err (rad)', i));
    if i == 1
        title(sprintf('%s arm tracking error', upper(mode_label)));
        legend('paper\_eso', 'px4\_like', 'Location', 'best');
    end
end
xlabel('Time (s)');
save_figure(fig, output_dir, sprintf('%s_arm_tracking_eso_vs_px4_like.png', mode_label));

function write_metric_table(report, output_dir)
fid = fopen(fullfile(output_dir, 'px4_like_metrics_summary.txt'), 'w');
if fid < 0
    error('Unable to write PX4-like metric summary');
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, 'controller,mode,position_axis_mean,position_axis_max,position_rms,position_max,arm_axis_max,is_divergent,thrust_saturation_ratio,torque_saturation_ratio\n');
write_metrics(fid, 'paper_eso', 'mode3', report.mode3.paper_eso);
write_metrics(fid, 'px4_like', 'mode3', report.mode3.px4_like);
write_metrics(fid, 'paper_eso', 'mode5', report.mode5.paper_eso);
write_metrics(fid, 'px4_like', 'mode5', report.mode5.px4_like);

function write_metrics(fid, controller, mode_label, metrics)
fprintf(fid, '%s,%s,%s,%s,%.6f,%.6f,%s,%d,%.6f,%.6f\n', ...
    controller, mode_label, vec_to_string(metrics.position_axis_mean), ...
    vec_to_string(metrics.position_axis_max), metrics.position_rms, ...
    metrics.position_max, vec_to_string(metrics.arm_axis_max), ...
    metrics.is_divergent, metrics.thrust_saturation_ratio, ...
    metrics.torque_saturation_ratio);

function text = vec_to_string(v)
text = sprintf('[%.6f %.6f %.6f]', v(1), v(2), v(3));

function data = fallback_signal(signals, name, fallback)
data = signal_data(signals, name);
if isempty(data)
    data = fallback;
end

function data = signal_data(signals, name)
data = [];
if isfield(signals, name)
    value = signals.(name);
    if isstruct(value) && isfield(value, 'data')
        data = value.data;
    elseif isnumeric(value)
        data = value;
    end
end

function t = signal_time(signals, name, sample_count)
if isfield(signals, name) && isfield(signals.(name), 'time')
    t = signals.(name).time(:);
else
    t = (0:sample_count - 1)' * 0.01;
end

function save_figure(fig, output_dir, filename)
exportgraphics(fig, fullfile(output_dir, filename), 'Resolution', 180);
close(fig);

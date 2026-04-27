function plot_mode2_data(mat_file)
%PLOT_MODE2_DATA Plot mode 2 tracking, PSD, and MSE figures.

if nargin < 1 || isempty(mat_file)
    mat_file = find_latest_result('mode2');
end

result = load_result_file(mat_file);
data = extract_plot_data(result);

plots_dir = fullfile(fileparts(mat_file), 'plots');
if ~exist(plots_dir, 'dir')
    mkdir(plots_dir);
end

[~, base_name, ~] = fileparts(mat_file);

fig1 = figure('Name', 'Mode 2 UAV Position Tracking', 'Color', 'w');
for i = 1:3
    subplot(3,1,i);
    plot(data.t, data.p_desired(:,i), 'r--', data.t, data.p_actual(:,i), 'b-', 'LineWidth', 1.3);
    ylabel(sprintf('%c (m)', 'X' + i - 1));
    grid on;
    if i == 1
        title('Mode 2 UAV Three-Axis Position Tracking');
        legend('Desired', 'Actual', 'Location', 'best');
    end
end
xlabel('Time (s)');
saveas(fig1, fullfile(plots_dir, [base_name '_uav_tracking.png']));

fig2 = figure('Name', 'Mode 2 Arm Tracking', 'Color', 'w');
for i = 1:3
    subplot(3,1,i);
    plot(data.t, data.q_desired(:,i), 'r--', data.t, data.q_actual(:,i), 'b-', 'LineWidth', 1.3);
    ylabel(sprintf('q%d (rad)', i));
    grid on;
    if i == 1
        title('Mode 2 Arm Three-Axis Tracking');
        legend('Desired', 'Actual', 'Location', 'best');
    end
end
xlabel('Time (s)');
saveas(fig2, fullfile(plots_dir, [base_name '_arm_tracking.png']));

fig3 = figure('Name', 'Mode 2 Measurement PSD', 'Color', 'w');
plot_psd_group(data.t, data.p_measurement_error, {'p_x', 'p_y', 'p_z'}, 1, 'UAV Measurement Error PSD');
plot_psd_group(data.t, data.q_measurement_error, {'q_1', 'q_2', 'q_3'}, 2, 'Arm Measurement Error PSD');
saveas(fig3, fullfile(plots_dir, [base_name '_measurement_psd.png']));

fig4 = figure('Name', 'Mode 2 UAV Running MSE', 'Color', 'w');
running_mse = compute_running_mse(data.p_actual - data.p_desired);
plot(data.t, running_mse(:,1), 'LineWidth', 1.3); hold on;
plot(data.t, running_mse(:,2), 'LineWidth', 1.3);
plot(data.t, running_mse(:,3), 'LineWidth', 1.3);
grid on;
xlabel('Time (s)');
ylabel('Running MSE (m^2)');
title('Mode 2 UAV Position Running MSE');
legend('MSE_x', 'MSE_y', 'MSE_z', 'Location', 'best');
saveas(fig4, fullfile(plots_dir, [base_name '_uav_mse.png']));

fprintf('Saved Mode 2 plots to %s\n', plots_dir);

function plot_psd_group(t, signal_matrix, labels, subplot_index, plot_title)
subplot(2,1,subplot_index);
hold on;
for i = 1:size(signal_matrix, 2)
    [freq, psd_vals] = compute_psd(t, signal_matrix(:,i));
    plot(freq, 10 * log10(psd_vals + eps), 'LineWidth', 1.2);
end
grid on;
xlabel('Frequency (Hz)');
ylabel('PSD (dB/Hz)');
title(plot_title);
legend(labels, 'Location', 'best');
xlim([0, min(10, max(freq))]);

function running_mse = compute_running_mse(error_matrix)
n = size(error_matrix, 1);
running_mse = zeros(size(error_matrix));
cumulative = zeros(1, size(error_matrix, 2));
for k = 1:n
    cumulative = cumulative + error_matrix(k,:).^2;
    running_mse(k,:) = cumulative / k;
end

function [freq, psd_vals] = compute_psd(t, signal)
signal = signal(:);
dt = median(diff(t));
signal = signal - mean(signal);
n = numel(signal);
window = hann(n);
nfft = max(256, 2^nextpow2(n));
[psd_vals, freq] = periodogram(signal, window, nfft, 1 / dt);

function data = extract_plot_data(result)
signals = result.signals;
data.t = get_signal_time(signals, {'p_true', 'p_actual', 'p_desired'});
data.p_actual = get_signal_data(signals, {'p_true', 'p_actual'});
data.p_desired = get_signal_data(signals, {'p_desired'});
data.q_actual = get_signal_data(signals, {'q_true', 'q_actual'});
data.q_desired = get_signal_data(signals, {'q_desired', 'q_desired_ctrl'});
data.p_measured = get_signal_data(signals, {'p_actual'});
data.q_measured = get_signal_data(signals, {'q_actual'});

n = min([size(data.p_actual,1), size(data.p_desired,1), size(data.q_actual,1), size(data.q_desired,1), size(data.p_measured,1), size(data.q_measured,1), numel(data.t)]);
data.t = data.t(1:n);
data.p_actual = data.p_actual(1:n,:);
data.p_desired = data.p_desired(1:n,:);
data.q_actual = data.q_actual(1:n,:);
data.q_desired = data.q_desired(1:n,:);
data.p_measured = data.p_measured(1:n,:);
data.q_measured = data.q_measured(1:n,:);
data.p_measurement_error = data.p_measured - data.p_actual;
data.q_measurement_error = data.q_measured - data.q_actual;

function t = get_signal_time(signals, names)
t = [];
for i = 1:numel(names)
    if isfield(signals, names{i}) && isfield(signals.(names{i}), 'time')
        t = signals.(names{i}).time;
        return;
    end
end
error('No time signal found in result.');

function data = get_signal_data(signals, names)
data = [];
for i = 1:numel(names)
    if isfield(signals, names{i}) && isfield(signals.(names{i}), 'data')
        data = signals.(names{i}).data;
        return;
    end
end
error('Missing required signal: %s', strjoin(names, ', '));

function file = find_latest_result(prefix)
files = dir(fullfile(pwd, 'tuning_results', [prefix '*.mat']));
if isempty(files)
    error('No result files found for prefix %s.', prefix);
end
[~, idx] = max([files.datenum]);
file = fullfile(files(idx).folder, files(idx).name);

function result = load_result_file(mat_file)
loaded = load(mat_file);
if isfield(loaded, 'result')
    result = loaded.result;
else
    error('Unsupported result file format: %s', mat_file);
end




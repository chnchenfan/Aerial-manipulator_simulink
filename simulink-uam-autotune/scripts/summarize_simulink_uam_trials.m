function payload = summarize_simulink_uam_trials(varargin)
%SUMMARIZE_SIMULINK_UAM_TRIALS Rank Simulink UAM autotune artifacts.

opts = parse_options(varargin{:});
root = fullfile(opts.ProjectRoot, 'tuning_results');
rows = {};

if exist(root, 'dir')
    files = dir(fullfile(root, 'autotune_*', 'summary.json'));
    for i = 1:numel(files)
        path = fullfile(files(i).folder, files(i).name);
        rows = [rows; collect_summary(path)]; %#ok<AGROW>
    end
end

payload = struct();
payload.project_root = opts.ProjectRoot;
payload.rows = rows;
payload.best = struct();

if ~isempty(rows)
    scores = cellfun(@(row) row.score, rows);
    [~, order] = sort(scores);
    rows = rows(order);
    payload.rows = rows;
    payload.best = rows{1};
end

if opts.Json
    disp(jsonencode(payload, 'PrettyPrint', true));
else
    print_rows(rows, opts.Limit);
end

function opts = parse_options(varargin)
script_dir = fileparts(mfilename('fullpath'));
opts = struct( ...
    'ProjectRoot', fileparts(fileparts(script_dir)), ...
    'Limit', 12, ...
    'Json', false);

if mod(numel(varargin), 2) ~= 0
    error('summarize_simulink_uam_trials: options must be name-value pairs.');
end

for i = 1:2:numel(varargin)
    name = char(varargin{i});
    if ~isfield(opts, name)
        error('summarize_simulink_uam_trials: unknown option %s.', name);
    end
    opts.(name) = varargin{i + 1};
end

opts.ProjectRoot = char(opts.ProjectRoot);
opts.Limit = double(opts.Limit);
opts.Json = logical(opts.Json);

function rows = collect_summary(path)
rows = {};
try
    data = jsondecode(fileread(path));
catch
    return;
end

if ~isfield(data, 'trials')
    return;
end

trials = data.trials;
if isstruct(trials)
    trials = num2cell(trials);
end

for i = 1:numel(trials)
    trial = trials{i};
    row = row_from_trial(trial, path);
    rows{end + 1, 1} = row; %#ok<AGROW>
end

function row = row_from_trial(trial, path)
row = struct();
row.summary_path = path;
row.index = get_field(trial, 'index', NaN);
row.score = get_field(trial, 'score', inf);
row.passed = get_field(trial, 'passed', false);
row.name = '';

if isfield(trial, 'candidate') && isfield(trial.candidate, 'name')
    row.name = trial.candidate.name;
end

row.mode3_mean = NaN;
row.mode5_mean = NaN;
row.mode3_rms = NaN;
row.mode5_rms = NaN;
row.error = '';

if isfield(trial, 'mode3')
    row.mode3_mean = get_field(trial.mode3, 'position_mean_error_m', NaN);
    row.mode3_rms = get_field(trial.mode3, 'position_rms', NaN);
end
if isfield(trial, 'mode5')
    row.mode5_mean = get_field(trial.mode5, 'position_mean_error_m', NaN);
    row.mode5_rms = get_field(trial.mode5, 'position_rms', NaN);
end
if isfield(trial, 'error')
    row.error = trial.error;
end

function value = get_field(s, field, default_value)
if isstruct(s) && isfield(s, field)
    value = s.(field);
else
    value = default_value;
end

function print_rows(rows, limit)
fprintf('Best Simulink UAM autotune trials:\n');
fprintf('score      pass  mode3_mean  mode5_mean  mode3_rms   mode5_rms   candidate\n');

n = min(limit, numel(rows));
for i = 1:n
    row = rows{i};
    fprintf('%9.4f  %4d  %10.4f  %10.4f  %9.4f  %9.4f   %s\n', ...
        row.score, row.passed, row.mode3_mean, row.mode5_mean, ...
        row.mode3_rms, row.mode5_rms, row.name);
end

if isempty(rows)
    fprintf('No autotune summaries found under tuning_results/autotune_*.\n');
end

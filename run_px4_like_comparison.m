function outputs = run_px4_like_comparison(varargin)
%RUN_PX4_LIKE_COMPARISON Fresh paper ESO vs PX4-like mode 3/mode 5 runs.

opts = parse_opts(varargin{:});
out_dir = opts.OutputDir;
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

outputs = struct();
outputs.paper_eso.mode3 = run_case('paper_eso', 'AerialManipulatorSystem', 3, out_dir, opts.StopTime);
outputs.paper_eso.mode5 = run_case('paper_eso', 'AerialManipulatorSystem', 5, out_dir, opts.StopTime);
outputs.px4_like.mode3 = run_case('px4_like', 'AerialManipulatorSystem_PX4Like', 3, out_dir, opts.StopTime);
outputs.px4_like.mode5 = run_case('px4_like', 'AerialManipulatorSystem_PX4Like', 5, out_dir, opts.StopTime);

outputs.figure_dir = fullfile(pwd, 'figures', 'px4_like_comparison');
plot_px4_like_comparison(outputs.paper_eso.mode3.output_file, ...
    outputs.paper_eso.mode5.output_file, ...
    outputs.px4_like.mode3.output_file, ...
    outputs.px4_like.mode5.output_file, ...
    outputs.figure_dir);

save(fullfile(out_dir, 'px4_like_comparison_outputs.mat'), 'outputs');

function result = run_case(controller_label, model_name, mode, out_dir, stop_time)
config = struct();
config.input = struct('mode', mode);
config.sim = struct('model_name', model_name, 'stop_time', stop_time);
config.output = struct( ...
    'label', sprintf('%s_mode%d', controller_label, mode), ...
    'save_dir', out_dir, ...
    'save_results', true);
config.controller = struct('type', controller_label);

if strcmp(controller_label, 'px4_like')
    config.controller.px4_like = struct();
    config.controller.px4_like_attitude = struct();
end

result = run_aerialmanipulator_experiment(config);

function opts = parse_opts(varargin)
opts = struct( ...
    'OutputDir', fullfile(pwd, 'tuning_results', 'px4_like_comparison'), ...
    'StopTime', 100.0);

if mod(numel(varargin), 2) ~= 0
    error('run_px4_like_comparison: options must be name-value pairs');
end

for i = 1:2:numel(varargin)
    opts.(varargin{i}) = varargin{i + 1};
end

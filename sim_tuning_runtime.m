function varargout = sim_tuning_runtime(action, varargin)
%SIM_TUNING_RUNTIME Runtime config and signal log manager for batch tuning.

persistent runtime;

if isempty(runtime)
    runtime = struct('config', struct(), 'log', struct());
end

switch lower(action)
    case 'set_config'
        if nargin < 2 || ~isstruct(varargin{1})
            error('sim_tuning_runtime:set_config expects a config struct.');
        end
        runtime.config = varargin{1};
        runtime.log = struct();

    case 'get_config'
        varargout{1} = runtime.config;

    case 'reset_log'
        runtime.log = struct();

    case 'log'
        if nargin < 4
            error('sim_tuning_runtime:log expects name, time, and value.');
        end
        if ~is_logging_enabled(runtime.config)
            return;
        end

        signal_name = matlab.lang.makeValidName(varargin{1});
        t = varargin{2};
        value = varargin{3};

        if isempty(value)
            return;
        end

        value_row = reshape(double(value), 1, []);

        if ~isfield(runtime.log, signal_name)
            runtime.log.(signal_name) = struct('time', [], 'data', []);
        end

        runtime.log.(signal_name).time(end + 1, 1) = double(t);
        runtime.log.(signal_name).data(end + 1, :) = value_row;

    case 'get_log'
        varargout{1} = runtime.log;

    case 'reset_all'
        runtime = struct('config', struct(), 'log', struct());

    otherwise
        error('sim_tuning_runtime: unknown action %s', action);
end

function enabled = is_logging_enabled(config)
enabled = true;

if isfield(config, 'logging') && isstruct(config.logging) ...
        && isfield(config.logging, 'enabled')
    enabled = logical(config.logging.enabled);
end

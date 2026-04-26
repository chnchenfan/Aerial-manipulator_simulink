function plot_mode2_data(varargin)
%PERFORMANCE_ANALYSIS 飞行机械臂系统性能分析工具 (增强版)
%
% 功能：计算关键性能指标并生成可视化图表，支持数据保存和多数据源
%
% 用法：
%   plot_mode2_data()                           - 从工作区加载数据
%   plot_mode2_data('workspace')                - 从工作区加载数据
%   plot_mode2_data('mat', 'filename.mat')      - 从MAT文件加载数据
%   plot_mode2_data('workspace', 'save', 'filename.mat') - 从工作区加载数据并保存到MAT文件
%
% 参数：
%   data_source  - 数据源：'workspace'(默认) 或 'mat'
%   action       - 操作：'save' (保存工作区数据到文件)
%   filename     - MAT文件名 (用于加载或保存)
%
% 使用前确保以下变量已从Simulink输出到工作区：
%   time_series      - 时间向量 [N×1]
%   quad_pos_actual  - 无人机实际位置 [N×3]
%   quad_pos_desired - 无人机期望位置 [N×3]
%   quad_att_actual  - 无人机实际姿态 [N×3]
%   quad_att_desired - 无人机期望姿态 [N×3]
%   arm_pos_actual   - 机械臂实际关节角 [N×3]
%   arm_pos_desired  - 机械臂期望关节角 [N×3]
%   control_thrust   - 推力控制输入 [N×1]
%   control_torque   - 力矩控制输入 [N×3]
%   arm_torque      - 机械臂控制力矩 [N×3]

clc;close all;
fprintf('=== 飞行机械臂系统性能分析 (增强版) ===\n\n');

% 解析输入参数
[data_source, save_flag, mat_filename] = parse_input_arguments(varargin);

fprintf('📋 分析配置：\n');
fprintf('   数据源：%s\n', data_source);
if save_flag
    fprintf('   保存数据到：%s\n', mat_filename);
end
if strcmp(data_source, 'mat')
    fprintf('   MAT文件：%s\n', mat_filename);
end
fprintf('\n');

% 1. 数据验证和加载
if strcmp(data_source, 'workspace')
    if ~validate_workspace_data()
        return;
    end
    % 从工作区加载数据
    data = load_simulation_data();
    % 如果需要，保存数据到MAT文件
    if save_flag
        save_workspace_data_to_mat(mat_filename);
    end
elseif strcmp(data_source, 'mat')
    if ~validate_mat_file(mat_filename)
        return;
    end
    % 从MAT文件加载数据
    data = load_data_from_mat(mat_filename);
else
    error('❌ 无效的数据源：%s', data_source);
end

% 2. 计算性能指标
metrics = calculate_performance_metrics(data);

% 3. 打印分析结果
print_analysis_results(metrics);

% 4. 生成可视化图表
create_visualization_plots(data, metrics);

fprintf('\n✅ 性能分析完成！\n');
if save_flag
    fprintf('✅ 数据已保存到：%s\n', mat_filename);
end

end

%=============================================================================
% 输入参数解析
%=============================================================================

function [data_source, save_flag, mat_filename] = parse_input_arguments(args)
%解析输入参数

% 默认值
data_source = 'workspace';
save_flag = false;
mat_filename = '';

if isempty(args)
    return;
end

% 解析参数
i = 1;
while i <= length(args)
    arg = args{i};
    
    if ischar(arg) || isstring(arg)
        arg = char(arg);
        
        switch lower(arg)
            case 'workspace'
                data_source = 'workspace';
                
            case 'mat'
                data_source = 'mat';
                % 下一个参数应该是文件名
                if i + 1 <= length(args)
                    mat_filename = char(args{i + 1});
                    i = i + 1;  % 跳过文件名参数
                else
                    error('❌ 使用 "mat" 选项时必须指定文件名');
                end
                
            case 'save'
                save_flag = true;
                % 下一个参数应该是文件名
                if i + 1 <= length(args)
                    mat_filename = char(args{i + 1});
                    i = i + 1;  % 跳过文件名参数
                else
                    error('❌ 使用 "save" 选项时必须指定文件名');
                end
                
            otherwise
                % 可能是文件名，检查是否以.mat结尾
                if endsWith(arg, '.mat', 'IgnoreCase', true)
                    if strcmp(data_source, 'mat') && isempty(mat_filename)
                        mat_filename = arg;
                    elseif save_flag && isempty(mat_filename)
                        mat_filename = arg;
                    end
                else
                    warning('⚠️  忽略未识别的参数：%s', arg);
                end
        end
    end
    
    i = i + 1;
end

% 验证参数组合
if strcmp(data_source, 'mat') && isempty(mat_filename)
    error('❌ 使用MAT文件数据源时必须指定文件名');
end

if save_flag && isempty(mat_filename)
    % 生成默认文件名
    mat_filename = sprintf('simulation_data_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
    fprintf('💡 未指定保存文件名，使用默认文件名：%s\n', mat_filename);
end

% 确保MAT文件名有正确的扩展名
if ~isempty(mat_filename) && ~endsWith(mat_filename, '.mat', 'IgnoreCase', true)
    mat_filename = [mat_filename, '.mat'];
end

end

%=============================================================================
% MAT文件相关功能
%=============================================================================

function save_workspace_data_to_mat(filename)
%将工作区数据保存到MAT文件

fprintf('💾 正在保存工作区数据到MAT文件...\n');

% 定义需要保存的变量
required_vars = {
    'quad_pos_actual', 'quad_pos_desired', 'quad_att_actual', ...
    'arm_pos_actual', 'arm_pos_desired', 'control_thrust', ...
    'control_torque', 'arm_torque'
};

% 可选变量
optional_vars = {
    'time_series', 'tout', 'simout', 'desired_R_matrix', ...
    'quaternion_error', 'attitude_error', 'quad_att_desired'
};

% 准备保存的数据结构
save_data = struct();
save_data.saved_time = datestr(now);
save_data.matlab_version = version;

% 检测和保存时间数据
time_var_names = {'time_series', 'tout', 'simout'};
for i = 1:length(time_var_names)
    if evalin('base', sprintf('exist(''%s'', ''var'')', time_var_names{i}))
        var_data = evalin('base', time_var_names{i});
        if isa(var_data, 'timeseries')
            save_data.time_vector = var_data.Time;
            save_data.time_source = time_var_names{i};
            fprintf('   ✅ 时间数据: %s\n', time_var_names{i});
            break;
        elseif isnumeric(var_data) && isvector(var_data)
            save_data.time_vector = var_data(:);
            save_data.time_source = time_var_names{i};
            fprintf('   ✅ 时间数据: %s\n', time_var_names{i});
            break;
        end
    end
end

% 如果没有找到独立的时间数据，从第一个时间序列变量提取
if ~isfield(save_data, 'time_vector')
    if evalin('base', sprintf('exist(''%s'', ''var'')', required_vars{1}))
        var_data = evalin('base', required_vars{1});
        if isa(var_data, 'timeseries')
            save_data.time_vector = var_data.Time;
            save_data.time_source = [required_vars{1}, '_extracted'];
            fprintf('   ✅ 时间数据: 从 %s 提取\n', required_vars{1});
        end
    end
end

% 保存必需变量
fprintf('   📦 保存必需变量:\n');
for i = 1:length(required_vars)
    var_name = required_vars{i};
    if evalin('base', sprintf('exist(''%s'', ''var'')', var_name))
        var_data = evalin('base', var_name);
        
        % 提取数值数据
        if isa(var_data, 'timeseries')
            numeric_data = extract_timeseries_data(var_data);
            save_data.(var_name) = numeric_data;
            save_data.([var_name '_type']) = 'timeseries';
        elseif isa(var_data, 'Simulink.SimulationData.Dataset')
            if var_data.numElements > 0
                ts_data = var_data.getElement(1);
                numeric_data = extract_timeseries_data(ts_data.Values);
                save_data.(var_name) = numeric_data;
                save_data.([var_name '_type']) = 'dataset';
            end
        elseif isnumeric(var_data)
            save_data.(var_name) = var_data;
            save_data.([var_name '_type']) = 'numeric';
        end
        
        fprintf('     - %s [%dx%d]\n', var_name, size(save_data.(var_name), 1), size(save_data.(var_name), 2));
    end
end

% 保存可选变量
fprintf('   📦 保存可选变量:\n');
optional_saved = 0;
for i = 1:length(optional_vars)
    var_name = optional_vars{i};
    if evalin('base', sprintf('exist(''%s'', ''var'')', var_name))
        var_data = evalin('base', var_name);
        
        % 跳过已经处理的时间数据
        if strcmp(var_name, save_data.time_source)
            continue;
        end
        
        % 提取数值数据
        if isa(var_data, 'timeseries')
            numeric_data = extract_timeseries_data(var_data);
            save_data.(var_name) = numeric_data;
            save_data.([var_name '_type']) = 'timeseries';
        elseif isa(var_data, 'Simulink.SimulationData.Dataset')
            if var_data.numElements > 0
                ts_data = var_data.getElement(1);
                numeric_data = extract_timeseries_data(ts_data.Values);
                save_data.(var_name) = numeric_data;
                save_data.([var_name '_type']) = 'dataset';
            end
        elseif isnumeric(var_data)
            save_data.(var_name) = var_data;
            save_data.([var_name '_type']) = 'numeric';
        end
        
        if isfield(save_data, var_name)
            fprintf('     - %s [%dx%d]\n', var_name, size(save_data.(var_name), 1), size(save_data.(var_name), 2));
            optional_saved = optional_saved + 1;
        end
    end
end

if optional_saved == 0
    fprintf('     (无可选变量)\n');
end

% 保存数据到文件
try
    save(filename, '-struct', 'save_data');
    fprintf('✅ 数据保存成功：%s\n', filename);
    
    % 显示文件信息
    file_info = dir(filename);
    fprintf('   文件大小：%.2f MB\n', file_info.bytes / 1024 / 1024);
    
catch ME
    error('❌ 保存文件失败：%s', ME.message);
end

end

function numeric_data = extract_timeseries_data(ts_data)
%从时间序列数据中提取数值数据

if isa(ts_data, 'timeseries')
    numeric_data = ts_data.Data;
else
    numeric_data = ts_data;
end

% 处理维度
if ndims(numeric_data) == 3
    numeric_data = squeeze(numeric_data);
end

% 确保正确的矩阵格式
if size(numeric_data, 2) == 1 && size(numeric_data, 1) > 1
    % 列向量，保持不变
elseif size(numeric_data, 1) == 1 && size(numeric_data, 2) > 1
    % 行向量，转置为列
    numeric_data = numeric_data';
end

end

function is_valid = validate_mat_file(filename)
%验证MAT文件是否存在且包含必需数据

fprintf('🔍 验证MAT文件：%s\n', filename);

% 检查文件是否存在
if ~exist(filename, 'file')
    fprintf('❌ 错误：MAT文件不存在：%s\n', filename);
    is_valid = false;
    return;
end

% 尝试加载文件并检查内容
try
    mat_data = load(filename);
    fprintf('✅ MAT文件加载成功\n');
    
    % 检查必需变量
    required_vars = {
        'quad_pos_actual', 'quad_pos_desired', 'quad_att_actual', ...
        'arm_pos_actual', 'arm_pos_desired', 'control_thrust', ...
        'control_torque', 'arm_torque'
    };
    
    missing_vars = {};
    for i = 1:length(required_vars)
        if ~isfield(mat_data, required_vars{i})
            missing_vars{end+1} = required_vars{i};
        end
    end
    
    if ~isempty(missing_vars)
        fprintf('❌ 错误：MAT文件中缺少以下必需变量：\n');
        for i = 1:length(missing_vars)
            fprintf('   - %s\n', missing_vars{i});
        end
        is_valid = false;
        return;
    end
    
    % 检查时间数据
    if ~isfield(mat_data, 'time_vector')
        fprintf('❌ 错误：MAT文件中缺少时间向量数据\n');
        is_valid = false;
        return;
    end
    
    % 显示文件信息
    fprintf('📊 MAT文件信息：\n');
    if isfield(mat_data, 'saved_time')
        fprintf('   保存时间：%s\n', mat_data.saved_time);
    end
    if isfield(mat_data, 'time_source')
        fprintf('   时间数据来源：%s\n', mat_data.time_source);
    end
    fprintf('   数据点数：%d\n', length(mat_data.time_vector));
    
    % 显示包含的变量
    fields = fieldnames(mat_data);
    data_fields = fields(~contains(fields, {'_type', 'saved_time', 'matlab_version', 'time_source'}));
    fprintf('   包含变量：%s\n', strjoin(data_fields, ', '));
    
    fprintf('✅ MAT文件验证通过\n\n');
    is_valid = true;
    
catch ME
    fprintf('❌ 错误：无法读取MAT文件：%s\n', ME.message);
    is_valid = false;
end

end

function data = load_data_from_mat(filename)
%从MAT文件加载仿真数据

fprintf('📂 从MAT文件加载数据：%s\n', filename);

% 加载MAT文件
mat_data = load(filename);

% 构建数据结构
data = struct();

% 加载基础时间数据
data.time = mat_data.time_vector;
data.N = length(data.time);
if data.N >= 2
    data.dt = data.time(2) - data.time(1);
else
    data.dt = 0.01;  % 默认采样时间
end

% 加载无人机数据
data.quad_pos_actual = mat_data.quad_pos_actual;
data.quad_pos_desired = mat_data.quad_pos_desired;
data.quad_att_actual = mat_data.quad_att_actual;

% 检查姿态相关数据
data.has_desired_attitude = false;
data.has_quaternion_error = false;
data.has_attitude_error = false;

if isfield(mat_data, 'desired_R_matrix')
    data.quad_att_desired = convert_rotation_matrix_to_euler(mat_data.desired_R_matrix);
    data.has_desired_attitude = true;
    fprintf('📐 已从旋转矩阵转换期望姿态角\n');
elseif isfield(mat_data, 'quad_att_desired')
    data.quad_att_desired = mat_data.quad_att_desired;
    data.has_desired_attitude = true;
    fprintf('📐 已加载期望姿态角\n');
end

if isfield(mat_data, 'quaternion_error')
    data.quaternion_error = mat_data.quaternion_error;
    data.has_quaternion_error = true;
    fprintf('📐 已加载四元数误差数据\n');
end

if isfield(mat_data, 'attitude_error')
    data.attitude_error = mat_data.attitude_error;
    data.has_attitude_error = true;
    fprintf('📐 已加载姿态误差数据\n');
end

% 加载机械臂数据
data.arm_pos_actual = mat_data.arm_pos_actual;
data.arm_pos_desired = mat_data.arm_pos_desired;

% 加载控制输入数据
data.control_thrust = mat_data.control_thrust;
data.control_torque = mat_data.control_torque;
data.arm_torque = mat_data.arm_torque;

fprintf('📊 已从MAT文件加载数据：%d个时间点，采样时间%.3fs\n', data.N, data.dt);

end

%=============================================================================
% 数据验证和加载 (原有函数，保持不变)
%=============================================================================

function is_valid = validate_workspace_data()
%验证工作区中是否存在必需的变量（支持时间序列格式）

required_vars = {
    'quad_pos_actual', 'quad_pos_desired', 'quad_att_actual', ...
    'arm_pos_actual', 'arm_pos_desired', 'control_thrust', ...
    'control_torque', 'arm_torque'
};

% 可选变量（姿态相关）
optional_vars = {
    'desired_R_matrix', 'quaternion_error', 'attitude_error'
};

% 时间数据变量（至少需要一个）
time_vars = {'time_series', 'tout', 'simout'};

fprintf('🔍 检查工作区数据...\n');

% 检查时间数据
time_found = false;
for i = 1:length(time_vars)
    if evalin('base', sprintf('exist(''%s'', ''var'')', time_vars{i}))
        var_data = evalin('base', time_vars{i});
        if isa(var_data, 'timeseries') || (isnumeric(var_data) && isvector(var_data))
            time_found = true;
            fprintf('✅ 找到时间数据: %s (%s)\n', time_vars{i}, class(var_data));
            break;
        end
    end
end

if ~time_found
    % 如果没有专门的时间变量，尝试从第一个数据变量中提取时间
    if evalin('base', sprintf('exist(''%s'', ''var'')', required_vars{1}))
        var_data = evalin('base', required_vars{1});
        if isa(var_data, 'timeseries')
            time_found = true;
            fprintf('✅ 从 %s 提取时间数据\n', required_vars{1});
        end
    end
end

if ~time_found
    fprintf('❌ 错误：未找到时间数据！\n');
    fprintf('请确保工作区中存在以下任一时间变量：%s\n', strjoin(time_vars, ', '));
    is_valid = false;
    return;
end

% 检查必需变量
missing_vars = {};
invalid_vars = {};

for i = 1:length(required_vars)
    var_name = required_vars{i};
    if ~evalin('base', sprintf('exist(''%s'', ''var'')', var_name))
        missing_vars{end+1} = var_name;
    else
        % 检查数据类型
        var_data = evalin('base', var_name);
        if isa(var_data, 'timeseries')
            fprintf('✅ %s (时间序列)\n', var_name);
        elseif isa(var_data, 'Simulink.SimulationData.Dataset')
            fprintf('✅ %s (Simulink数据集)\n', var_name);
        elseif isnumeric(var_data)
            fprintf('✅ %s (数值矩阵)\n', var_name);
        else
            invalid_vars{end+1} = sprintf('%s (%s)', var_name, class(var_data));
        end
    end
end

% 报告缺失变量
if ~isempty(missing_vars)
    fprintf('❌ 错误：以下必需变量在工作区中不存在：\n');
    for i = 1:length(missing_vars)
        fprintf('   - %s\n', missing_vars{i});
    end
end

% 报告无效数据类型
if ~isempty(invalid_vars)
    fprintf('❌ 错误：以下变量数据类型不支持：\n');
    for i = 1:length(invalid_vars)
        fprintf('   - %s\n', invalid_vars{i});
    end
end

if ~isempty(missing_vars) || ~isempty(invalid_vars)
    fprintf('\n💡 提示：请确保使用 ToWorkspace 模块输出以下变量：\n');
    for i = 1:length(required_vars)
        fprintf('   - %s\n', required_vars{i});
    end
    fprintf('\n📋 ToWorkspace 模块设置建议：\n');
    fprintf('   - 保存格式：时间序列 (Timeseries)\n');
    fprintf('   - 变量名：使用上述准确的变量名\n');
    fprintf('   - 记录时间：勾选（如果需要独立时间向量）\n');
    is_valid = false;
    return;
end

% 检查可选的姿态变量
available_attitude_vars = {};
for i = 1:length(optional_vars)
    if evalin('base', sprintf('exist(''%s'', ''var'')', optional_vars{i}))
        var_data = evalin('base', optional_vars{i});
        if isa(var_data, 'timeseries') || isa(var_data, 'Simulink.SimulationData.Dataset') || isnumeric(var_data)
            available_attitude_vars{end+1} = optional_vars{i};
            fprintf('✅ %s (姿态数据，%s)\n', optional_vars{i}, class(var_data));
        end
    end
end

if ~isempty(available_attitude_vars)
    fprintf('📐 可用姿态分析数据: %s\n', strjoin(available_attitude_vars, ', '));
else
    fprintf('⚠️  未检测到姿态分析数据，将跳过姿态跟踪分析\n');
end

fprintf('✅ 数据验证通过\n\n');
is_valid = true;

end

function data = load_simulation_data()
%从工作区加载仿真数据（支持时间序列格式）

data = struct();

% 检测数据格式并加载基础时间数据
[data.time, data.dt, data.N] = load_time_data();

% 加载无人机位置数据
data.quad_pos_actual = load_timeseries_data('quad_pos_actual');
data.quad_pos_desired = load_timeseries_data('quad_pos_desired');
data.quad_att_actual = load_timeseries_data('quad_att_actual');

% 姿态相关数据（可选）
data.has_desired_attitude = false;
data.has_quaternion_error = false;
data.has_attitude_error = false;

% 检查是否有期望旋转矩阵
if evalin('base', 'exist(''desired_R_matrix'', ''var'')')
    desired_R_raw = load_timeseries_data('desired_R_matrix');
    % 从旋转矩阵转换为欧拉角
    data.quad_att_desired = convert_rotation_matrix_to_euler(desired_R_raw);
    data.has_desired_attitude = true;
    fprintf('📐 已从旋转矩阵转换期望姿态角\n');
end

% 检查是否有四元数误差
if evalin('base', 'exist(''quaternion_error'', ''var'')')
    data.quaternion_error = load_timeseries_data('quaternion_error');
    data.has_quaternion_error = true;
    fprintf('📐 已加载四元数误差数据\n');
end

% 检查是否有姿态误差
if evalin('base', 'exist(''attitude_error'', ''var'')')
    data.attitude_error = load_timeseries_data('attitude_error');
    data.has_attitude_error = true;
    fprintf('📐 已加载姿态误差数据\n');
end

% 机械臂数据
data.arm_pos_actual = load_timeseries_data('arm_pos_actual');
data.arm_pos_desired = load_timeseries_data('arm_pos_desired');

% 控制输入数据
data.control_thrust = load_timeseries_data('control_thrust');
data.control_torque = load_timeseries_data('control_torque');
data.arm_torque = load_timeseries_data('arm_torque');

fprintf('📊 已加载数据：%d个时间点，采样时间%.3fs\n', data.N, data.dt);

end

function [time_vector, dt, N] = load_time_data()
%智能加载时间数据，支持多种格式

% 尝试不同的时间数据变量名
time_var_names = {'time_series', 'quad_pos_actual', 'tout', 'simout'};

time_vector = [];
for i = 1:length(time_var_names)
    if evalin('base', sprintf('exist(''%s'', ''var'')', time_var_names{i}))
        var_data = evalin('base', time_var_names{i});
        
        if isa(var_data, 'timeseries')
            time_vector = var_data.Time;
            fprintf('⏰ 从时间序列 %s 提取时间数据\n', time_var_names{i});
            break;
        elseif isnumeric(var_data) && isvector(var_data)
            time_vector = var_data(:);
            fprintf('⏰ 使用数值向量 %s 作为时间数据\n', time_var_names{i});
            break;
        end
    end
end

if isempty(time_vector)
    error('❌ 无法找到时间数据！请确保工作区中存在时间序列数据。');
end

N = length(time_vector);
if N < 2
    error('❌ 时间数据点数不足！');
end

dt = time_vector(2) - time_vector(1);

end

function output_data = load_timeseries_data(var_name)
%加载时间序列数据并提取数值

if ~evalin('base', sprintf('exist(''%s'', ''var'')', var_name))
    error('❌ 变量 %s 在工作区中不存在！', var_name);
end

var_data = evalin('base', var_name);

if isa(var_data, 'timeseries')
    % 时间序列对象
    output_data = var_data.Data;
    
    % 处理不同的数据维度
    if ndims(output_data) == 3
        % 如果是 [N×1×M] 格式，重塑为 [N×M]
        output_data = squeeze(output_data);
    end
    
    % 确保是列向量或矩阵格式
    if size(output_data, 2) == 1 && size(output_data, 1) > 1
        % 单列数据，保持不变
    elseif size(output_data, 1) == 1 && size(output_data, 2) > 1
        % 单行数据，转置为列
        output_data = output_data';
    end
    
    fprintf('📥 已加载时间序列: %s [%dx%d]\n', var_name, size(output_data, 1), size(output_data, 2));
    
elseif isa(var_data, 'Simulink.SimulationData.Dataset')
    % Simulink数据集格式
    % 尝试提取第一个元素
    if var_data.numElements > 0
        ts_data = var_data.getElement(1);
        output_data = ts_data.Values.Data;
        if ndims(output_data) == 3
            output_data = squeeze(output_data);
        end
        fprintf('📥 已加载数据集: %s [%dx%d]\n', var_name, size(output_data, 1), size(output_data, 2));
    else
        error('❌ 数据集 %s 为空！', var_name);
    end
    
elseif isnumeric(var_data)
    % 直接数值数据
    output_data = var_data;
    fprintf('📥 已加载数值数据: %s [%dx%d]\n', var_name, size(output_data, 1), size(output_data, 2));
    
else
    error('❌ 不支持的数据类型: %s (%s)', var_name, class(var_data));
end

% 验证数据
if isempty(output_data)
    error('❌ 变量 %s 的数据为空！', var_name);
end

end

function euler_angles = convert_rotation_matrix_to_euler(R_matrix_data)
%将旋转矩阵数据转换为欧拉角
% R_matrix_data: [N×9] 格式 [R11,R21,R31,R12,R22,R32,R13,R23,R33]

if size(R_matrix_data, 2) == 9
    % 标准的 [N×9] 格式
    N = size(R_matrix_data, 1);
    euler_angles = zeros(N, 3);
    
    for i = 1:N
        % 重构旋转矩阵 [R11,R21,R31,R12,R22,R32,R13,R23,R33] -> 3×3
        R = reshape(R_matrix_data(i, :), 3, 3)';
        
        % 转换为欧拉角 (ZYX序列)
        sy = sqrt(R(1,1)^2 + R(2,1)^2);
        singular = sy < 1e-6;
        
        if ~singular
            roll = atan2(R(3,2), R(3,3));
            pitch = atan2(-R(3,1), sy);
            yaw = atan2(R(2,1), R(1,1));
        else
            roll = atan2(-R(2,3), R(2,2));
            pitch = atan2(-R(3,1), sy);
            yaw = 0;
        end
        
        euler_angles(i, :) = [roll, pitch, yaw];
    end
    
elseif size(R_matrix_data, 2) == 3 && size(R_matrix_data, 3) == 3
    % [N×3×3] 格式
    N = size(R_matrix_data, 1);
    euler_angles = zeros(N, 3);
    
    for i = 1:N
        R = squeeze(R_matrix_data(i, :, :));
        
        sy = sqrt(R(1,1)^2 + R(2,1)^2);
        singular = sy < 1e-6;
        
        if ~singular
            roll = atan2(R(3,2), R(3,3));
            pitch = atan2(-R(3,1), sy);
            yaw = atan2(R(2,1), R(1,1));
        else
            roll = atan2(-R(2,3), R(2,2));
            pitch = atan2(-R(3,1), sy);
            yaw = 0;
        end
        
        euler_angles(i, :) = [roll, pitch, yaw];
    end
else
    error('❌ 旋转矩阵数据格式不正确！期望 [N×9] 或 [N×3×3]，实际 [%dx%dx%d]', ...
          size(R_matrix_data, 1), size(R_matrix_data, 2), size(R_matrix_data, 3));
end

end

%=============================================================================
% 性能指标计算 (保持原有函数不变)
%=============================================================================

function metrics = calculate_performance_metrics(data)
%计算所有性能指标

fprintf('🔄 正在计算性能指标...\n');

metrics = struct();

% 1. 无人机位置跟踪指标
metrics.quad = calculate_quadrotor_metrics(data);

% 2. 机械臂跟踪指标
metrics.arm = calculate_arm_metrics(data);

% 3. 控制输入指标
metrics.control = calculate_control_metrics(data);

fprintf('✅ 性能指标计算完成\n\n');

end

function quad_metrics = calculate_quadrotor_metrics(data)
%计算无人机相关性能指标

quad_metrics = struct();

% 位置误差计算
pos_error_vec = data.quad_pos_actual - data.quad_pos_desired;
pos_error_norm = sqrt(sum(pos_error_vec.^2, 2));

% 1. 绝对位置误差
quad_metrics.absolute_error = pos_error_norm;
quad_metrics.max_absolute_error = max(pos_error_norm);
quad_metrics.final_absolute_error = pos_error_norm(end);

% 2. 径向误差（适用于螺旋轨迹）
r_actual = sqrt(data.quad_pos_actual(:,1).^2 + data.quad_pos_actual(:,2).^2);
r_desired = sqrt(data.quad_pos_desired(:,1).^2 + data.quad_pos_desired(:,2).^2);
quad_metrics.radial_error = abs(r_actual - r_desired);
quad_metrics.max_radial_error = max(quad_metrics.radial_error);

% 3. 均方根误差 (RMSE)
quad_metrics.rmse_position = sqrt(mean(pos_error_norm.^2));
quad_metrics.rmse_x = sqrt(mean(pos_error_vec(:,1).^2));
quad_metrics.rmse_y = sqrt(mean(pos_error_vec(:,2).^2));
quad_metrics.rmse_z = sqrt(mean(pos_error_vec(:,3).^2));

% 4. 平均绝对误差 (MAE)
quad_metrics.mae_position = mean(abs(pos_error_norm));
quad_metrics.mae_x = mean(abs(pos_error_vec(:,1)));
quad_metrics.mae_y = mean(abs(pos_error_vec(:,2)));
quad_metrics.mae_z = mean(abs(pos_error_vec(:,3)));

% 5. 稳态误差 (取最后20%数据)
steady_start = round(0.8 * data.N);
quad_metrics.steady_state_error = mean(pos_error_norm(steady_start:end));
quad_metrics.steady_state_error_x = mean(abs(pos_error_vec(steady_start:end, 1)));
quad_metrics.steady_state_error_y = mean(abs(pos_error_vec(steady_start:end, 2)));
quad_metrics.steady_state_error_z = mean(abs(pos_error_vec(steady_start:end, 3)));

% 6. 超调量（相对于期望值的百分比）
% 计算每个轴的超调
target_final = data.quad_pos_desired(end, :);
for axis = 1:3
    if abs(target_final(axis)) > 1e-6  % 避免除零
        peak_overshoot = max(abs(data.quad_pos_actual(:, axis) - target_final(axis)));
        quad_metrics.overshoot(axis) = (peak_overshoot / abs(target_final(axis))) * 100;
    else
        quad_metrics.overshoot(axis) = max(abs(data.quad_pos_actual(:, axis))) * 100;
    end
end
quad_metrics.max_overshoot = max(quad_metrics.overshoot);

% 姿态误差（根据可用数据计算）
if data.has_attitude_error
    % 如果直接有姿态误差数据
    quad_metrics.attitude_error = data.attitude_error;
    quad_metrics.rmse_attitude = sqrt(mean(quad_metrics.attitude_error.^2));
    quad_metrics.attitude_data_source = '直接姿态误差数据';
elseif data.has_quaternion_error
    % 如果有四元数误差，转换为标量误差
    quad_metrics.attitude_error = sqrt(sum(data.quaternion_error.^2, 2));
    quad_metrics.rmse_attitude = sqrt(mean(quad_metrics.attitude_error.^2));
    quad_metrics.attitude_data_source = '四元数误差转换';
elseif data.has_desired_attitude
    % 如果有期望姿态，计算传统姿态误差
    att_error_vec = data.quad_att_actual - data.quad_att_desired;
    att_error_vec(:, 3) = wrapToPi(att_error_vec(:, 3)); % 处理偏航角跳跃
    quad_metrics.attitude_error = sqrt(sum(att_error_vec.^2, 2));
    quad_metrics.rmse_attitude = sqrt(mean(quad_metrics.attitude_error.^2));
    quad_metrics.attitude_data_source = '欧拉角误差计算';
else
    % 没有姿态误差数据
    quad_metrics.attitude_error = [];
    quad_metrics.rmse_attitude = NaN;
    quad_metrics.attitude_data_source = '无姿态误差数据';
end

end

function arm_metrics = calculate_arm_metrics(data)
%计算机械臂相关性能指标

arm_metrics = struct();

% 关节角误差计算
arm_error_vec = data.arm_pos_actual - data.arm_pos_desired;
arm_error_norm = sqrt(sum(arm_error_vec.^2, 2));

% 1. 绝对位置误差（关节角误差）
arm_metrics.absolute_error = arm_error_norm;
arm_metrics.max_absolute_error = max(arm_error_norm);

% 2. 各关节RMSE
arm_metrics.rmse_joint1 = sqrt(mean(arm_error_vec(:,1).^2));
arm_metrics.rmse_joint2 = sqrt(mean(arm_error_vec(:,2).^2));
arm_metrics.rmse_joint3 = sqrt(mean(arm_error_vec(:,3).^2));
arm_metrics.rmse_total = sqrt(mean(arm_error_norm.^2));

% 3. 平均绝对误差
arm_metrics.mae_joint1 = mean(abs(arm_error_vec(:,1)));
arm_metrics.mae_joint2 = mean(abs(arm_error_vec(:,2)));
arm_metrics.mae_joint3 = mean(abs(arm_error_vec(:,3)));
arm_metrics.mae_total = mean(arm_error_norm);

% 4. 稳态误差
steady_start = round(0.8 * data.N);
arm_metrics.steady_state_error = mean(arm_error_norm(steady_start:end));

% 5. 超调量
target_final = data.arm_pos_desired(end, :);
for joint = 1:3
    if abs(target_final(joint)) > 1e-6
        peak_overshoot = max(abs(data.arm_pos_actual(:, joint) - target_final(joint)));
        arm_metrics.overshoot(joint) = (peak_overshoot / abs(target_final(joint))) * 100;
    else
        arm_metrics.overshoot(joint) = max(abs(data.arm_pos_actual(:, joint))) ; % 转换为度
    end
end
arm_metrics.max_overshoot = max(arm_metrics.overshoot);

end

function control_metrics = calculate_control_metrics(data)
%计算控制输入相关指标

control_metrics = struct();

% 7. 控制输入能量
thrust_energy = sum(data.control_thrust.^2) * data.dt;
torque_energy = sum(sum(data.control_torque.^2, 2)) * data.dt;
arm_energy = sum(sum(data.arm_torque.^2, 2)) * data.dt;

control_metrics.thrust_energy = thrust_energy;
control_metrics.torque_energy = torque_energy;
control_metrics.arm_energy = arm_energy;
control_metrics.total_energy = thrust_energy + torque_energy + arm_energy;

% 8. 控制输入平滑度（基于二阶差分）
control_metrics.thrust_smoothness = calculate_smoothness(data.control_thrust);
control_metrics.torque_smoothness = zeros(3, 1);
for i = 1:3
    control_metrics.torque_smoothness(i) = calculate_smoothness(data.control_torque(:, i));
end
control_metrics.arm_smoothness = zeros(3, 1);
for i = 1:3
    control_metrics.arm_smoothness(i) = calculate_smoothness(data.arm_torque(:, i));
end

% 9. 频域分析（新增）
fs = 1/data.dt;  % 采样频率
control_metrics.fft_analysis = struct();

% 推力频域分析
[control_metrics.fft_analysis.thrust_freq, control_metrics.fft_analysis.thrust_psd, ...
 control_metrics.fft_analysis.thrust_metrics] = analyze_frequency_domain(data.control_thrust, fs);

% 力矩频域分析
control_metrics.fft_analysis.torque_freq = control_metrics.fft_analysis.thrust_freq;  % 频率轴相同
for i = 1:3
    [~, control_metrics.fft_analysis.torque_psd(:,i), ...
     control_metrics.fft_analysis.torque_metrics(i)] = analyze_frequency_domain(data.control_torque(:,i), fs);
end

% 机械臂频域分析
for i = 1:3
    [~, control_metrics.fft_analysis.arm_psd(:,i), ...
     control_metrics.fft_analysis.arm_metrics(i)] = analyze_frequency_domain(data.arm_torque(:,i), fs);
end

% 控制输入统计
control_metrics.thrust_mean = mean(data.control_thrust);
control_metrics.thrust_std = std(data.control_thrust);
control_metrics.torque_mean = mean(data.control_torque, 1);
control_metrics.torque_std = std(data.control_torque, 1);

end

function smoothness = calculate_smoothness(signal)
%计算信号平滑度（基于相邻点差分的方差）
if length(signal) < 2
    smoothness = 0;
    return;
end

% 一阶差分
diff1 = diff(signal);
% 二阶差分
diff2 = diff(diff1);

% 平滑度指标：二阶差分的均方值（越小越平滑）
smoothness = mean(diff2.^2);

end

function [freq, psd, metrics] = analyze_frequency_domain(signal, fs)
%频域分析函数
% 输入：
%   signal - 时域信号
%   fs - 采样频率
% 输出：
%   freq - 频率向量
%   psd - 功率谱密度
%   metrics - 频域指标

% 确保信号长度为偶数（便于FFT）
N = length(signal);
if mod(N, 2) == 1
    signal = signal(1:end-1);
    N = N - 1;
end

% 去除直流分量
signal = signal - mean(signal);

% 应用窗函数减少频谱泄漏
window = hann(N);
signal_windowed = signal .* window;

% FFT变换
Y = fft(signal_windowed);
P2 = abs(Y/N).^2;  % 双边功率谱
P1 = P2(1:N/2+1);  % 单边功率谱
P1(2:end-1) = 2*P1(2:end-1);  % 补偿单边谱的能量

% 频率向量
freq = fs*(0:(N/2))/N;

% 功率谱密度（归一化）
psd = P1;

% 计算频域指标
metrics = struct();

% 1. 带宽指标（90%能量所在频率范围）
total_power = sum(psd);
cumulative_power = cumsum(psd);
f_10 = freq(find(cumulative_power >= 0.1*total_power, 1, 'first'));
f_90 = freq(find(cumulative_power >= 0.9*total_power, 1, 'first'));
metrics.bandwidth_90 = f_90 - f_10;

% 2. 主频率（能量最大的频率）
[~, max_idx] = max(psd(2:end));  % 排除直流分量
metrics.dominant_freq = freq(max_idx + 1);

% 3. 高频能量比例（>10Hz的能量占比）
high_freq_threshold = 10;  % Hz
high_freq_idx = freq > high_freq_threshold;
if any(high_freq_idx)
    metrics.high_freq_ratio = sum(psd(high_freq_idx)) / total_power;
else
    metrics.high_freq_ratio = 0;
end

% 4. 频谱重心（加权平均频率）
metrics.spectral_centroid = sum(freq .* psd) / total_power;

% 5. 平滑度指标（基于频谱梯度）
freq_gradient = gradient(psd);
metrics.spectral_smoothness = -sum(abs(freq_gradient));  % 负值，越大越平滑

% 6. 谐波失真度（检测周期性振荡）
% 寻找谱峰
[peaks, peak_locs] = findpeaks(psd, 'MinPeakHeight', 0.1*max(psd), 'MinPeakDistance', 5);
if length(peaks) > 1
    % 计算总谐波失真
    fundamental = max(peaks);
    harmonics = sum(peaks) - fundamental;
    metrics.thd = harmonics / fundamental;
else
    metrics.thd = 0;
end

end

%=============================================================================
% 结果打印 (保持原有函数不变)
%=============================================================================

function print_analysis_results(metrics)
%打印详细的分析结果

fprintf('📊 ===== 性能分析结果 =====\n\n');

% 无人机性能指标
fprintf('🚁 无人机位置跟踪性能：\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('1. 绝对位置误差：\n');
fprintf('   最大误差：%.4f m\n', metrics.quad.max_absolute_error);
fprintf('   最终误差：%.4f m\n', metrics.quad.final_absolute_error);
fprintf('\n');

fprintf('2. 径向误差（螺旋轨迹）：\n');
fprintf('   最大径向误差：%.4f m\n', metrics.quad.max_radial_error);
fprintf('\n');

fprintf('3. 均方根误差 (RMSE)：\n');
fprintf('   总体RMSE：%.4f m\n', metrics.quad.rmse_position);
fprintf('   X轴RMSE：%.4f m\n', metrics.quad.rmse_x);
fprintf('   Y轴RMSE：%.4f m\n', metrics.quad.rmse_y);
fprintf('   Z轴RMSE：%.4f m\n', metrics.quad.rmse_z);
fprintf('\n');

fprintf('4. 平均绝对误差 (MAE)：\n');
fprintf('   总体MAE：%.4f m\n', metrics.quad.mae_position);
fprintf('   X轴MAE：%.4f m\n', metrics.quad.mae_x);
fprintf('   Y轴MAE：%.4f m\n', metrics.quad.mae_y);
fprintf('   Z轴MAE：%.4f m\n', metrics.quad.mae_z);
fprintf('\n');

fprintf('5. 稳态误差：\n');
fprintf('   总体稳态误差：%.4f m\n', metrics.quad.steady_state_error);
fprintf('   X轴稳态误差：%.4f m\n', metrics.quad.steady_state_error_x);
fprintf('   Y轴稳态误差：%.4f m\n', metrics.quad.steady_state_error_y);
fprintf('   Z轴稳态误差：%.4f m\n', metrics.quad.steady_state_error_z);
fprintf('\n');

fprintf('6. 超调量：\n');
fprintf('   X轴超调：%.2f%%\n', metrics.quad.overshoot(1));
fprintf('   Y轴超调：%.2f%%\n', metrics.quad.overshoot(2));
fprintf('   Z轴超调：%.2f%%\n', metrics.quad.overshoot(3));
fprintf('   最大超调：%.2f%%\n', metrics.quad.max_overshoot);
fprintf('\n');

fprintf('🔧 姿态跟踪性能：\n');
if ~isnan(metrics.quad.rmse_attitude)
    fprintf('   姿态RMSE：%.4f rad (%.2f°)\n', metrics.quad.rmse_attitude, ...
            metrics.quad.rmse_attitude * 180/pi);
    fprintf('   数据来源：%s\n', metrics.quad.attitude_data_source);
else
    fprintf('   ⚠️  无姿态误差数据可用\n');
end
fprintf('\n');

% 机械臂性能指标
fprintf('🦾 机械臂跟踪性能：\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('1. 绝对误差：\n');
fprintf('   最大误差：%.4f rad (%.2f°)\n', metrics.arm.max_absolute_error, ...
        metrics.arm.max_absolute_error * 180/pi);
fprintf('\n');

fprintf('2. 各关节RMSE：\n');
fprintf('   关节1 RMSE：%.4f rad (%.2f°)\n', metrics.arm.rmse_joint1, ...
        metrics.arm.rmse_joint1 * 180/pi);
fprintf('   关节2 RMSE：%.4f rad (%.2f°)\n', metrics.arm.rmse_joint2, ...
        metrics.arm.rmse_joint2 * 180/pi);
fprintf('   关节3 RMSE：%.4f rad (%.2f°)\n', metrics.arm.rmse_joint3, ...
        metrics.arm.rmse_joint3 * 180/pi);
fprintf('   总体RMSE：%.4f rad (%.2f°)\n', metrics.arm.rmse_total, ...
        metrics.arm.rmse_total * 180/pi);
fprintf('\n');

fprintf('3. 平均绝对误差：\n');
fprintf('   总体MAE：%.4f rad (%.2f°)\n', metrics.arm.mae_total, ...
        metrics.arm.mae_total * 180/pi);
fprintf('\n');

fprintf('4. 稳态误差：\n');
fprintf('   稳态误差：%.4f rad (%.2f°)\n', metrics.arm.steady_state_error, ...
        metrics.arm.steady_state_error * 180/pi);
fprintf('\n');

fprintf('5. 超调量：\n');
fprintf('   关节1超调：%.2f%%\n', metrics.arm.overshoot(1));
fprintf('   关节2超调：%.2f%%\n', metrics.arm.overshoot(2));
fprintf('   关节3超调：%.2f%%\n', metrics.arm.overshoot(3));
fprintf('\n');

% 控制输入指标
fprintf('⚡ 控制输入性能：\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('7. 控制输入能量：\n');
fprintf('   推力能量：%.2f N²⋅s\n', metrics.control.thrust_energy);
fprintf('   力矩能量：%.2f N²⋅m²⋅s\n', metrics.control.torque_energy);
fprintf('   机械臂能量：%.2f N²⋅m²⋅s\n', metrics.control.arm_energy);
fprintf('   总能量：%.2f\n', metrics.control.total_energy);
fprintf('\n');

fprintf('8. 控制输入平滑度：\n');
fprintf('   推力平滑度：%.4e\n', metrics.control.thrust_smoothness);
fprintf('   力矩平滑度：[%.4e, %.4e, %.4e]\n', metrics.control.torque_smoothness);
fprintf('   机械臂平滑度：[%.4e, %.4e, %.4e]\n', metrics.control.arm_smoothness);
fprintf('\n');

fprintf('9. 频域分析指标：\n');
fprintf('   推力频域特性：\n');
fprintf('     - 主频率：%.2f Hz\n', metrics.control.fft_analysis.thrust_metrics.dominant_freq);
fprintf('     - 90%%带宽：%.2f Hz\n', metrics.control.fft_analysis.thrust_metrics.bandwidth_90);
fprintf('     - 高频能量比例：%.2f%%\n', metrics.control.fft_analysis.thrust_metrics.high_freq_ratio*100);
fprintf('     - 频谱重心：%.2f Hz\n', metrics.control.fft_analysis.thrust_metrics.spectral_centroid);
fprintf('   力矩频域平均特性：\n');
fprintf('     - 平均主频率：%.2f Hz\n', mean([metrics.control.fft_analysis.torque_metrics.dominant_freq]));
fprintf('     - 平均高频比例：%.2f%%\n', mean([metrics.control.fft_analysis.torque_metrics.high_freq_ratio])*100);
fprintf('   机械臂频域平均特性：\n');
fprintf('     - 平均主频率：%.2f Hz\n', mean([metrics.control.fft_analysis.arm_metrics.dominant_freq]));
fprintf('     - 平均高频比例：%.2f%%\n', mean([metrics.control.fft_analysis.arm_metrics.high_freq_ratio])*100);
fprintf('\n');

fprintf('📈 控制输入统计：\n');
fprintf('   平均推力：%.2f N (标准差: %.2f)\n', metrics.control.thrust_mean, ...
        metrics.control.thrust_std);
fprintf('   平均力矩：[%.2f, %.2f, %.2f] N⋅m\n', metrics.control.torque_mean);
fprintf('\n');

end

%=============================================================================
% 可视化图表 (保持原有函数不变)
%=============================================================================

function create_visualization_plots(data, metrics)
%创建所有可视化图表

fprintf('🎨 正在生成可视化图表...\n');

% 设置图表通用属性
set(0, 'DefaultLineLineWidth', 1.5);
set(0, 'DefaultAxesFontSize', 12);

% 1. 无人机位置跟踪图（三轴分别显示）
create_quadrotor_tracking_plots(data);

% 2. 无人机三维轨迹图
create_quadrotor_3d_plot(data);

% 3. 无人机位置误差图
create_quadrotor_error_plot(data, metrics);

% 4. 无人机姿态跟踪图
create_quadrotor_attitude_plot(data);

% 5. 机械臂关节跟踪图
create_arm_tracking_plots(data);

% 6. 机械臂误差图
create_arm_error_plot(data, metrics);

% 7. 控制输入频域分析图（新增）
create_frequency_domain_plots(data, metrics);

fprintf('✅ 可视化图表生成完成\n');

end

function create_quadrotor_tracking_plots(data)
%创建无人机三轴位置跟踪图

% X轴跟踪
figure('Name', '无人机X轴位置跟踪', 'Position', [100, 500, 800, 400]);
plot(data.time, data.quad_pos_desired(:, 1), 'b-', 'DisplayName', '目标参数');
hold on;
plot(data.time, data.quad_pos_actual(:, 1), 'r--', 'DisplayName', '测试结果');
xlabel('时间 (s)');
ylabel('X位置 (m)');
legend('Location', 'best');
grid on;

% Y轴跟踪
figure('Name', '无人机Y轴位置跟踪', 'Position', [200, 500, 800, 400]);
plot(data.time, data.quad_pos_desired(:, 2), 'b-', 'DisplayName', '目标参数');
hold on;
plot(data.time, data.quad_pos_actual(:, 2), 'r--', 'DisplayName', '测试结果');
xlabel('时间 (s)');
ylabel('Y位置 (m)');
legend('Location', 'best');
grid on;

% Z轴跟踪
figure('Name', '无人机Z轴位置跟踪', 'Position', [300, 500, 800, 400]);
plot(data.time, data.quad_pos_desired(:, 3), 'b-', 'DisplayName', '目标参数');
hold on;
plot(data.time, data.quad_pos_actual(:, 3), 'r--', 'DisplayName', '测试结果');
xlabel('时间 (s)');
ylabel('Z位置 (m)');
legend('Location', 'best');
grid on;

end

function create_quadrotor_3d_plot(data)
%创建无人机三维轨迹图

figure('Name', '无人机三维轨迹', 'Position', [400, 300, 800, 600]);
plot3(data.quad_pos_desired(:, 1), data.quad_pos_desired(:, 2), data.quad_pos_desired(:, 3), ...
      'b-', 'DisplayName', '目标参数');
hold on;
plot3(data.quad_pos_actual(:, 1), data.quad_pos_actual(:, 2), data.quad_pos_actual(:, 3), ...
      'r--', 'DisplayName', '测试结果');

% 标记起点和终点
plot3(data.quad_pos_desired(1, 1), data.quad_pos_desired(1, 2), data.quad_pos_desired(1, 3), ...
      'go', 'MarkerSize', 8, 'DisplayName', '起点');
plot3(data.quad_pos_desired(end, 1), data.quad_pos_desired(end, 2), data.quad_pos_desired(end, 3), ...
      'rs', 'MarkerSize', 8, 'DisplayName', '终点');

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
legend('Location', 'best');
grid on;
axis equal;
view(45, 30);

end

function create_quadrotor_error_plot(data, metrics)
%创建无人机位置误差图

figure('Name', '无人机位置误差', 'Position', [500, 400, 800, 400]);
plot(data.time, metrics.quad.absolute_error, 'r-', 'DisplayName', '位置误差');
xlabel('时间 (s)');
ylabel('误差 (m)');
legend('Location', 'best');
grid on;

end

function create_quadrotor_attitude_plot(data)
%创建无人机姿态跟踪图

% 只有在有期望姿态数据时才绘制姿态跟踪图
if ~data.has_desired_attitude
    fprintf('⚠️  跳过姿态跟踪图：无期望姿态数据\n');
    return;
end

% Roll角跟踪
figure('Name', '无人机Roll角跟踪', 'Position', [100, 200, 800, 400]);
plot(data.time, data.quad_att_desired(:, 1) , 'b-', 'DisplayName', '目标参数');
hold on;
plot(data.time, data.quad_att_actual(:, 1) , 'r--', 'DisplayName', '测试结果');
xlabel('时间 (s)');
ylabel('Roll角 (rad)');
legend('Location', 'best');
grid on;

% Pitch角跟踪
figure('Name', '无人机Pitch角跟踪', 'Position', [200, 200, 800, 400]);
plot(data.time, data.quad_att_desired(:, 2) , 'b-', 'DisplayName', '目标参数');
hold on;
plot(data.time, data.quad_att_actual(:, 2) , 'r--', 'DisplayName', '测试结果');
xlabel('时间 (s)');
ylabel('Pitch角 (rad)');
legend('Location', 'best');
grid on;

% Yaw角跟踪
figure('Name', '无人机Yaw角跟踪', 'Position', [300, 200, 800, 400]);
plot(data.time, data.quad_att_desired(:, 3) , 'b-', 'DisplayName', '目标参数');
hold on;
plot(data.time, data.quad_att_actual(:, 3) , 'r--', 'DisplayName', '测试结果');
xlabel('时间 (s)');
ylabel('Yaw角 (rad)');
legend('Location', 'best');
grid on;

% 如果有姿态误差数据，绘制姿态误差图
if data.has_attitude_error || data.has_quaternion_error || ~isempty(data.quad_att_desired)
    figure('Name', '无人机姿态误差', 'Position', [400, 200, 800, 400]);
    
    if data.has_attitude_error
        plot(data.time, data.attitude_error , 'r-', 'DisplayName', '姿态误差');
    elseif data.has_quaternion_error
        attitude_error_norm = sqrt(sum(data.quaternion_error.^2, 2));
        plot(data.time, attitude_error_norm , 'r-', 'DisplayName', '四元数姿态误差');
    else
        att_error_vec = data.quad_att_actual - data.quad_att_desired;
        att_error_vec(:, 3) = wrapToPi(att_error_vec(:, 3));
        attitude_error_norm = sqrt(sum(att_error_vec.^2, 2));
        plot(data.time, attitude_error_norm , 'r-', 'DisplayName', '姿态误差');
    end
    
    xlabel('时间 (s)');
    ylabel('误差 (rad)');
    legend('Location', 'best');
    grid on;
end

end

function create_arm_tracking_plots(data)
%创建机械臂关节跟踪图

% 关节1跟踪
figure('Name', '机械臂关节1跟踪', 'Position', [600, 500, 800, 400]);
plot(data.time, data.arm_pos_desired(:, 1) , 'b-', 'DisplayName', '目标参数');
hold on;
plot(data.time, data.arm_pos_actual(:, 1) , 'r--', 'DisplayName', '测试结果');
xlabel('时间 (s)');
ylabel('关节1角度 (rad)');
legend('Location', 'best');
grid on;

% 关节2跟踪
figure('Name', '机械臂关节2跟踪', 'Position', [700, 500, 800, 400]);
plot(data.time, data.arm_pos_desired(:, 2) , 'b-', 'DisplayName', '目标参数');
hold on;
plot(data.time, data.arm_pos_actual(:, 2) , 'r--', 'DisplayName', '测试结果');
xlabel('时间 (s)');
ylabel('关节2角度 (rad)');
legend('Location', 'best');
grid on;

% 关节3跟踪
figure('Name', '机械臂关节3跟踪', 'Position', [800, 500, 800, 400]);
plot(data.time, data.arm_pos_desired(:, 3), 'b-', 'DisplayName', '目标参数');
hold on;
plot(data.time, data.arm_pos_actual(:, 3) , 'r--', 'DisplayName', '测试结果');
xlabel('时间 (s)');
ylabel('关节3角度 (rad)');
legend('Location', 'best');
grid on;

end

function create_arm_error_plot(data, metrics)
%创建机械臂误差图

figure('Name', '机械臂跟踪误差', 'Position', [900, 400, 800, 400]);
plot(data.time, metrics.arm.absolute_error , 'r-', 'DisplayName', '关节角误差');
xlabel('时间 (s)');
ylabel('误差 (rad)');
legend('Location', 'best');
grid on;

end

function angle = wrapToPi(angle)
%将角度限制在[-π, π]范围内
angle = mod(angle + pi, 2*pi) - pi;
end

function create_frequency_domain_plots(data, metrics)
%创建控制输入频域分析图

% 设置频域图表属性
freq = metrics.control.fft_analysis.thrust_freq;

% 1. 推力频域分析
figure('Name', '推力控制频域分析', 'Position', [100, 100, 1000, 600]);

subplot(2,1,1);
plot(data.time, data.control_thrust, 'b-', 'LineWidth', 1);
xlabel('时间 (s)');
ylabel('推力 (N)');
grid on;
title('推力时域信号');

subplot(2,1,2);
semilogy(freq, metrics.control.fft_analysis.thrust_psd, 'r-', 'LineWidth', 1.5);
xlabel('频率 (Hz)');
ylabel('功率谱密度');
grid on;
title('推力频域特性');
xlim([0, min(50, max(freq))]);  % 显示0-50Hz或最大频率

% 标注关键频率点
hold on;
dominant_freq = metrics.control.fft_analysis.thrust_metrics.dominant_freq;
dominant_idx = find(abs(freq - dominant_freq) == min(abs(freq - dominant_freq)), 1);
plot(dominant_freq, metrics.control.fft_analysis.thrust_psd(dominant_idx), 'ro', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'r');
text(dominant_freq, metrics.control.fft_analysis.thrust_psd(dominant_idx)*1.5, ...
     sprintf('主频: %.1fHz', dominant_freq), 'HorizontalAlignment', 'center');

% 2. 力矩频域分析
figure('Name', '力矩控制频域分析', 'Position', [200, 100, 1200, 800]);

torque_labels = {'X轴力矩', 'Y轴力矩', 'Z轴力矩'};
colors = {'r', 'g', 'b'};

for i = 1:3
    % 时域信号
    subplot(3,2,2*i-1);
    plot(data.time, data.control_torque(:,i), colors{i}, 'LineWidth', 1);
    xlabel('时间 (s)');
    ylabel('力矩 (N⋅m)');
    grid on;
    title([torque_labels{i} ' - 时域']);
    
    % 频域信号
    subplot(3,2,2*i);
    semilogy(freq, metrics.control.fft_analysis.torque_psd(:,i), colors{i}, 'LineWidth', 1.5);
    xlabel('频率 (Hz)');
    ylabel('功率谱密度');
    grid on;
    title([torque_labels{i} ' - 频域']);
    xlim([0, min(50, max(freq))]);
    
    % 标注主频率
    hold on;
    dominant_freq = metrics.control.fft_analysis.torque_metrics(i).dominant_freq;
    dominant_idx = find(abs(freq - dominant_freq) == min(abs(freq - dominant_freq)), 1);
    plot(dominant_freq, metrics.control.fft_analysis.torque_psd(dominant_idx,i), ...
         [colors{i} 'o'], 'MarkerSize', 6, 'MarkerFaceColor', colors{i});
end

% 3. 机械臂力矩频域分析
figure('Name', '机械臂力矩频域分析', 'Position', [300, 100, 1200, 800]);

arm_labels = {'关节1力矩', '关节2力矩', '关节3力矩'};

for i = 1:3
    % 时域信号
    subplot(3,2,2*i-1);
    plot(data.time, data.arm_torque(:,i), colors{i}, 'LineWidth', 1);
    xlabel('时间 (s)');
    ylabel('力矩 (N⋅m)');
    grid on;
    title([arm_labels{i} ' - 时域']);
    
    % 频域信号
    subplot(3,2,2*i);
    semilogy(freq, metrics.control.fft_analysis.arm_psd(:,i), colors{i}, 'LineWidth', 1.5);
    xlabel('频率 (Hz)');
    ylabel('功率谱密度');
    grid on;
    title([arm_labels{i} ' - 频域']);
    xlim([0, min(50, max(freq))]);
    
    % 标注主频率
    hold on;
    dominant_freq = metrics.control.fft_analysis.arm_metrics(i).dominant_freq;
    dominant_idx = find(abs(freq - dominant_freq) == min(abs(freq - dominant_freq)), 1);
    plot(dominant_freq, metrics.control.fft_analysis.arm_psd(dominant_idx,i), ...
         [colors{i} 'o'], 'MarkerSize', 6, 'MarkerFaceColor', colors{i});
end

% 4. 综合频域对比图
figure('Name', '控制输入频域综合对比', 'Position', [400, 100, 1000, 600]);

subplot(2,1,1);
% 推力和力矩范数的频域对比
torque_norm_psd = sqrt(sum(metrics.control.fft_analysis.torque_psd.^2, 2));
semilogy(freq, metrics.control.fft_analysis.thrust_psd/max(metrics.control.fft_analysis.thrust_psd), ...
         'b-', 'LineWidth', 2, 'DisplayName', '推力(归一化)');
hold on;
semilogy(freq, torque_norm_psd/max(torque_norm_psd), ...
         'r-', 'LineWidth', 2, 'DisplayName', '力矩范数(归一化)');
xlabel('频率 (Hz)');
ylabel('归一化功率谱密度');
title('四旋翼控制输入频域对比');
legend('Location', 'best');
grid on;
xlim([0, min(50, max(freq))]);

subplot(2,1,2);
% 机械臂各关节频域对比
arm_norm_psd = sqrt(sum(metrics.control.fft_analysis.arm_psd.^2, 2));
semilogy(freq, metrics.control.fft_analysis.arm_psd(:,1)/max(metrics.control.fft_analysis.arm_psd(:,1)), ...
         'r-', 'LineWidth', 1.5, 'DisplayName', '关节1');
hold on;
semilogy(freq, metrics.control.fft_analysis.arm_psd(:,2)/max(metrics.control.fft_analysis.arm_psd(:,2)), ...
         'g-', 'LineWidth', 1.5, 'DisplayName', '关节2');
semilogy(freq, metrics.control.fft_analysis.arm_psd(:,3)/max(metrics.control.fft_analysis.arm_psd(:,3)), ...
         'b-', 'LineWidth', 1.5, 'DisplayName', '关节3');
xlabel('频率 (Hz)');
ylabel('归一化功率谱密度');
title('机械臂控制输入频域对比');
legend('Location', 'best');
grid on;
xlim([0, min(50, max(freq))]);

% 5. 频域离散点图（3D散点图）
figure('Name', '控制输入频域离散点分析', 'Position', [500, 100, 1000, 600]);

% 准备3D散点图数据
max_freq_display = min(25, max(freq));  % 显示0-25Hz
freq_idx = freq <= max_freq_display;
freq_display = freq(freq_idx);

subplot(1,2,1);
% 四旋翼控制频域3D散点
thrust_psd_display = metrics.control.fft_analysis.thrust_psd(freq_idx);
torque_x_psd = metrics.control.fft_analysis.torque_psd(freq_idx,1);
torque_y_psd = metrics.control.fft_analysis.torque_psd(freq_idx,2);

scatter3(freq_display, thrust_psd_display, torque_x_psd, 36, freq_display, 'filled', 'o');
hold on;
scatter3(freq_display, thrust_psd_display, torque_y_psd, 36, freq_display, 'filled', '^');
xlabel('频率 (Hz)');
ylabel('推力PSD');
zlabel('力矩PSD');
title('四旋翼控制频域离散点');
colorbar;
grid on;
legend('X轴力矩', 'Y轴力矩', 'Location', 'best');

subplot(1,2,2);
% 机械臂控制频域3D散点
arm1_psd = metrics.control.fft_analysis.arm_psd(freq_idx,1);
arm2_psd = metrics.control.fft_analysis.arm_psd(freq_idx,2);
arm3_psd = metrics.control.fft_analysis.arm_psd(freq_idx,3);

scatter3(freq_display, arm1_psd, arm2_psd, 36, arm3_psd, 'filled', 'o');
xlabel('频率 (Hz)');
ylabel('关节1 PSD');
zlabel('关节2 PSD');
title('机械臂控制频域离散点');
colorbar;
colormap('jet');
grid on;

end
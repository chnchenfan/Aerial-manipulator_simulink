function plot_mode1_data(varargin)
%% 基于Simulink数据的性能分析工具 - Mode 1 (定高飞行)
%
% 用法：
%   plot_mode1_data()                   - 自动抓取工作区变量分析
%   plot_mode1_data('filename.mat')     - 分析指定的MAT文件
%
% 兼容变量名 (任意一种组合均可):
%   组合A: pd(期望位置), p(实际位置), qd(期望关节), q(实际关节)
%   组合B: quad_pos_desired, quad_pos_actual, arm_pos_desired, arm_pos_actual

clc; close all;
fprintf('=== 飞行机械臂系统性能分析 (Mode 1: 定高飞行) ===\n\n');

%% 1. 解析输入与加载数据
[data, data_source] = load_data_source(varargin);

if isempty(data)
    return; % 数据加载失败，终止运行
end

% 将数据解包为本地变量，方便后续计算
p = data.p;
pd = data.pd;
q = data.q;
qd = data.qd;

% 尝试获取时间向量 (如果数据结构中没有，后续会生成)
if isfield(data, 'time')
    t = data.time;
    has_time = true;
else
    has_time = false;
end

%% 2. 数据预处理 (保持原有逻辑，稍作适配)
% 处理位置数据
[p_data, pd_data, t_pos] = extract_data_values(p, pd);
% 处理关节数据
[q_data, qd_data, ~] = extract_data_values(q, qd);

% 确定时间向量
if has_time
    % 使用加载的时间
elseif ~isempty(t_pos)
    t = t_pos;
else
    % 创建默认时间向量
    dt = 0.01;
    t = (0:size(p_data,1)-1)' * dt;
    fprintf('⚠️  未找到时间向量，使用假设的采样时间%.3fs\n', dt);
end

% 确保数据维度一致 (列向量: 时间 x 通道)
if size(p_data, 1) ~= length(t)
    fprintf('⚠️  数据长度与时间不匹配，尝试转置...\n');
    if size(p_data, 2) == length(t)
        p_data = p_data'; pd_data = pd_data';
        q_data = q_data'; qd_data = qd_data';
    end
end

% 赋值给计算变量
quad_position = p_data(:, 1:3);
desired_position = pd_data(:, 1:3);
if size(q_data, 2) >= 3
    arm_angles = q_data(:, 1:3);
    desired_arm_angles = qd_data(:, 1:3);
else
    arm_angles = q_data;
    desired_arm_angles = qd_data;
end
z_actual = quad_position(:, 3);
z_desired = desired_position(:, 3);

fprintf('✅ 数据预处理完成\n');
fprintf('   数据点数: %d\n', length(t));
fprintf('   时间范围: %.2f s\n', t(end));

%% 3. Mode 1 特性验证
x_variation = std(desired_position(:, 1));
y_variation = std(desired_position(:, 2));
z_final = mean(z_desired(end-min(100, length(z_desired)):end));

fprintf('\n=== Mode 1 特征验证 ===\n');
fprintf('   Z轴目标高度: %.3f m\n', z_final);
if x_variation < 0.1 && y_variation < 0.1 && z_final > 0.1
    fprintf('   ✅ 确认为Mode 1：无人机定高飞行模式\n');
else
    fprintf('   ⚠️  警告：数据特征可能不符合定高飞行模式\n');
end

%% 4. 性能指标计算
fprintf('\n=== 性能指标计算 ===\n');

% 基础误差
z_error = abs(z_actual - z_desired);
position_error_norm = sqrt(sum((quad_position - desired_position).^2, 2));
arm_error = abs(arm_angles - desired_arm_angles);
arm_error_norm = sqrt(sum(arm_error.^2, 2));

% 稳态分析
steady_value = z_final;
final_height = mean(z_actual(end-min(50, length(z_actual)):end));
steady_state_error = abs(final_height - steady_value);

% 上升时间 (10% - 90%)
rise_10 = steady_value * 0.1;
rise_90 = steady_value * 0.9;
idx_10 = find(z_actual >= rise_10, 1);
idx_90 = find(z_actual >= rise_90, 1);
if ~isempty(idx_10) && ~isempty(idx_90)
    rise_time = t(idx_90) - t(idx_10);
else
    rise_time = NaN;
end

% 峰值与超调
[peak_value, peak_idx] = max(z_actual);
peak_time = t(peak_idx);
if peak_value > steady_value
    overshoot_percent = (peak_value - steady_value) / steady_value * 100;
else
    overshoot_percent = 0;
end

% 调节时间 (2%误差带)
tolerance = 0.02 * steady_value;
settling_mask = abs(z_actual - steady_value) <= max(tolerance, 0.01);
last_unsettled = find(~settling_mask, 1, 'last');
if isempty(last_unsettled)
    settling_time = 0;
elseif last_unsettled == length(t)
    settling_time = t(end); % 未稳定
else
    settling_time = t(last_unsettled+1);
end

% 统计指标
mean_error = mean(z_error);
max_error = max(z_error);
rms_error = sqrt(mean(z_error.^2));
q_max_swing = max(max(abs(arm_angles)));
q_rms = sqrt(mean(mean(arm_angles.^2)));

% 打印结果
fprintf('1. 时域指标:\n');
fprintf('   上升时间: %.3f s\n', rise_time);
fprintf('   调节时间: %.3f s\n', settling_time);
fprintf('   超调量:   %.2f%%\n', overshoot_percent);
fprintf('   稳态误差: %.4f m\n', steady_state_error);
fprintf('2. 跟踪精度:\n');
fprintf('   RMS误差:  %.4f m (%.1f mm)\n', rms_error, rms_error*1000);
fprintf('   最大误差: %.4f m\n', max_error);

%% 5. 简单的评分系统 (补全逻辑)
% 定义评分权重和基准
rise_score = max(0, 100 - rise_time * 10); % 假设2秒内上升为满分标准，每慢0.1秒扣1分
overshoot_score = max(0, 100 - overshoot_percent * 2); % 每1%超调扣2分
if settling_time < t(end)
    settling_score = 100 - settling_time * 5; 
else
    settling_score = 40; % 未稳定
end
accuracy_score = max(0, 100 - rms_error * 1000); % 误差每1mm扣1分
stability_score = max(0, 100 - q_rms * 180/pi); % 机械臂摆动每1度扣1分

scores = [rise_score, overshoot_score, settling_score, accuracy_score, stability_score];
avg_score = mean(scores);

%% 6. 可视化绘图
fprintf('\n=== 生成分析图表 ===\n');
set(0, 'DefaultAxesFontSize', 11, 'DefaultLineLineWidth', 1.5);

% 图1: 高度跟踪
figure('Name', '无人机高度响应', 'Color', 'w');
plot(t, z_desired, 'r--', 'LineWidth', 2); hold on;
plot(t, z_actual, 'b-', 'LineWidth', 1.5);
title('高度阶跃响应'); xlabel('时间 (s)'); ylabel('高度 (m)');
legend('期望高度', '实际高度'); grid on;

% 图2: 3D轨迹
figure('Name', '3D 飞行轨迹', 'Color', 'w');
plot3(desired_position(:,1), desired_position(:,2), desired_position(:,3), 'r--'); hold on;
plot3(quad_position(:,1), quad_position(:,2), quad_position(:,3), 'b-');
plot3(quad_position(end,1), quad_position(end,2), quad_position(end,3), 'ro', 'MarkerFaceColor','r');
title('空间飞行轨迹'); xlabel('X'); ylabel('Y'); zlabel('Z');
grid on; view(45, 30); axis equal;

% 图3: 机械臂关节
figure('Name', '机械臂关节角度', 'Color', 'w');
for i=1:3
    subplot(3,1,i);
    plot(t, desired_arm_angles(:,i), 'r--'); hold on;
    plot(t, arm_angles(:,i), 'b-');
    ylabel(['关节 ' num2str(i) ' (rad)']); grid on;
    if i==1, title('机械臂关节跟踪'); legend('期望', '实际'); end
end
xlabel('时间 (s)');

%% 7. 保存结果
if strcmp(data_source, 'mat')
    % 如果是读文件的，保存结果时换个名字以免覆盖原数据
    save_name = ['Result_' datestr(now, 'HHMMSS') '.mat'];
else
    save_name = 'mode1_analysis_results.mat';
end

results.metrics.rise_time = rise_time;
results.metrics.settling_time = settling_time;
results.metrics.overshoot = overshoot_percent;
results.metrics.rms_error = rms_error;
results.score = avg_score;

try
    save(save_name, 'results');
    fprintf('\n💾 结果已保存: %s\n', save_name);
catch
    fprintf('\n⚠️  保存结果失败 (可能是权限问题)\n');
end

fprintf('\n✅ 分析全部完成！综合评分: %.1f\n', avg_score);

end

%% ================= 辅助函数 =================

function [data, source] = load_data_source(args)
    % 智能加载数据 (工作区 或 MAT文件)
    data = [];
    
    % 1. 判断输入参数
    if isempty(args)
        source = 'workspace';
        filename = '';
    else
        source = 'mat';
        filename = args{1};
    end
    
    % 2. 执行加载
    raw_data = [];
    if strcmp(source, 'mat')
        if ~exist(filename, 'file')
            error('❌ 文件不存在: %s', filename);
        end
        fprintf('📂 正在加载MAT文件: %s ...\n', filename);
        raw_data = load(filename);
    else
        fprintf('🔍 正在扫描工作区变量...\n');
        % 尝试从工作区获取所有相关变量
        try
            vars = evalin('base', 'who');
            for i=1:length(vars)
                raw_data.(vars{i}) = evalin('base', vars{i});
            end
        catch
            error('❌ 无法访问工作区变量，请确保数据已加载到工作区');
        end
    end
    
    % 3. 变量映射 (标准化变量名为 p, pd, q, qd)
    data = map_variable_names(raw_data);
    
    % 4. 验证必需变量
    required = {'p', 'pd', 'q', 'qd'};
    missing = {};
    for i=1:length(required)
        if ~isfield(data, required{i})
            missing{end+1} = required{i};
        end
    end
    
    if ~isempty(missing)
        fprintf('❌ 缺少必需变量:\n');
        disp(missing);
        fprintf('支持的变量名对 (实际/期望):\n');
        fprintf('  - p / pd\n');
        fprintf('  - quad_pos_actual / quad_pos_desired\n');
        data = [];
    end
end

function data = map_variable_names(raw)
    % 将不同命名习惯的变量映射为标准名称 p, pd, q, qd
    data = raw;
    
    % 映射表 {标准名, 备选名1, 备选名2}
    maps = {
        'p',  'quad_pos_actual', 'pos_actual';
        'pd', 'quad_pos_desired', 'pos_desired';
        'q',  'arm_pos_actual',  'q_actual';
        'qd', 'arm_pos_desired', 'q_desired';
        'time', 'time_series', 'tout'
    };
    
    for i = 1:size(maps, 1)
        std_name = maps{i, 1};
        % 如果标准名已经存在，跳过
        if isfield(data, std_name), continue; end
        
        % 检查备选名
        for j = 2:size(maps, 2)
            alt_name = maps{i, j};
            if isfield(raw, alt_name)
                data.(std_name) = raw.(alt_name);
                fprintf('   🔗 映射变量: %s -> %s\n', alt_name, std_name);
                break;
            end
        end
    end
end

function [val, val_d, t] = extract_data_values(raw_act, raw_des)
    % 从timeseries或struct中提取数值
    t = [];
    
    % 处理实际值
    if isa(raw_act, 'timeseries')
        val = raw_act.Data;
        t = raw_act.Time;
    elseif isstruct(raw_act) && isfield(raw_act, 'signals')
        val = raw_act.signals.values;
        if isfield(raw_act, 'time'), t = raw_act.time; end
    else
        val = raw_act;
    end
    
    % 处理期望值
    if isa(raw_des, 'timeseries')
        val_d = raw_des.Data;
    elseif isstruct(raw_des) && isfield(raw_des, 'signals')
        val_d = raw_des.signals.values;
    else
        val_d = raw_des;
    end
end
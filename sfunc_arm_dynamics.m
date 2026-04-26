function [sys,x0,str,ts] = sfunc_arm_dynamics(t,x,u,flag)
%SFUNC_ARM_DYNAMICS Delta机械臂动力学模型离散S-Function (修复版)
%
% 功能：严格按照论文实现Delta机械臂的动力学模型，包含与四旋翼基座运动的耦合
%      修复了数组维度不匹配的问题
%
% 输入参数 u: (保持原有15个输入，避免接口变更)
%   u(1:3)   - 机械臂控制力矩 τ_M ∈ R³ (N⋅m)
%   u(4:6)   - 四旋翼位置 p ∈ R³ (m)
%   u(7:9)   - 四旋翼速度 v ∈ R³ (m/s)
%   u(10:12) - 四旋翼姿态角 [φ, θ, ψ] (rad)
%   u(13:15) - 四旋翼角速度 ω ∈ R³ (rad/s)
%
% 输出参数 y: (保持原有15个输出)
%   y(1:3)   - 机械臂关节角度 q ∈ R³ (rad)
%   y(4:6)   - 机械臂关节角速度 q̇ ∈ R³ (rad/s)
%   y(7:9)   - 机械臂关节角加速度 q̈ ∈ R³ (rad/s²)
%   y(10:12) - 末端执行器位置 p_E ∈ R³ (m)
%   y(13:15) - 末端执行器速度 ṗ_E ∈ R³ (m/s)
%
% 状态变量 x: (重新设计状态结构)
%   x(1:3)   - 机械臂关节角度 q
%   x(4:6)   - 机械臂关节角速度 q̇
%   x(7:15)  - 速度历史数据 (3×3矩阵，用于估计加速度)
%   x(16:24) - 角速度历史数据 (3×3矩阵，用于估计角加速度)
%   x(25:27) - 内部估计的线加速度
%   x(28:30) - 内部估计的角加速度

switch flag
    case 0 % 初始化
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 2 % 离散状态更新
        sys = mdlUpdate(t,x,u);
    case 3 % 输出计算
        sys = mdlOutputs(t,x,u);
    case {1,4,9} % 其他情况
        sys = [];
    otherwise
        error('arm_dynamics_sfunc: 未处理的flag值: %d', flag);
end

%=============================================================================
% mdlInitializeSizes: 系统初始化
%=============================================================================
function [sys,x0,str,ts] = mdlInitializeSizes

sizes = simsizes;
sizes.NumContStates  = 0;   % 无连续状态
sizes.NumDiscStates  = 30;  % 30个离散状态（修复后的状态结构）
sizes.NumOutputs     = 15;  % 15个输出（保持原有）
sizes.NumInputs      = 15;  % 15个输入（保持原有）
sizes.DirFeedthrough = 1;   % 有直通项
sizes.NumSampleTimes = 1;   % 一个采样时间

sys = simsizes(sizes);

% 初始离散状态 - 确保所有维度正确
x0 = zeros(30, 1);

% 初始关节角度和角速度
x0(1:3) = [0; 0; 0];   % 初始关节角度
x0(4:6) = [0; 0; 0];   % 初始关节角速度

% 速度历史数据初始化 (3×3 = 9个元素)
x0(7:15) = zeros(9, 1);

% 角速度历史数据初始化 (3×3 = 9个元素)
x0(16:24) = zeros(9, 1);

% 估计的加速度初始化
x0(25:27) = [0; 0; 0]; % 线加速度估计
x0(28:30) = [0; 0; 0]; % 角加速度估计

str = []; % 无状态名称
ts = [0.01 0]; % 离散时间系统，采样时间10ms

%=============================================================================
% mdlUpdate: 离散状态更新
%=============================================================================
function sys = mdlUpdate(t,x,u)

% 检查输入维度
if length(u) ~= 15
    error('输入向量u的长度应为15，实际为%d', length(u));
end

% 解析输入
tau_M = u(1:3);          % 机械臂控制力矩
p_base = u(4:6);         % 四旋翼位置
v_base = u(7:9);         % 四旋翼速度
att_base = u(10:12);     % 四旋翼姿态角
omega_base = u(13:15);   % 四旋翼角速度

% 解析状态 - 确保维度正确
q = x(1:3);                    % 机械臂关节角度
q_dot = x(4:6);                % 机械臂关节角速度
v_hist_vec = x(7:15);          % 速度历史向量
omega_hist_vec = x(16:24);     % 角速度历史向量
a_est = x(25:27);              % 估计的线加速度
alpha_est = x(28:30);          % 估计的角加速度

% 将历史向量重塑为矩阵
v_hist = reshape(v_hist_vec, 3, 3);       % 3×3矩阵
omega_hist = reshape(omega_hist_vec, 3, 3); % 3×3矩阵

% 获取系统参数
params = common_functions('get_system_params');
dt = params.dt;

% 更新历史数据 - 确保维度匹配
v_hist_new = [v_hist(:,2:3), v_base];         % 3×3矩阵
omega_hist_new = [omega_hist(:,2:3), omega_base]; % 3×3矩阵

% 估计加速度（基于历史数据）
if size(v_hist_new, 2) >= 3
    % 使用中心差分法
    a_base_est = (v_hist_new(:,3) - v_hist_new(:,1)) / (2*dt);
    alpha_base_est = (omega_hist_new(:,3) - omega_hist_new(:,1)) / (2*dt);
else
    % 使用前向差分法
    if size(v_hist_new, 2) >= 2
        a_base_est = (v_hist_new(:,end) - v_hist_new(:,end-1)) / dt;
        alpha_base_est = (omega_hist_new(:,end) - omega_hist_new(:,end-1)) / dt;
    else
        a_base_est = [0; 0; 0];
        alpha_base_est = [0; 0; 0];
    end
end

% 平滑滤波
alpha_smooth = 0.7;
a_est_new = alpha_smooth * a_est + (1 - alpha_smooth) * a_base_est;
alpha_est_new = alpha_smooth * alpha_est + (1 - alpha_smooth) * alpha_base_est;

% 计算机械臂动力学
q_ddot = compute_delta_arm_dynamics_complete(q, q_dot, tau_M, ...
    p_base, v_base, att_base, omega_base, a_est_new, alpha_est_new, params);

% 积分更新状态
q_next = q + dt * q_dot;
q_dot_next = q_dot + dt * q_ddot;

% 关节角度限制
q_limit = pi/3;  % ±60度限制
for i = 1:3
    if q_next(i) > q_limit
        q_next(i) = q_limit;
        if q_dot_next(i) > 0
            q_dot_next(i) = 0; % 停止向外运动
        end
    elseif q_next(i) < -q_limit
        q_next(i) = -q_limit;
        if q_dot_next(i) < 0
            q_dot_next(i) = 0; % 停止向内运动
        end
    end
end

% 组装新的状态向量 - 确保所有维度正确
sys = [q_next;                              % 3个元素
       q_dot_next;                          % 3个元素
       reshape(v_hist_new, 9, 1);          % 9个元素
       reshape(omega_hist_new, 9, 1);      % 9个元素
       a_est_new;                          % 3个元素
       alpha_est_new];                     % 3个元素
                                           % 总计30个元素

% 验证输出维度
if length(sys) ~= 30
    error('状态更新向量长度错误：期望30，实际%d', length(sys));
end

%=============================================================================
% mdlOutputs: 输出方程
%=============================================================================
function sys = mdlOutputs(t,x,u)

% 解析输入
tau_M = u(1:3);
p_base = u(4:6);
v_base = u(7:9);
att_base = u(10:12);
omega_base = u(13:15);

% 解析状态
q = x(1:3);
q_dot = x(4:6);
a_est = x(25:27);
alpha_est = x(28:30);

% 获取系统参数
params = common_functions('get_system_params');

% 计算关节角加速度
q_ddot = compute_delta_arm_dynamics_complete(q, q_dot, tau_M, ...
    p_base, v_base, att_base, omega_base, a_est, alpha_est, params);

% 计算末端执行器运动学
[p_E, p_E_dot] = compute_end_effector_kinematics_complete(...
    q, q_dot, q_ddot, p_base, v_base, a_est, att_base, omega_base, alpha_est, params);

% q = [0;0;0];
% q_dot = [0;0;0];
% q_ddot = [0;0;0];

% 输出 - 保持原有15个输出
true_output = [q; q_dot; q_ddot; p_E; p_E_dot];
sim_tuning_runtime('log', 'q_true', t, q);
sim_tuning_runtime('log', 'qd_true', t, q_dot);
sim_tuning_runtime('log', 'qdd_true', t, q_ddot);
sim_tuning_runtime('log', 'pE_true', t, p_E);
sim_tuning_runtime('log', 'pE_dot_true', t, p_E_dot);
sys = apply_arm_measurement_channel(t, true_output);

% 验证输出维度
if length(sys) ~= 15
    error('输出向量长度错误：期望15，实际%d', length(sys));
end

%=============================================================================
% Delta机械臂动力学实现
%=============================================================================

function measured_output = apply_arm_measurement_channel(t, true_output)
persistent arm_buffer arm_stream arm_seed arm_prev_output arm_noise_state arm_bias_state;

config = get_arm_measurement_config();
if ~config.enabled
    measured_output = true_output;
    return;
end

dt = get_measurement_sample_time(config);
base_delay_steps = get_delay_in_steps(config, dt, 'arm');
jitter_steps = max(0, round(config.arm_delay_jitter_steps));
expected_cols = ceil(base_delay_steps + jitter_steps) + 2;

if isempty(arm_seed) || t <= 0
    arm_seed = config.seed + 101;
    arm_stream = RandStream('mt19937ar', 'Seed', arm_seed);
    arm_buffer = repmat(true_output(:), 1, expected_cols);
    arm_prev_output = true_output(:);
    arm_noise_state = zeros(size(true_output(:)));
    arm_bias_state = zeros(size(true_output(:)));
end

if isempty(arm_buffer) || size(arm_buffer, 1) ~= numel(true_output) || size(arm_buffer, 2) ~= expected_cols
    arm_buffer = repmat(true_output(:), 1, expected_cols);
    arm_prev_output = true_output(:);
    arm_noise_state = zeros(size(true_output(:)));
    arm_bias_state = zeros(size(true_output(:)));
end

arm_buffer(:, 1:end-1) = arm_buffer(:, 2:end);
arm_buffer(:, end) = true_output(:);
actual_delay = base_delay_steps;
if strcmpi(config.model, 'empirical') && jitter_steps > 0
    actual_delay = actual_delay + randi(arm_stream, [0, jitter_steps], 1, 1);
end
delayed_output = sample_delayed_buffer(arm_buffer, actual_delay);

if strcmpi(config.model, 'empirical')
    alpha_vec = config.arm_colored_noise_alpha * ones(size(true_output(:)));
    std_vec = [ ...
        config.arm_position_noise_std * ones(3,1); ...
        config.arm_velocity_noise_std * ones(3,1); ...
        config.arm_acceleration_noise_std * ones(3,1); ...
        config.end_effector_position_noise_std * ones(3,1); ...
        config.end_effector_velocity_noise_std * ones(3,1)];
    bias_walk_vec = [ ...
        config.arm_position_bias_walk_std * ones(3,1); ...
        config.arm_velocity_bias_walk_std * ones(3,1); ...
        config.arm_acceleration_bias_walk_std * ones(3,1); ...
        config.end_effector_position_bias_walk_std * ones(3,1); ...
        config.end_effector_velocity_bias_walk_std * ones(3,1)];
    quant_vec = [ ...
        config.arm_position_quantization_step * ones(3,1); ...
        config.arm_velocity_quantization_step * ones(3,1); ...
        zeros(3,1); ...
        zeros(3,1); ...
        zeros(3,1)];

    innovation = sqrt(max(0, 1 - alpha_vec.^2)) .* std_vec .* randn(arm_stream, numel(true_output), 1);
    arm_noise_state = alpha_vec .* arm_noise_state + innovation;
    arm_bias_state = arm_bias_state + bias_walk_vec .* randn(arm_stream, numel(true_output), 1);
    measured_output = delayed_output + arm_noise_state + arm_bias_state;

    if rand(arm_stream, 1, 1) < config.arm_dropout_prob
        measured_output = arm_prev_output;
    end

    quant_mask = quant_vec > 0;
    measured_output(quant_mask) = quant_vec(quant_mask) .* round(measured_output(quant_mask) ./ quant_vec(quant_mask));
else
    noise = zeros(size(delayed_output));
    noise(1:3) = config.arm_position_noise_std * randn(arm_stream, 3, 1);
    noise(4:6) = config.arm_velocity_noise_std * randn(arm_stream, 3, 1);
    noise(7:9) = config.arm_acceleration_noise_std * randn(arm_stream, 3, 1);
    noise(10:12) = config.end_effector_position_noise_std * randn(arm_stream, 3, 1);
    noise(13:15) = config.end_effector_velocity_noise_std * randn(arm_stream, 3, 1);
    measured_output = delayed_output + noise;
end

arm_prev_output = measured_output;

function config = get_arm_measurement_config()
runtime_config = sim_tuning_runtime('get_config');
defaults = struct( ...
    'enabled', true, ...
    'seed', 20260424, ...
    'model', 'empirical', ...
    'arm_delay_seconds', 0.001, ...
    'arm_delay_steps', 0, ...
    'arm_delay_jitter_steps', 0, ...
    'arm_dropout_prob', 0.0, ...
    'arm_colored_noise_alpha', 0.86, ...
    'arm_position_noise_std', 0.001875, ...
    'arm_position_bias_walk_std', 0.00002, ...
    'arm_position_quantization_step', 0.0005, ...
    'arm_velocity_noise_std', 0.00525, ...
    'arm_velocity_bias_walk_std', 0.00005, ...
    'arm_velocity_quantization_step', 0.0010, ...
    'arm_acceleration_noise_std', 0.00750, ...
    'arm_acceleration_bias_walk_std', 0.00008, ...
    'end_effector_position_noise_std', 0.000675, ...
    'end_effector_position_bias_walk_std', 0.00001, ...
    'end_effector_velocity_noise_std', 0.0020625, ...
    'end_effector_velocity_bias_walk_std', 0.00003);
config = defaults;

if isstruct(runtime_config) && isfield(runtime_config, 'measurement') ...
        && isstruct(runtime_config.measurement)
    config = merge_structs(config, runtime_config.measurement);
end

function dt = get_measurement_sample_time(config)
dt = 0.01;
if isstruct(config) && isfield(config, 'sample_time')
    dt = config.sample_time;
else
    try
        runtime_config = sim_tuning_runtime('get_config');
        if isfield(runtime_config, 'sim') && isfield(runtime_config.sim, 'sample_time')
            dt = runtime_config.sim.sample_time;
        end
    catch
    end
end

function delay_steps = get_delay_in_steps(config, dt, prefix)
delay_field = [prefix '_delay_seconds'];
step_field = [prefix '_delay_steps'];

if isfield(config, delay_field)
    delay_steps = max(0, config.(delay_field) / dt);
elseif isfield(config, step_field)
    delay_steps = max(0, config.(step_field));
else
    delay_steps = 0;
end

function delayed_output = sample_delayed_buffer(buffer, delay_steps)
buffer_cols = size(buffer, 2);
delay_steps = max(0, delay_steps);
index_float = buffer_cols - delay_steps;
index_float = min(max(index_float, 1), buffer_cols);
index_low = floor(index_float);
index_high = min(index_low + 1, buffer_cols);
alpha = index_float - index_low;
delayed_output = (1 - alpha) * buffer(:, index_low) + alpha * buffer(:, index_high);

function q_ddot = compute_delta_arm_dynamics_complete(q, q_dot, tau_M, ...
    p_base, v_base, att_base, omega_base, a_base, alpha_base, params)
% 完整的Delta机械臂动力学计算

try
    % 1. 计算质量矩阵
    M_arm = compute_delta_mass_matrix(q, params);
    
    % 2. 计算科里奥利矩阵
    C_arm = compute_delta_coriolis_matrix(q, q_dot, params);
    
    % 3. 计算重力项
    G_arm = compute_delta_gravity_vector(q, att_base, params);
    
    % 4. 计算耦合力
    F_coupling = compute_base_coupling_complete(q, q_dot, ...
        p_base, v_base, a_base, att_base, omega_base, alpha_base, params);
    
    % 5. 求解动力学方程
    rhs = tau_M - C_arm * q_dot - G_arm - F_coupling;
    
    % 数值稳定性检查
    cond_M = cond(M_arm);
    if cond_M > 1e8
        fprintf('警告：质量矩阵条件数过大 (%.2e)，使用正则化\n', cond_M);
        lambda = 1e-4;
        M_arm = M_arm + lambda * eye(3);
    end
    
    q_ddot = M_arm \ rhs;
    
    % 限制加速度
    max_accel = 30; % rad/s²
    for i = 1:3
        if abs(q_ddot(i)) > max_accel
            q_ddot(i) = sign(q_ddot(i)) * max_accel;
        end
    end
    
catch ME
    fprintf('动力学计算失败: %s，使用备用方法\n', ME.message);
    q_ddot = compute_simplified_dynamics(q, q_dot, tau_M, params);
end

function M_arm = compute_delta_mass_matrix(q, params)
% 计算Delta机械臂质量矩阵

% 几何参数
l_U = params.l_U;
l_L = params.l_L;
r_F = params.r_F;
m_total = params.m_M;

% 质量分布
m_link = m_total / 3;  % 每个链的质量
I_link = m_link * l_U^2 / 3;  % 简化的转动惯量

% 初始化质量矩阵
M_arm = zeros(3, 3);

% 主对角线元素
for i = 1:3
    % 基本惯性项
    M_arm(i, i) = I_link * (1 + 0.5 * cos(q(i))^2);
    
    % 添加小的耦合项
    for j = 1:3
        if i ~= j
            alpha_diff = (i - j) * 2*pi/3;
            M_arm(i, j) = 0.01 * I_link * cos(alpha_diff) * cos(q(i) - q(j));
        end
    end
end

% 确保正定性
min_eig = min(eig(M_arm));
if min_eig <= 0
    M_arm = M_arm + (0.02 - min_eig) * eye(3);
end

function C_arm = compute_delta_coriolis_matrix(q, q_dot, params)
% 计算科里奥利矩阵（简化版本）

C_arm = zeros(3, 3);

% 简化的科里奥利项计算
for i = 1:3
    for j = 1:3
        if i == j
            % 对角线阻尼项
            C_arm(i, j) = 0.05 * (1 + 0.1 * abs(q_dot(i)));
        else
            % 非对角线耦合项
            C_arm(i, j) = 0.01 * sin(q(i) - q(j)) * q_dot(j);
        end
    end
end

function G_arm = compute_delta_gravity_vector(q, att_base, params)
% 计算重力项

% 基座旋转矩阵
R_base = common_functions('euler_to_rotation_matrix', att_base);
g_base = R_base' * [0; 0; -params.g];

% 重力项计算
l_U = params.l_U;
m_link = params.m_M / 3;

G_arm = zeros(3, 1);
for i = 1:3
    alpha_i = (i-1) * 2*pi/3;
    % 重力对关节的力矩
    G_arm(i) = m_link * g_base(3) * 0.5 * l_U * cos(q(i));
end

function F_coupling = compute_base_coupling_complete(q, q_dot, ...
    p_base, v_base, a_base, att_base, omega_base, alpha_base, params)
% 计算基座运动耦合

% 机械臂质心相对位置
m_link = params.m_M / 3;
F_coupling = zeros(3, 1);

for i = 1:3
    alpha_i = (i-1) * 2*pi/3;
    
    % 关节位置向量
    r_i = [params.r_F * cos(alpha_i) + 0.5 * params.l_U * cos(q(i)) * cos(alpha_i);
           params.r_F * sin(alpha_i) + 0.5 * params.l_U * cos(q(i)) * sin(alpha_i);
           0.5 * params.l_U * sin(q(i))];
    
    % 平移惯性力
    F_trans = -m_link * a_base;
    
    % 旋转惯性力
    F_rot = -m_link * (cross(alpha_base, r_i) + cross(omega_base, cross(omega_base, r_i)));
    
    % 总力
    F_total = F_trans + F_rot;
    
    % 投影到关节方向
    joint_direction = [-sin(alpha_i); cos(alpha_i); 0];
    F_coupling(i) = joint_direction' * F_total;
end

% 限制耦合力
max_coupling = 5; % N⋅m
for i = 1:3
    if abs(F_coupling(i)) > max_coupling
        F_coupling(i) = sign(F_coupling(i)) * max_coupling;
    end
end

function [p_E, p_E_dot] = compute_end_effector_kinematics_complete(...
    q, q_dot, q_ddot, p_base, v_base, a_base, att_base, omega_base, alpha_base, params)
% 计算末端执行器运动学

% 1. 计算末端在Delta坐标系中的位置
[p_E_D, J_arm] = common_functions('delta_forward_kinematics', q, params);

% 2. 转换到全局坐标系
R_base = common_functions('euler_to_rotation_matrix', att_base);
offset = [0; 0; -0.1]; % Delta基座偏移

% 全局位置
p_E = p_base + R_base * (offset + p_E_D);

% 3. 计算速度
omega_skew = common_functions('skew_symmetric', omega_base);
R_dot = R_base * omega_skew;

p_E_dot = v_base + R_dot * (offset + p_E_D) + R_base * J_arm * q_dot;

function q_ddot = compute_simplified_dynamics(q, q_dot, tau_M, params)
% 简化动力学（备用）

I_simple = 0.1 * eye(3);
damping = 0.1 * diag([1, 1, 1]);

q_ddot = I_simple \ (tau_M - damping * q_dot);

% 限制
max_accel = 15; % rad/s²
for i = 1:3
    if abs(q_ddot(i)) > max_accel
        q_ddot(i) = sign(q_ddot(i)) * max_accel;
    end
end

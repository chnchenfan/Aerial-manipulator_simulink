function [sys,x0,str,ts] = sfunc_quadrotor_dynamics(t,x,u,flag)
%SFUNC_QUADROTOR_DYNAMICS 四旋翼动力学模型离散S-Function
%
% 功能：实现论文公式(6)的四旋翼动力学，包含与机械臂的动态耦合
%
% 输入参数 u:
%   u(1)     - 控制推力 f (N)
%   u(2:4)   - 控制力矩 τ ∈ R³ (N⋅m)
%   u(5:7)   - 机械臂关节角度 q ∈ R³ (rad)
%   u(8:10)  - 机械臂关节角速度 q̇ ∈ R³ (rad/s)
%   u(11:13) - 机械臂关节角加速度 q̈ ∈ R³ (rad/s²)
%   u(14)    - 负载质量 (kg)
%   u(15:17) - 实际加速度 (kg)
%   u(18:20) - 实际角加速度 (kg)

% 输出参数 y:
%   y(1:3)   - 四旋翼位置 p ∈ R³ (m)
%   y(4:6)   - 四旋翼速度 v ∈ R³ (m/s)
%   y(7:9)   - 四旋翼姿态角 [φ, θ, ψ] (rad)
%   y(10:12) - 四旋翼角速度 ω ∈ R³ (rad/s)
%   y(13:15) - 扰动h_v 
%
% 状态变量 x:
%   x(1:3)   - 四旋翼位置 p
%   x(4:6)   - 四旋翼速度 v
%   x(7:9)   - 四旋翼姿态角 [φ, θ, ψ]
%   x(10:12) - 四旋翼角速度 ω


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
        error('quadrotor_dynamics_sfunc: 未处理的flag值: %d', flag);
end

%=============================================================================
% mdlInitializeSizes: 系统初始化
%=============================================================================
function [sys,x0,str,ts] = mdlInitializeSizes

sizes = simsizes;
sizes.NumContStates  = 0;   % 无连续状态
sizes.NumDiscStates  = 12;  % 12个离散状态
sizes.NumOutputs     = 15;  % 15个输出
sizes.NumInputs      = 20;  % 20个输入
sizes.DirFeedthrough = 1;   % 有直通项
sizes.NumSampleTimes = 1;   % 一个采样时间

sys = simsizes(sizes);

% 初始离散状态
x0 = zeros(12, 1);
x0(3) = 5;  % 初始高度0m

str = []; % 无状态名称
ts = [0.01 0]; % 离散时间系统，采样时间10ms

%=============================================================================
% mdlUpdate: 离散状态更新
%=============================================================================
function sys = mdlUpdate(t,x,u)

% 解析输入
f_control = u(1);        % 控制推力
tau_control = u(2:4);    % 控制力矩
q = u(5:7);              % 机械臂关节角度
q_dot = u(8:10);         % 机械臂关节角速度
q_ddot = u(11:13);       % 机械臂关节角加速度
payload_mass = u(14);    % 负载质量
a = u(15:17);            % 四旋翼加速度
w_dot = u(18:20);        % 四旋翼角加速度

% 解析状态
p = x(1:3);              % 四旋翼位置
v = x(4:6);              % 四旋翼速度
att = x(7:9);            % 四旋翼姿态角
omega = x(10:12);        % 四旋翼角速度

% 获取系统参数
params = common_functions('get_system_params');

% 计算四旋翼动力学 (论文公式(6))
[p_dot, v_dot, att_dot, omega_dot, h_v] = compute_quadrotor_dynamics_improved(...
    p, v, att, omega, q, q_dot, q_ddot, f_control, tau_control, params, ...
    payload_mass, w_dot, a);

% 离散化积分 (前向欧拉方法)
dt = params.dt;
p_next = p + dt * p_dot;
v_next = v + dt * v_dot;
att_next = att + dt * att_dot;
omega_next = omega + dt * omega_dot;

% 姿态角限制 (防止数值溢出)
att_next(1) = wrapToPi(att_next(1)); % roll
att_next(2) = wrapToPi(att_next(2)); % pitch
att_next(3) = wrapToPi(att_next(3)); % yaw

% 更新离散状态
sys = [p_next; v_next; att_next; omega_next];

%=============================================================================
% mdlOutputs: 输出方程
%=============================================================================
function sys = mdlOutputs(t,x,u)
% 解析输入
f_control = u(1);        % 控制推力
tau_control = u(2:4);    % 控制力矩
q = u(5:7);              % 机械臂关节角度
q_dot = u(8:10);         % 机械臂关节角速度
q_ddot = u(11:13);       % 机械臂关节角加速度
payload_mass = u(14);    % 负载质量
a = u(15:17);            % 四旋翼加速度
w_dot = u(18:20);        % 四旋翼角加速度

% 解析状态
p = x(1:3);              % 四旋翼位置
v = x(4:6);              % 四旋翼速度
att = x(7:9);            % 四旋翼姿态角
omega = x(10:12);        % 四旋翼角速度

% 获取系统参数
params = common_functions('get_system_params');

% 计算四旋翼动力学 (论文公式(6))
[~, ~, ~, ~, h_v] = compute_quadrotor_dynamics_improved(...
    p, v, att, omega, q, q_dot, q_ddot, f_control, tau_control, params, ...
    payload_mass, w_dot, a);
% 直接输出状态的前12个元素
true_output = [x(1:12); h_v];
sim_tuning_runtime('log', 'p_true', t, x(1:3));
sim_tuning_runtime('log', 'v_true', t, x(4:6));
sim_tuning_runtime('log', 'att_true', t, x(7:9));
sim_tuning_runtime('log', 'omega_true', t, x(10:12));
sim_tuning_runtime('log', 'h_v_true', t, h_v);
sys = apply_quad_measurement_channel(t, true_output);
% fprintf('输出长度: %d, 内容: %s\n', length(sys), mat2str(sys));
% fprintf('数据类型: %s, 是否实数: %d\n', class(sys), isreal(sys));

%=============================================================================
% 四旋翼动力学实现
%=============================================================================

function measured_output = apply_quad_measurement_channel(t, true_output)
persistent quad_buffer quad_stream quad_seed quad_prev_output quad_noise_state quad_bias_state;

config = get_quad_measurement_config();
if ~config.enabled
    measured_output = true_output;
    return;
end

dt = get_measurement_sample_time(config);
base_delay_steps = get_delay_in_steps(config, dt, 'quad');
jitter_steps = max(0, round(config.quad_delay_jitter_steps));
expected_cols = ceil(base_delay_steps + jitter_steps) + 2;

if isempty(quad_seed) || t <= 0
    quad_seed = config.seed;
    quad_stream = RandStream('mt19937ar', 'Seed', quad_seed);
    quad_buffer = repmat(true_output(:), 1, expected_cols);
    quad_prev_output = true_output(:);
    quad_noise_state = zeros(size(true_output(:)));
    quad_bias_state = zeros(size(true_output(:)));
end

if isempty(quad_buffer) || size(quad_buffer, 1) ~= numel(true_output) || size(quad_buffer, 2) ~= expected_cols
    quad_buffer = repmat(true_output(:), 1, expected_cols);
    quad_prev_output = true_output(:);
    quad_noise_state = zeros(size(true_output(:)));
    quad_bias_state = zeros(size(true_output(:)));
end

quad_buffer(:, 1:end-1) = quad_buffer(:, 2:end);
quad_buffer(:, end) = true_output(:);
actual_delay = base_delay_steps;
if strcmpi(config.model, 'empirical') && jitter_steps > 0
    actual_delay = actual_delay + randi(quad_stream, [0, jitter_steps], 1, 1);
end
delayed_output = sample_delayed_buffer(quad_buffer, actual_delay);

if strcmpi(config.model, 'empirical')
    alpha_vec = config.quad_colored_noise_alpha * ones(size(true_output(:)));
    std_vec = [ ...
        config.position_noise_std * ones(3,1); ...
        config.velocity_noise_std * ones(3,1); ...
        config.attitude_noise_std * ones(3,1); ...
        config.omega_noise_std * ones(3,1); ...
        config.hv_noise_std * ones(3,1)];
    bias_walk_vec = [ ...
        config.position_bias_walk_std * ones(3,1); ...
        config.velocity_bias_walk_std * ones(3,1); ...
        config.attitude_bias_walk_std * ones(3,1); ...
        config.omega_bias_walk_std * ones(3,1); ...
        config.hv_bias_walk_std * ones(3,1)];
    quant_vec = [ ...
        config.position_quantization_step * ones(3,1); ...
        config.velocity_quantization_step * ones(3,1); ...
        config.attitude_quantization_step * ones(3,1); ...
        config.omega_quantization_step * ones(3,1); ...
        zeros(3,1)];

    innovation = sqrt(max(0, 1 - alpha_vec.^2)) .* std_vec .* randn(quad_stream, numel(true_output), 1);
    quad_noise_state = alpha_vec .* quad_noise_state + innovation;
    quad_bias_state = quad_bias_state + bias_walk_vec .* randn(quad_stream, numel(true_output), 1);
    measured_output = delayed_output + quad_noise_state + quad_bias_state;

    if rand(quad_stream, 1, 1) < config.quad_dropout_prob
        measured_output = quad_prev_output;
    end

    quant_mask = quant_vec > 0;
    measured_output(quant_mask) = quant_vec(quant_mask) .* round(measured_output(quant_mask) ./ quant_vec(quant_mask));
else
    noise = zeros(size(delayed_output));
    noise(1:3) = config.position_noise_std * randn(quad_stream, 3, 1);
    noise(4:6) = config.velocity_noise_std * randn(quad_stream, 3, 1);
    noise(7:9) = config.attitude_noise_std * randn(quad_stream, 3, 1);
    noise(10:12) = config.omega_noise_std * randn(quad_stream, 3, 1);
    noise(13:15) = config.hv_noise_std * randn(quad_stream, 3, 1);
    measured_output = delayed_output + noise;
end

measured_output(7:9) = wrap_measurement_angles(measured_output(7:9));
quad_prev_output = measured_output;

function config = get_quad_measurement_config()
runtime_config = sim_tuning_runtime('get_config');
defaults = struct( ...
    'enabled', true, ...
    'seed', 20260424, ...
    'model', 'empirical', ...
    'quad_delay_seconds', 0.010, ...
    'quad_delay_steps', 1, ...
    'quad_delay_jitter_steps', 0, ...
    'quad_dropout_prob', 0.0, ...
    'quad_colored_noise_alpha', 0.88, ...
    'position_noise_std', 0.000825, ...
    'position_bias_walk_std', 0.00001, ...
    'position_quantization_step', 0.0005, ...
    'velocity_noise_std', 0.001875, ...
    'velocity_bias_walk_std', 0.00003, ...
    'velocity_quantization_step', 0.0010, ...
    'attitude_noise_std', 0.000375, ...
    'attitude_bias_walk_std', 0.00001, ...
    'attitude_quantization_step', 0.0002, ...
    'omega_noise_std', 0.0013125, ...
    'omega_bias_walk_std', 0.00002, ...
    'omega_quantization_step', 0.0005, ...
    'hv_noise_std', 0.001875, ...
    'hv_bias_walk_std', 0.00003);
config = defaults;

if isstruct(runtime_config) && isfield(runtime_config, 'measurement') ...
        && isstruct(runtime_config.measurement)
    config = merge_structs(config, runtime_config.measurement);
end

function angles = wrap_measurement_angles(angles)
angles = angles(:);
for idx = 1:numel(angles)
    angles(idx) = wrapToPi(angles(idx));
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

function [p_dot, v_dot, att_dot, omega_dot,h_v] = compute_quadrotor_dynamics_improved(...
    p, v, att, omega, q, q_dot, q_ddot, f_control, tau_control, params, ...
    payload_mass, omega_dot, v_dot_est)
% 实现论文公式(6)的四旋翼动力学

% === 位置动力学 ===
% 根据论文公式(6): ṗ = v, v̇ = g - f - f_c / (m_B + m_M)
% 位置导数
p_dot = v;
% fprintf('v: %s\n', mat2str(p_dot));

% 计算控制力向量 (论文中的f)
R = common_functions('euler_to_rotation_matrix', att);
f_vector = R * [0; 0; f_control];
% fprintf('f: %s\n', mat2str(f_vector));

% 计算动态耦合力 f_c (论文公式(7) - 完整实现)
[f_c,h_v] = compute_coupling_force_complete(att, omega, omega_dot, ...
                                     q, q_dot, q_ddot, params, payload_mass);
% fprintf('f_c: %s\n', mat2str(f_c));
% 重力向量
g = [0; 0; params.g];

% 速度导数
total_mass = params.m_B + params.m_M + payload_mass;
v_dot = -g + (f_vector - f_c) / total_mass;
% fprintf('a: %s\n', mat2str(v_dot));
% === 姿态动力学 ===
% 根据论文公式(6): Ṙ = R[ω]×, ω̇ = M⁻¹[τ - ω × (Mω) + τ_c]

% 姿态角导数 (从角速度计算)
att_dot = compute_euler_rates_from_body_rates(att, omega);
% fprintf('w: %s\n', mat2str(att_dot));
% 计算动态耦合力矩 τ_c (论文公式(8) - 完整实现)
tau_c = compute_coupling_torque_complete(p, v, v_dot_est, att, omega, omega_dot, ...
                                        q, q_dot, q_ddot, params, payload_mass);
% fprintf('tau_c: %s tau: %s\n', mat2str(tau_c), mat2str(tau_control));

% 陀螺力矩
gyro_torque = cross(omega, params.M * omega);
% fprintf('gyro_torque: %s\n', mat2str(gyro_torque));
% 角速度导数
% fprintf('M_inv: %s\n', mat2str(params.M_inv));
omega_dot = params.M_inv * (tau_control - gyro_torque + tau_c);
% fprintf('aw: %s\n', mat2str(omega_dot));
function [f_c,h_v] = compute_coupling_force_complete(att, omega, omega_dot, ...
                                              q, q_dot, q_ddot, params, payload_mass)
% 计算动态耦合力 f_c - 完整实现
% 根据论文公式(7):
% f_c = -(m_B + m_M) * R * [ω × (ω × p_C^B) + ω̇ × p_C^B + 2ω × ṗ_C^B + p̈_C^B]

% 计算飞行机械臂质心位置及其导数
[p_C_B, p_C_B_dot, p_C_B_ddot] = compute_mass_center_derivatives_complete(...
    q, q_dot, q_ddot, params, payload_mass);
% fprintf('质心：p_C_B: %s, p_C_B_dot: %s, p_C_B_ddot: %s\n', mat2str(p_C_B), mat2str(p_C_B_dot), mat2str(p_C_B_ddot));

% 计算旋转矩阵
R = common_functions('euler_to_rotation_matrix', att);

% 使用反对称矩阵计算各项（确保正确的叉乘运算）
omega_cross = common_functions('skew_symmetric', omega);
omega_dot_cross = common_functions('skew_symmetric', omega_dot);

% 计算各项
term1 = omega_cross *(omega_cross * p_C_B);  % ω × (ω × p_C^B)
term2 = omega_dot_cross * p_C_B;              % ω̇ × p_C^B
term3 = 2 * omega_cross * p_C_B_dot;          % 2ω × ṗ_C^B
term4 = p_C_B_ddot;                           % p̈_C^B

h_v = -R *(term2 + term3 + term4);
% 动态耦合力
total_mass = params.m_B + params.m_M + payload_mass;
f_c_body = term1 + term2 + term3 + term4;
f_c = -total_mass * R * f_c_body;

function tau_c = compute_coupling_torque_complete(p, v, v_dot, att, omega, omega_dot, ...
                                                 q, q_dot, q_ddot, params, payload_mass)
% 计算动态耦合力矩 τ_c - 完整实现
% 根据论文公式(8)

% 计算质心位置及其导数
[p_C_B, p_C_B_dot, p_C_B_ddot] = compute_mass_center_derivatives_complete(...
    q, q_dot, q_ddot, params, payload_mass);

% 计算机械臂惯性矩阵及其导数
[M_M_B, M_M_B_dot] = common_functions('compute_arm_inertia_advanced', q, q_dot, params);

% 计算旋转矩阵
R = common_functions('euler_to_rotation_matrix', att);

% 根据论文公式(8)计算各项
omega_cross = common_functions('skew_symmetric', omega);

term1 = -M_M_B * omega_dot;                    % -M_M^B * ω̇
term2 = -omega_cross *( M_M_B * omega);          % -ω × (M_M^B * ω)
term3 = -M_M_B_dot * omega;                    % -Ṁ_M^B * ω

% 重力和加速度项
total_mass = params.m_B + params.m_M + payload_mass;
gravity_body = [0; 0; -params.g];
term4 = total_mass * cross(p_C_B, R' * (gravity_body - v_dot));

% 高阶项（论文最后一项）
if payload_mass > 0
    mass_ratio = total_mass^2 / params.m_M;
else
    mass_ratio = (params.m_B + params.m_M)^2 / params.m_M;
end
term5 = -mass_ratio * (cross(p_C_B, p_C_B_ddot) - ...
                      (omega_cross * cross(p_C_B, p_C_B_dot)));

% 总的动态耦合力矩
tau_c = term1 + term2 + term3 + term4 + term5;

function [p_C_B, p_C_B_dot, p_C_B_ddot] = compute_mass_center_derivatives_complete(...
    q, q_dot, q_ddot, params, payload_mass)
% 计算质心位置及其导数 - 完整解析方法

% 质心位置
p_C_B = common_functions('compute_mass_center', q, params, payload_mass, 0, 0);

% 质心速度 - 解析计算
total_mass = params.m_B + params.m_M + payload_mass;
mass_ratio = (params.m_M + payload_mass) / total_mass;

% 计算雅可比矩阵
J_arm = compute_arm_mass_jacobian(q, params, payload_mass);
p_C_B_dot = mass_ratio * J_arm * q_dot;

% 质心加速度 - 解析计算
J_arm_dot = compute_arm_mass_jacobian_derivative(q, q_dot, params, payload_mass);
p_C_B_ddot = mass_ratio * (J_arm * q_ddot + J_arm_dot * q_dot);

function J = compute_arm_mass_jacobian(q, params, payload_mass)
% 计算机械臂质心雅可比矩阵
delta = 1e-6;
p_0 = common_functions('compute_mass_center', q, params, payload_mass, 0, 0);

J = zeros(3, 3);
for i = 1:3
    q_pert = q;
    q_pert(i) = q_pert(i) + delta;
    p_pert = common_functions('compute_mass_center', q_pert, params, payload_mass, 0, 0);
    J(:, i) = (p_pert - p_0) / delta;
end

function J_dot = compute_arm_mass_jacobian_derivative(q, q_dot, params, payload_mass)
% 计算雅可比矩阵的时间导数
delta_t = 1e-4;
q_next = q + q_dot * delta_t;
J_current = compute_arm_mass_jacobian(q, params, payload_mass);
J_next = compute_arm_mass_jacobian(q_next, params, payload_mass);
J_dot = (J_next - J_current) / delta_t;

function att_dot = compute_euler_rates_from_body_rates(att, omega)
% 论文公式(6)：Ṙ = R[ω]×
% 直接实现论文的旋转矩阵动力学，然后转换为欧拉角速率

% 按论文公式(6)计算
R = common_functions('euler_to_rotation_matrix', att);
omega_skew = common_functions('skew_symmetric', omega);

% 论文公式：Ṙ = R[ω]×
R_dot = R * omega_skew;
% fprintf('R_dot: %s\n', mat2str(R_dot));
% 从Ṙ提取欧拉角速率
att_dot = extract_euler_rates_from_R_dot(R_dot, att);

function euler_rates = extract_euler_rates_from_R_dot(R_dot, att)
% 从旋转矩阵导数计算欧拉角速率
% 使用欧拉角参数化的导数关系

theta = att(2);
cos_theta = cos(theta);

% 防止奇异点
if abs(cos_theta) < 1e-6
    % 接近万向锁，使用安全值
    euler_rates = [0; -R_dot(3,1); 0];
    return;
end

% 从R_dot的元素计算欧拉角速率
phi_dot = R_dot(3,2) / cos_theta;    % roll rate
theta_dot = -R_dot(3,1);             % pitch rate
psi_dot = R_dot(2,1) / cos_theta;    % yaw rate

euler_rates = [phi_dot; theta_dot; psi_dot];

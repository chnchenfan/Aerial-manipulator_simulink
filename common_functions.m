function varargout = common_functions(func_name, varargin)
%COMMON_FUNCTIONS 公共函数库
%
% 包含所有模块共用的数学函数和系统参数
% 修正版：解决了Delta运动学中的无限递归问题
%
% 调用方式：
%   result = common_functions('function_name', input1, input2, ...)

switch func_name
    case 'get_system_params'
        varargout{1} = get_system_params();
    case 'euler_to_rotation_matrix'
        varargout{1} = euler_to_rotation_matrix(varargin{1});
    case 'rotation_matrix_to_euler'
        varargout{1} = rotation_matrix_to_euler(varargin{1});
    case 'skew_symmetric'
        varargout{1} = skew_symmetric(varargin{1});
    case 'vee_map'
        varargout{1} = vee_map(varargin{1});
    case 'delta_forward_kinematics'
        [varargout{1}, varargout{2}] = delta_forward_kinematics(varargin{1}, varargin{2});
    case 'delta_inverse_kinematics'
        varargout{1} = delta_inverse_kinematics(varargin{1}, varargin{2});
    case 'delta_jacobian'
        varargout{1} = delta_jacobian(varargin{1}, varargin{2});
    case 'compute_mass_center'
        varargout{1} = compute_mass_center(varargin{1}, varargin{2}, varargin{3});
    case 'compute_arm_inertia'
        varargout{1} = compute_arm_inertia(varargin{1}, varargin{2});
    case 'compute_arm_inertia_advanced'
        [varargout{1}, varargout{2}] = compute_arm_inertia_advanced(varargin{1}, varargin{2}, varargin{3});
    case 'estimate_angular_acceleration'
        varargout{1} = estimate_angular_acceleration(varargin{1}, varargin{2});
    case 'estimate_velocity_derivative'
        varargout{1} = estimate_velocity_derivative(varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5});
    otherwise
        error('未知函数名: %s', func_name);
end

%=============================================================================
% 系统参数函数
%=============================================================================
function params = get_system_params()
% 获取系统参数（严格按照论文Table II）

% 质量参数 (论文实验平台数据)
params.m_B = 3.60;          % 四旋翼质量 (kg)
params.m_M = 1.00;          % Delta机械臂质量 (kg)
% params.m_M = 0;          % Delta机械臂质量 (kg)
% 四旋翼惯性矩阵 (kg⋅m²)
params.M = [0.0347, 0, 0;
            0, 0.0458, 0;
            0, 0, 0.0977];
params.M_inv = inv(params.M);

% Delta机械臂几何参数 (论文Section II-B)
params.l_U = 0.2;           % 上臂长度 (m)
params.l_L = 0.4;           % 下臂长度 (m)  
params.r_F = 0.1;           % 基座半径 (m)
params.r_M = 0.05;          % 末端平台半径 (m)

% 重力加速度
params.g = 9.81;            % m/s²

% % 控制器参数 (论文Section VII实验设置)位置越大越好，但是不能过大；姿态越小越好，但是不能过小
% params.Omega_p = diag([10.75, 10.75, 14.05]);    % 位置子环增益
% params.K_v = diag([5.5, 4.5, 15.80]);        % 速度增益
% params.Omega_q = diag([1.7, 1.7, 2.9]);    % 姿态子环增益  
% params.K_omega = diag([1.6, 1.6, 2.8]);    % 角速度增益
% params.k_beta = 2;                        % 四元数误差增益
% 
% % ESO参数 (论文Section VII) 可以补偿刚开始的超调，但是后面又出现超调
% params.w_p = [1.95, 1.95, 2.5];     % 位置ESO带宽 越大越好
% params.w_o = [1, 1, 2.5];  % 姿态ESO带宽 越小越好

params.Omega_p = diag([1.65, 1.65, 16.5]); % 位置子环增益
params.K_v = diag([6.2, 6.2, 16.5]); % 速度增益
params.Omega_q = diag([0.5, 1.5, 1.5]); % 姿态子环增益 
params.K_omega = diag([0.5, 1.5, 1.4]); % 角速度增益
params.k_beta = 1.9; % 四元数误差增益

% ESO参数 (论文Section VII) 可以补偿刚开始的超调，但是后面又出现超调
params.w_p = [3.8, 3.4, 3.0]; % 位置ESO带宽 越大越好
params.w_o = [4.8, 6.2, 6.2]; % 姿态ESO带宽 越小越好


% params.Omega_p = diag([1.75, 1.75, 9.85]); % 位置子环增益
% params.K_v = diag([5.5, 5.5, 8.80]); % 速度增益
% params.Omega_q = diag([0.5, 1.5, 1.5]); % 姿态子环增益 
% params.K_omega = diag([0.5, 1.5, 1.4]); % 角速度增益
% params.k_beta = 1.9; % 四元数误差增益
% 
% % ESO参数 (论文Section VII) 可以补偿刚开始的超调，但是后面又出现超调
% params.w_p = [1.95, 1.95, 2.1]; % 位置ESO带宽 越大越好
% params.w_o = [5, 7, 7]; % 姿态ESO带宽 越小越好


% 协作规划参数 (论文Section VI)
params.K_c = diag([1.2, 1.2, 1.2]);        % CLIK增益

% 采样时间
params.dt = 0.01;           % 10ms采样

% 默认负载质量
params.default_payload = 0.0;  % kg
params = apply_runtime_param_overrides(params);

%=============================================================================
% 坐标变换函数
%=============================================================================
function R = euler_to_rotation_matrix(euler)
% 欧拉角转旋转矩阵 (ZYX顺序)
% 输入: euler = [roll; pitch; yaw] (rad)
% 输出: R ∈ SO(3)

phi = euler(1);     % roll
theta = euler(2);   % pitch  
psi = euler(3);     % yaw

% 防止数值问题
phi = max(-pi/2+1e-6, min(pi/2-1e-6, phi));
theta = max(-pi/2+1e-6, min(pi/2-1e-6, theta));

R = [cos(theta)*cos(psi), -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi);
     cos(theta)*sin(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi);
     -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];

function euler = rotation_matrix_to_euler(R)
% 旋转矩阵转欧拉角 (ZYX顺序)
% 输入: R ∈ SO(3)
% 输出: euler = [roll; pitch; yaw] (rad)

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

euler = [roll; pitch; yaw];

function S = skew_symmetric(v)
% 向量到反对称矩阵
% 输入: v ∈ R³
% 输出: S ∈ R³ˣ³, 使得 S*u = v × u

S = [0, -v(3), v(2);
     v(3), 0, -v(1);
     -v(2), v(1), 0];

function v = vee_map(S)
% 反对称矩阵到向量 (vee算子)
% 输入: S ∈ R³ˣ³ 反对称矩阵
% 输出: v ∈ R³

v = [S(3,2); S(1,3); S(2,1)];

function params = apply_runtime_param_overrides(params)
try
    config = sim_tuning_runtime('get_config');
catch
    config = struct();
end

if ~isfield(config, 'controller') || ~isstruct(config.controller) || ...
        ~isfield(config.controller, 'system_params')
    return;
end

overrides = config.controller.system_params;
fields = fieldnames(overrides);

for i = 1:numel(fields)
    field_name = fields{i};
    params.(field_name) = overrides.(field_name);
end

if isfield(params, 'M')
    params.M_inv = inv(params.M);
end

%=============================================================================
% Delta机械臂运动学函数 (严格按照论文Section II-B)
%=============================================================================
function [p_E_D, J] = delta_forward_kinematics(q, params)
% Delta机械臂正运动学
% 输入: q = [q1; q2; q3] 关节角度 (rad)
%       params: 系统参数
% 输出: p_E_D: 末端位置在Delta坐标系中 (m)
%       J: 雅可比矩阵

% 根据论文公式(2)和(3)
l_U = params.l_U;
l_L = params.l_L;
r_F = params.r_F;
r_M = params.r_M;

% 计算各关节对应的h_i (论文公式(3))
h = zeros(3, 3);
for i = 1:3
    angle = (i-1) * pi/3;  % 120度分布
    h(1, i) = -(r_F - r_M + l_U * cos(q(i))) * cos(angle);
    h(2, i) = (r_F - r_M + l_U * cos(q(i))) * sin(angle);
    h(3, i) = l_U * sin(q(i));
end

% 求解末端位置 (论文公式(2): ||p_E_D - h_i||² = l_L²)
% 这是一个非线性方程组，使用数值方法求解
p_E_D = solve_forward_kinematics_numerical(h, l_L);

% 计算雅可比矩阵
J = compute_delta_jacobian_numerical(q, params);

function p_E_D = solve_forward_kinematics_numerical(h, l_L)
% 数值求解Delta机械臂正运动学
% 使用球面交线方法

% 初始猜测
p_E_D = [0; 0; -0.3];

% 牛顿迭代法求解
for iter = 1:20
    % 计算残差
    F = zeros(3, 1);
    for i = 1:3
        diff = p_E_D - h(:, i);
        F(i) = dot(diff, diff) - l_L^2;
    end
    
    % 计算雅可比矩阵
    J_num = zeros(3, 3);
    for i = 1:3
        diff = p_E_D - h(:, i);
        J_num(i, :) = 2 * diff';
    end
    
    % 牛顿步
    if rank(J_num) >= 3
        delta_p = -J_num \ F;
        p_E_D = p_E_D + delta_p;
    else
        break;
    end
    
    % 收敛判断
    if norm(F) < 1e-8
        break;
    end
end

function J = compute_delta_jacobian_numerical(q, params)
% 数值计算Delta机械臂雅可比矩阵
% 修正：避免循环调用，使用内部函数计算

delta_q = 1e-6;

% 先计算当前位置（不计算雅可比）
p_nominal = delta_forward_kinematics_no_jacobian(q, params);

J = zeros(3, 3);
for i = 1:3
    q_pert = q;
    q_pert(i) = q_pert(i) + delta_q;
    p_pert = delta_forward_kinematics_no_jacobian(q_pert, params);
    J(:, i) = (p_pert - p_nominal) / delta_q;
end

function p_E_D = delta_forward_kinematics_no_jacobian(q, params)
% 仅计算正运动学位置，不计算雅可比（避免循环调用）

l_U = params.l_U;
l_L = params.l_L;
r_F = params.r_F;
r_M = params.r_M;

% 计算h_i
h = zeros(3, 3);
for i = 1:3
    angle = (i-1) * pi/3;
    h(1, i) = -(r_F - r_M + l_U * cos(q(i))) * cos(angle);
    h(2, i) = (r_F - r_M + l_U * cos(q(i))) * sin(angle);
    h(3, i) = l_U * sin(q(i));
end

% 数值求解
p_E_D = solve_forward_kinematics_numerical(h, l_L);

function q = delta_inverse_kinematics(p_E_D, params)
% Delta机械臂逆运动学
% 输入: p_E_D: 期望末端位置在Delta坐标系中
% 输出: q: 关节角度

l_U = params.l_U;
l_L = params.l_L;
r_F = params.r_F;
r_M = params.r_M;

q = zeros(3, 1);

% 对每个关节求解 (3个独立的逆运动学问题)
for i = 1:3
    angle = (i-1) * pi/3;
    
    % 将问题转换到2D平面
    x_i = -p_E_D(1) * cos(angle) - p_E_D(2) * sin(angle);
    z_i = p_E_D(3);
    
    % 求解单关节逆运动学
    % 这是一个复杂的几何问题，使用数值方法
    q(i) = solve_single_joint_ik(x_i, z_i, l_U, l_L, r_F, r_M);
end

function q_i = solve_single_joint_ik(x, z, l_U, l_L, r_F, r_M)
% 求解单个关节的逆运动学
% 使用二分法求解

q_min = -pi/3;
q_max = pi/3;
tolerance = 1e-8;

for iter = 1:50
    q_mid = (q_min + q_max) / 2;
    
    % 计算当前角度对应的末端位置
    h_x = -(r_F - r_M + l_U * cos(q_mid));
    h_z = l_U * sin(q_mid);
    
    % 计算到期望位置的距离
    dist_sq = (x - h_x)^2 + (z - h_z)^2;
    error = dist_sq - l_L^2;
    
    if abs(error) < tolerance
        break;
    end
    
    % 计算导数确定搜索方向
    delta_q = 1e-6;
    h_x_pert = -(r_F - r_M + l_U * cos(q_mid + delta_q));
    h_z_pert = l_U * sin(q_mid + delta_q);
    dist_sq_pert = (x - h_x_pert)^2 + (z - h_z_pert)^2;
    error_pert = dist_sq_pert - l_L^2;
    
    if (error_pert - error) > 0
        q_max = q_mid;
    else
        q_min = q_mid;
    end
end

q_i = (q_min + q_max) / 2;

function J = delta_jacobian(q, params)
% 计算Delta机械臂雅可比矩阵
J = compute_delta_jacobian_numerical(q, params);

%=============================================================================
% 动力学相关函数 - 保持原版本
%=============================================================================
function p_C_B = compute_mass_center(q, params, payload_mass)
% 计算系统质心在机体坐标系中的位置（改进版）
% 考虑动态负载和精确的质量分布
% 输入: q - 机械臂关节角度 [q1; q2; q3]
%       params - 系统参数
%       payload_mass - 负载质量 (kg)，如果未提供则使用默认值
% 输出: p_C_B - 系统质心在机体坐标系中的位置

% 处理输入参数
if nargin < 3
    payload_mass = params.default_payload;
end

% 质量参数（根据论文数据）
m_quad = params.m_B;              % 四旋翼质量：3.60 kg
m_base = 0.56;                    % Delta基座质量：0.56 kg
m_arm = 0.44;                     % 可动臂质量：0.44 kg
m_payload = payload_mass;         % 动态负载

% 计算总质量
total_mass = m_quad + m_base + m_arm + m_payload;

% 初始化质心计算
weighted_sum = zeros(3, 1);

% 1. 四旋翼质心（在机体原点）
weighted_sum = weighted_sum + m_quad * [0; 0; 0];

% 2. Delta基座质心
p_base_B = [0; 0; -0.1];  % 基座在机体下方10cm
weighted_sum = weighted_sum + m_base * p_base_B;

% 3. 机械臂各部件质心（精确计算）
% 将可动臂质量平均分配到三个臂
m_per_arm = m_arm / 3;

for i = 1:3
    % 第i个臂的质心计算
    angle_i = (i-1) * 2*pi/3;  % 120度分布
    
    % 上臂质心（假设质量均匀分布）
    p_upper_center = [
        params.r_F * cos(angle_i) + 0.5 * params.l_U * cos(q(i)) * cos(angle_i);
        params.r_F * sin(angle_i) + 0.5 * params.l_U * cos(q(i)) * sin(angle_i);
        0.5 * params.l_U * sin(q(i))
    ];
    
    % 下臂末端位置
    p_end = compute_arm_endpoint(q(i), angle_i, params);
    
    % 下臂质心（中点）
    p_lower_center = (p_upper_center + p_end) / 2;
    
    % 转换到机体坐标系
    R_D_B = get_delta_to_body_rotation();
    p_upper_B = p_base_B + R_D_B * p_upper_center;
    p_lower_B = p_base_B + R_D_B * p_lower_center;
    
    % 累加质心贡献（假设上下臂质量相等）
    weighted_sum = weighted_sum + 0.5 * m_per_arm * p_upper_B + 0.5 * m_per_arm * p_lower_B;
end

% 4. 负载质心（在末端执行器处）
if m_payload > 0
    p_E_D = delta_forward_kinematics_no_jacobian(q, params);
    p_payload_B = p_base_B + get_delta_to_body_rotation() * p_E_D;
    weighted_sum = weighted_sum + m_payload * p_payload_B;
end

% 计算系统质心
p_C_B = weighted_sum / total_mass;
% p_C_B = [0;0;0];

function p_end = compute_arm_endpoint(qi, angle_i, params)
% 计算单个臂的末端位置
h_x = -(params.r_F - params.r_M + params.l_U * cos(qi)) * cos(angle_i);
h_y = (params.r_F - params.r_M + params.l_U * cos(qi)) * sin(angle_i);
h_z = params.l_U * sin(qi);
p_end = [h_x; h_y; h_z];

function R_D_B = get_delta_to_body_rotation()
% 根据论文Fig.2计算Delta坐标系到机体坐标系的旋转矩阵
% Delta坐标系: X向内，Y向左，Z向上
% 机体坐标系: X向前，Y向右，Z向下

R_D_B = [0, -1, 0;    % Delta的X轴 -> 机体的-Y轴
         1, 0, 0;     % Delta的Y轴 -> 机体的X轴  
         0, 0, -1];   % Delta的Z轴 -> 机体的-Z轴

function [M_M_B, M_M_B_dot] = compute_arm_inertia_advanced(q, q_dot, params)
% 计算机械臂惯性矩阵及其导数（高精度版本）
% 使用递归牛顿-欧拉方法，平衡精度和实时性

% 机械臂链接参数
link_mass = [0.15, 0.15, 0.14];  % 三个臂的质量分布
link_inertia_base = diag([0.002, 0.002, 0.0005]);  % 基础惯性张量

% 初始化惯性矩阵
M_M_B = zeros(3, 3);

% 计算每个链接的贡献
for i = 1:3
    angle_i = (i-1) * 2*pi/3;
    
    % 链接坐标系到机体坐标系的变换
    R_link = compute_link_rotation(q(i), angle_i);
    p_link = compute_link_position(q(i), angle_i, params);
    
    % 链接惯性在机体坐标系中的表示
    I_link = R_link * link_inertia_base * R_link';
    
    % 使用平行轴定理
    m_i = link_mass(i);
    M_M_B = M_M_B + I_link + m_i * (p_link' * p_link * eye(3) - p_link * p_link');
end

% 计算惯性矩阵导数
if nargout > 1
    % 使用解析方法计算导数
    M_M_B_dot = zeros(3, 3);
    
    for i = 1:3
        angle_i = (i-1) * 2*pi/3;
        
        % 角速度引起的旋转矩阵变化
        R_link = compute_link_rotation(q(i), angle_i);
        R_link_dot = compute_link_rotation_derivative(q(i), q_dot(i), angle_i);
        
        % 位置导数
        p_link = compute_link_position(q(i), angle_i, params);
        p_link_dot = compute_link_position_derivative(q(i), q_dot(i), angle_i, params);
        
        % 惯性导数贡献
        I_link_dot = R_link_dot * link_inertia_base * R_link' + ...
                     R_link * link_inertia_base * R_link_dot';
        
        % 平行轴定理的导数
        m_i = link_mass(i);
        parallel_axis_dot = m_i * (2 * p_link' * p_link_dot * eye(3) - ...
                                   p_link_dot * p_link' - p_link * p_link_dot');
        
        M_M_B_dot = M_M_B_dot + I_link_dot + parallel_axis_dot;
    end
end

function R = compute_link_rotation(qi, angle_i)
% 计算链接的旋转矩阵
R_base = [cos(angle_i), -sin(angle_i), 0;
          sin(angle_i), cos(angle_i), 0;
          0, 0, 1];
R_joint = [cos(qi), 0, sin(qi);
           0, 1, 0;
           -sin(qi), 0, cos(qi)];
R = get_delta_to_body_rotation() * R_base * R_joint;

function R_dot = compute_link_rotation_derivative(qi, qi_dot, angle_i)
% 计算旋转矩阵的导数
R_base = [cos(angle_i), -sin(angle_i), 0;
          sin(angle_i), cos(angle_i), 0;
          0, 0, 1];
R_joint_dot = [-sin(qi)*qi_dot, 0, cos(qi)*qi_dot;
               0, 0, 0;
               -cos(qi)*qi_dot, 0, -sin(qi)*qi_dot];
R_dot = get_delta_to_body_rotation() * R_base * R_joint_dot;

function p = compute_link_position(qi, angle_i, params)
% 计算链接质心位置
p_delta = [params.r_F * cos(angle_i) + 0.5 * params.l_U * cos(qi) * cos(angle_i);
           params.r_F * sin(angle_i) + 0.5 * params.l_U * cos(qi) * sin(angle_i);
           0.5 * params.l_U * sin(qi)];
p_base_B = [0; 0; -0.1];
p = p_base_B + get_delta_to_body_rotation() * p_delta;

function p_dot = compute_link_position_derivative(qi, qi_dot, angle_i, params)
% 计算链接质心位置导数
p_delta_dot = [-0.5 * params.l_U * sin(qi) * cos(angle_i) * qi_dot;
               -0.5 * params.l_U * sin(qi) * sin(angle_i) * qi_dot;
                0.5 * params.l_U * cos(qi) * qi_dot];
p_dot = get_delta_to_body_rotation() * p_delta_dot;

function M_M_B = compute_arm_inertia(q, params)
% 保留原函数接口，调用改进版本
M_M_B = compute_arm_inertia_advanced(q, zeros(3,1), params);

function [sys,x0,str,ts] = sfunc_cooperative_planner(t,x,u,flag)
%SFUNC_COOPERATIVE_PLANNER 协作规划器 - 严格按照论文实现
%
% 基于论文: "ESO-Based Robust and High-Precision Tracking Control for Aerial Manipulation"
% Section VI: Cooperative Planning
%
% 实现要求：
% - P-P模式：严格按照论文公式68-69实现CLIK方法
% - E-P模式：严格按照论文公式70-72实现QP方法
% - 物理约束：严格按照论文Section VI-A实现约束变换
%
% 输入参数 u:
%   u(1:3)   - 期望末端执行器位置 p_E,d ∈ R³ (m)
%   u(4:6)   - 期望末端执行器速度 ṗ_E,d ∈ R³ (m/s)
%   u(7:9)   - 当前四旋翼位置 p ∈ R³ (m)
%   u(10:12) - 当前四旋翼速度 v ∈ R³ (m/s)
%   u(13:15) - 当前四旋翼姿态角 [φ, θ, ψ] (rad)
%   u(16:18) - 当前四旋翼角速度 ω ∈ R³ (rad/s)
%   u(19:21) - 当前机械臂关节角度 q ∈ R³ (rad)
%   u(22:24) - 当前机械臂关节角速度 q̇ ∈ R³ (rad/s)
%   u(25)    - 规划模式 (1: P-P模式, 2: E-P模式)

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
        error('cooperative_planner_sfunc: 未处理的flag值: %d', flag);
end

%=============================================================================
% mdlInitializeSizes: 系统初始化
%=============================================================================
function [sys,x0,str,ts] = mdlInitializeSizes

sizes = simsizes;
sizes.NumContStates  = 0;   % 无连续状态
sizes.NumDiscStates  = 12;  % 简化状态存储：[p_d_prev; q_d_prev]
sizes.NumOutputs     = 12;  % 12个输出
sizes.NumInputs      = 25;  % 25个输入
sizes.DirFeedthrough = 1;   % 有直通项
sizes.NumSampleTimes = 1;   % 一个采样时间

sys = simsizes(sizes);

% 初始离散状态
x0 = [0; 0; 3; 0; 0; 0; 0; 0; 0; 0; 0; 0];  % [p_d_prev(3), v_d_prev(3), q_d_prev(3), q_dot_d_prev(3)]

str = [];
ts = [0.01 0]; % 采样时间10ms

%=============================================================================
% mdlUpdate: 离散状态更新
%=============================================================================
function sys = mdlUpdate(t,x,u)
% 存储当前计算结果，供下次迭代使用
sys = x;  % 保持状态，在mdlOutputs中更新

%=============================================================================
% mdlOutputs: 主要计算逻辑
%=============================================================================
function sys = mdlOutputs(t,x,u)

% 解析输入
p_E_d = u(1:3);        % 期望末端位置
p_E_d_dot = u(4:6);    % 期望末端速度
p = u(7:9);            % 当前四旋翼位置
v = u(10:12);          % 当前四旋翼速度
att = u(13:15);        % 当前四旋翼姿态
omega = u(16:18);      % 当前四旋翼角速度
q = u(19:21);          % 当前机械臂关节角度
q_dot = u(22:24);      % 当前机械臂关节角速度
mode = u(25);          % 规划模式

% 获取系统参数（严格按照论文Table II）
params = common_functions('get_system_params');
params = get_paper_accurate_params(params);

% 根据模式选择算法
if mode == 1
    % P-P模式：严格按照论文Section VI-B-1实现
    [p_d, v_d, q_d, q_dot_d] = pp_mode_paper_accurate(p_E_d, p_E_d_dot, ...
        p, v, att, omega, q, q_dot, params);
else
    % E-P模式：严格按照论文Section VI-B-2实现
    [p_d, v_d, q_d, q_dot_d] = ep_mode_paper_accurate(p_E_d, p_E_d_dot, ...
        p, v, att, omega, q, q_dot, params);
end

% 输出
sys = [p_d; v_d; q_d; q_dot_d];

%=============================================================================
% P-P模式：严格按照论文公式68-69实现
%=============================================================================
function [p_d, v_d, q_d, q_dot_d] = pp_mode_paper_accurate(...
    p_E_d, p_E_d_dot, p, v, att, omega, q, q_dot, params)
% P-P模式协作规划
% 严格实现论文Section VI-B-1的CLIK方法

% 步骤1：计算当前末端执行器位置（论文公式4）
R = common_functions('euler_to_rotation_matrix', att);
[p_E_D, J_arm] = common_functions('delta_forward_kinematics', q, params);

% 坐标系变换（论文Fig.2定义）
R_D_B = [0, -1, 0; 1, 0, 0; 0, 0, -1];  % Delta到机体坐标系
p_F_B = [0; 0; -0.1];  % Delta基座在机体坐标系中的位置

% 当前末端在世界坐标系中的位置
p_E = p + R * (p_F_B + R_D_B * p_E_D);

% 步骤2：CLIK算法（论文公式68）
% ṗ_E_D,CLIK = (RR_D^B)^T[ṗ_E,d - K_c(p_E - p_E,d) - ṗ + [Rp_E^B]_× ω]

p_E_error = p_E - p_E_d;  % 位置误差
p_E_B = p_F_B + R_D_B * p_E_D;  % 末端在机体坐标系中的位置

% CLIK反馈项
feedback_term = p_E_d_dot - params.K_c * p_E_error - v + ...
                common_functions('skew_symmetric', R * p_E_B) * omega;

% 期望末端速度在Delta坐标系中
p_E_D_dot_CLIK = (R * R_D_B)' * feedback_term;

% 步骤3：应用物理约束（论文公式69）
[p_E_D_dot_d, constraint_active] = apply_pp_physical_constraints(...
    p_E_D_dot_CLIK, p_E_D, q, q_dot, J_arm, params);

% 步骤4：通过雅可比逆运算得到关节速度
if cond(J_arm) < params.jacobian_condition_threshold
    q_dot_d = J_arm \ p_E_D_dot_d;
else
    % 使用阻尼最小二乘法（数值稳定性）
    lambda = params.damping_factor;
    q_dot_d = J_arm' * ((J_arm * J_arm' + lambda * eye(3)) \ p_E_D_dot_d);
end

% 步骤5：积分得到期望关节位置
q_d = q + q_dot_d * params.dt;

% 步骤6：重新计算期望末端位置
[p_E_D_d, ~] = common_functions('delta_forward_kinematics', q_d, params);

% 步骤7：计算四旋翼期望位置和速度
p_d = p_E_d - R * (p_F_B + R_D_B * p_E_D_d);

% 四旋翼期望速度（考虑机械臂运动的影响）
v_d = p_E_d_dot - R * R_D_B * J_arm * q_dot_d - ...
      cross(omega, R * (p_F_B + R_D_B * p_E_D_d));

%=============================================================================
% E-P模式：严格按照论文公式70-72实现
%=============================================================================
function [p_d, v_d, q_d, q_dot_d] = ep_mode_paper_accurate(...
    p_E_d, p_E_d_dot, p, v, att, omega, q, q_dot, params)
% E-P模式协作规划
% 严格实现论文Section VI-B-2的二次规划方法

% 步骤1：计算当前系统状态
R = common_functions('euler_to_rotation_matrix', att);
[p_E_D, J_arm] = common_functions('delta_forward_kinematics', q, params);
R_D_B = [0, -1, 0; 1, 0, 0; 0, 0, -1];
p_F_B = [0; 0; -0.1];

% 当前末端位置
p_E = p + R * (p_F_B + R_D_B * p_E_D);

% 步骤2：构造QP问题（论文公式70）
% 状态向量 s = [p^T, p_E_D^T]^T
s = [p; p_E_D];

% 计算雅可比矩阵J（论文公式5）
J = [eye(3), R * R_D_B * J_arm];

% 计算CLIK反馈项 s_c（论文公式71）
p_E_error = p_E - p_E_d;
p_E_B = p_F_B + R_D_B * p_E_D;
s_c = p_E_d_dot - params.K_c * p_E_error + ...
      common_functions('skew_symmetric', R * p_E_B) * omega;

% 步骤3：计算物理约束边界（论文Section VI-A）
[s_dot_min, s_dot_max] = compute_paper_physical_constraints(s, [v; J_arm * q_dot], params);

% 步骤4：构造权重矩阵W（论文公式72）
W = construct_weight_matrix(params);

% 步骤5：求解QP问题（论文公式70）
% min_{ṡ} (1/2)ṡ^T W ṡ
% s.t. J ṡ = s_c, s_dot_min ≤ ṡ ≤ s_dot_max
s_dot_optimal = solve_paper_qp_problem(W, J, s_c, s_dot_min, s_dot_max, params);

% 步骤6：提取结果
v_d = s_dot_optimal(1:3);      % 四旋翼期望速度
p_E_D_dot_d = s_dot_optimal(4:6);  % 末端期望速度

% 步骤7：计算机械臂期望关节速度
if cond(J_arm) < params.jacobian_condition_threshold
    q_dot_d = J_arm \ p_E_D_dot_d;
else
    lambda = params.damping_factor;
    q_dot_d = J_arm' * ((J_arm * J_arm' + lambda * eye(3)) \ p_E_D_dot_d);
end

% 步骤8：积分得到期望位置
p_d = p + v_d * params.dt;
q_d = q + q_dot_d * params.dt;

%=============================================================================
% 物理约束处理：严格按照论文Section VI-A实现
%=============================================================================
function [s_dot_min, s_dot_max] = compute_paper_physical_constraints(s, s_dot_current, params)
% 严格按照论文Section VI-A实现物理约束
% 包括位置约束(57)、速度约束(58)、加速度约束(59)的变换

dt = params.dt;
n = length(s);

% 初始化约束边界
s_dot_min = -inf(n, 1);
s_dot_max = inf(n, 1);

% 约束1：位置约束转换（论文公式62）
% s_min ≤ s + Δt·ṡ ≤ s_max  =>  (s_min - s)/Δt ≤ ṡ ≤ (s_max - s)/Δt
s_position_min = (params.s_min - s) / dt;
s_position_max = (params.s_max - s) / dt;

s_dot_min = max(s_dot_min, s_position_min);
s_dot_max = min(s_dot_max, s_position_max);

% 约束2：速度约束（论文公式58）
s_dot_min = max(s_dot_min, params.s_dot_min);
s_dot_max = min(s_dot_max, params.s_dot_max);

% 约束3：可行性条件（论文公式64-66）
% 严格按照论文实现线性化的可行性条件
for i = 1:n
    if abs(params.s_dot_min(i)) > 1e-8
        s_viab_min_i = 2 * params.s_ddot_max(i) / params.s_dot_min(i) * s(i) - ...
                       2 * params.s_ddot_max(i) / params.s_dot_min(i) * params.s_min(i);
        s_dot_min(i) = max(s_dot_min(i), s_viab_min_i);
    end
    
    if abs(params.s_dot_max(i)) > 1e-8
        s_viab_max_i = 2 * params.s_ddot_min(i) / params.s_dot_max(i) * s(i) - ...
                       2 * params.s_ddot_min(i) / params.s_dot_max(i) * params.s_max(i);
        s_dot_max(i) = min(s_dot_max(i), s_viab_max_i);
    end
end

% 约束4：加速度约束转换
s_accel_min = s_dot_current + params.s_ddot_min * dt;
s_accel_max = s_dot_current + params.s_ddot_max * dt;

s_dot_min = max(s_dot_min, s_accel_min);
s_dot_max = min(s_dot_max, s_accel_max);

% 确保约束可行性
for i = 1:n
    if s_dot_min(i) > s_dot_max(i)
        % 约束冲突，使用中值并给小幅度调整
        mid_val = 0.5 * (s_dot_min(i) + s_dot_max(i));
        tolerance = params.constraint_tolerance;
        s_dot_min(i) = mid_val - tolerance;
        s_dot_max(i) = mid_val + tolerance;
        
        if params.debug_mode
            warning('约束冲突在维度%d，已调整为可行范围', i);
        end
    end
end

function W = construct_weight_matrix(params)
% 构造权重矩阵（论文公式72）
% W = diag([w_u, w_u, w_u, 1, 1, 1])

w_u = params.w_u;
W = diag([w_u, w_u, w_u, 1, 1, 1]);

function s_dot_opt = solve_paper_qp_problem(W, J, s_c, s_dot_min, s_dot_max, params)
% 求解二次规划问题，采用多层次策略确保鲁棒性

% 方法1：尝试MATLAB的quadprog（如果可用）
if exist('quadprog', 'file') == 2
    try
        options = optimoptions('quadprog', ...
            'Display', 'off', ...
            'Algorithm', 'interior-point-convex', ...
            'MaxIterations', params.qp_max_iterations, ...
            'OptimalityTolerance', params.qp_optimality_tol, ...
            'ConstraintTolerance', params.qp_constraint_tol);
        
        [s_dot_opt, ~, exitflag] = quadprog(W, [], [], [], J, s_c, ...
                                            s_dot_min, s_dot_max, [], options);
        
        if exitflag > 0 && all(isfinite(s_dot_opt))
            % 验证解的质量
            constraint_error = norm(J * s_dot_opt - s_c);
            if constraint_error < params.solution_tolerance
                return;
            end
        end
    catch
        % quadprog失败，继续下一种方法
    end
end

% 方法2：论文中提到的投影梯度法（活跃集方法的变体）
if params.debug_mode
    fprintf('使用投影梯度法求解QP问题\n');
end

try
    s_dot_opt = solve_qp_projection_gradient(W, J, s_c, s_dot_min, s_dot_max, params);
    
    % 验证解的质量
    constraint_error = norm(J * s_dot_opt - s_c);
    if constraint_error < params.solution_tolerance && all(isfinite(s_dot_opt))
        return;
    end
catch
    % 投影梯度法也失败
end

% 方法3：最小二乘法（保底方案）
if params.debug_mode
    fprintf('QP求解失败，使用最小二乘法\n');
end

s_dot_opt = solve_constrained_least_squares(J, s_c, s_dot_min, s_dot_max, params);

function s_dot = solve_qp_projection_gradient(W, J, s_c, s_dot_min, s_dot_max, params)
% 投影梯度法求解QP问题

[m, n] = size(J);

% 计算零空间投影矩阵
[U, S, V] = svd(J, 'econ');
s_vals = diag(S);
tol = max(size(J)) * eps(max(s_vals));
r = sum(s_vals > tol);

if r < m
    if params.debug_mode
        fprintf('雅可比矩阵秩亏，rank=%d，期望=%d\n', r, m);
    end
end

% 伪逆
J_pinv = V(:,1:r) * diag(1./s_vals(1:r)) * U(:,1:r)';

% 初始解：满足等式约束的最小范数解
s_dot = J_pinv * s_c;
s_dot = project_to_box_constraints(s_dot, s_dot_min, s_dot_max);

% 零空间投影矩阵
P_null = eye(n) - J_pinv * J;

% 迭代参数
max_iter = params.pg_max_iterations;
alpha = params.pg_step_size;
tol_eq = params.pg_equality_tolerance;
tol_conv = params.pg_convergence_tolerance;

for iter = 1:max_iter
    % 修正等式约束
    eq_error = J * s_dot - s_c;
    if norm(eq_error) > tol_eq
        correction = J_pinv * eq_error;
        s_dot = s_dot - correction;
    end
    
    % 投影到盒约束
    s_dot = project_to_box_constraints(s_dot, s_dot_min, s_dot_max);
    
    % 梯度步
    grad = W * s_dot;
    s_dot_new = s_dot - alpha * P_null * grad;
    
    % 再次投影
    s_dot_new = project_to_box_constraints(s_dot_new, s_dot_min, s_dot_max);
    
    % 收敛检查
    if norm(s_dot_new - s_dot) < tol_conv
        s_dot = s_dot_new;
        break;
    end
    
    s_dot = s_dot_new;
    
    % 自适应步长
    if mod(iter, 20) == 0
        alpha = alpha * 0.9;
    end
end

% 最终等式约束修正
eq_error = J * s_dot - s_c;
if norm(eq_error) > tol_eq
    correction = J_pinv * eq_error;
    s_dot = s_dot - correction;
    s_dot = project_to_box_constraints(s_dot, s_dot_min, s_dot_max);
end

function s_dot = project_to_box_constraints(s_dot, s_dot_min, s_dot_max)
% 投影到盒约束
s_dot = max(s_dot_min, min(s_dot_max, s_dot));

function s_dot = solve_constrained_least_squares(J, s_c, s_dot_min, s_dot_max, params)
% 约束最小二乘法（保底方案）

% 无约束最小二乘解
s_dot = pinv(J) * s_c;

% 投影到约束
s_dot = project_to_box_constraints(s_dot, s_dot_min, s_dot_max);

% 迭代改善等式约束满足程度
for iter = 1:5
    eq_error = J * s_dot - s_c;
    if norm(eq_error) < params.solution_tolerance
        break;
    end
    
    correction = pinv(J) * eq_error;
    s_dot_new = s_dot - 0.5 * correction;
    s_dot_new = project_to_box_constraints(s_dot_new, s_dot_min, s_dot_max);
    
    if norm(J * s_dot_new - s_c) < norm(J * s_dot - s_c)
        s_dot = s_dot_new;
    else
        break;
    end
end

function [p_E_D_dot_d, constraint_active] = apply_pp_physical_constraints(...
    p_E_D_dot_CLIK, p_E_D, q, q_dot, J_arm, params)
% P-P模式的物理约束应用

% 转换为关节速度
if cond(J_arm) < params.jacobian_condition_threshold
    q_dot_desired = J_arm \ p_E_D_dot_CLIK;
else
    lambda = params.damping_factor;
    q_dot_desired = J_arm' * ((J_arm * J_arm' + lambda * eye(3)) \ p_E_D_dot_CLIK);
end

% 应用关节速度约束
q_dot_constrained = max(params.q_dot_min, min(params.q_dot_max, q_dot_desired));

% 检查关节位置约束
q_next = q + q_dot_constrained * params.dt;
for i = 1:3
    if q_next(i) < params.q_min(i)
        q_dot_constrained(i) = (params.q_min(i) - q(i)) / params.dt;
    elseif q_next(i) > params.q_max(i)
        q_dot_constrained(i) = (params.q_max(i) - q(i)) / params.dt;
    end
end

% 转换回末端速度
p_E_D_dot_d = J_arm * q_dot_constrained;

% 检查约束是否激活
constraint_active = norm(q_dot_constrained - q_dot_desired) > params.constraint_activation_threshold;

function params = get_paper_accurate_params(params)
% 获取严格按照论文的参数设置

% 物理约束参数（论文Table II中的实验平台约束）
% 状态向量 s = [p^T, p_E_D^T]^T
params.s_min = [-2.0; -2.0; 0.2; -0.08; -0.08; -0.30];     % 位置下限
params.s_max = [2.0; 2.0; 6.0; 0.08; 0.08; 0.15];         % 位置上限

params.s_dot_min = [-0.8; -0.8; -0.8; -0.8; -0.8; -0.8];  % 速度下限
params.s_dot_max = [0.8; 0.8; 0.8; 0.8; 0.8; 0.8];        % 速度上限

params.s_ddot_min = [-2.0; -2.0; -2.0; -2.0; -2.0; -2.0]; % 加速度下限
params.s_ddot_max = [2.0; 2.0; 2.0; 2.0; 2.0; 2.0];       % 加速度上限

% 关节约束（Delta机械臂的物理限制）
params.q_min = [-pi/3; -pi/3; -pi/3];                      % 关节角度下限
params.q_max = [pi/3; pi/3; pi/3];                         % 关节角度上限
params.q_dot_min = [-1.0; -1.0; -1.0];                     % 关节速度下限
params.q_dot_max = [1.0; 1.0; 1.0];                        % 关节速度上限

% 协作规划器参数
if ~isfield(params, 'K_c')
    params.K_c = diag([1.2, 1.2, 1.2]);                    % CLIK增益（论文Section VI）
end

params.w_u = 0.2;                                          % E-P模式权重（论文公式72）

% 数值计算参数
params.dt = 0.01;                                          % 采样时间
params.damping_factor = 0.01;                              % 阻尼最小二乘法参数
params.jacobian_condition_threshold = 1e6;                 % 雅可比条件数阈值

% QP求解器参数
params.qp_max_iterations = 1000;
params.qp_optimality_tol = 1e-6;
params.qp_constraint_tol = 1e-6;

% 投影梯度法参数
params.pg_max_iterations = 100;
params.pg_step_size = 0.01;
params.pg_equality_tolerance = 1e-4;
params.pg_convergence_tolerance = 1e-6;

% 容差参数
params.solution_tolerance = 1e-4;
params.constraint_tolerance = 0.01;
params.constraint_activation_threshold = 1e-6;

% 调试模式
params.debug_mode = true;  % 可以设为false关闭调试信息
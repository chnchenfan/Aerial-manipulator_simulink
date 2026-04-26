function [sys,x0,str,ts] = sfunc_position_controller(t,x,u,flag)
%SFUNC_POSITION_CONTROLLER Discrete position controller S-Function.

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 2
        sys = mdlUpdate(x,u);
    case 3
        sys = mdlOutputs(t,x,u);
    case {1,4,9}
        sys = [];
    otherwise
        error('position_controller_sfunc: unhandled flag %d', flag);
end

function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 3;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 27;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

x0 = [0; 0; 0];
str = [];
ts = [0.01 0];

function sys = mdlUpdate(x,u)
p_current = u(1:3);
p_desired = u(7:9);
pos_integral = x(1:3);
v_current = u(4:6);

params = common_functions('get_system_params');
limits = get_position_controller_limits();
pos_error = p_current - p_desired;
pos_integral_next = pos_integral + params.dt * pos_error;
pos_integral_next = max(-limits.integral_limit, min(limits.integral_limit, pos_integral_next));

sys = pos_integral_next;

function sys = mdlOutputs(t,x,u)
p_current = u(1:3);
v_current = u(4:6);
p_desired = u(7:9);
v_desired = u(10:12);
h_v_est = u(13:15);
att_current = u(16:18);
q_current = u(19:21);
omega_current = u(22:24);
a_desired = u(25:27);
pos_integral = x(1:3);

params = common_functions('get_system_params');
[f_total,f_desired_world] = compute_position_control( ...
    p_current, v_current, a_desired, p_desired, v_desired, ...
    pos_integral, att_current, h_v_est, q_current, omega_current, params);

sim_tuning_runtime('log', 'p_actual', t, p_current);
sim_tuning_runtime('log', 'v_actual', t, v_current);
sim_tuning_runtime('log', 'f', t, f_total);
sim_tuning_runtime('log', 'f_desired_world', t, f_desired_world);

sys = [f_total; f_desired_world];

function [f_total,f_desired_world] = compute_position_control( ...
    p, v, a_d, p_d, v_d, pos_integral, att_current, h_v_est, q, omega_current, params)
limits = get_position_controller_limits();
p_tilde = clamp_vector(p - p_d, limits.position_error_limit);
v_r = v_d - 2 * params.Omega_p * p_tilde - params.Omega_p^2 * pos_integral;

r_v = clamp_vector(v - v_r, limits.velocity_error_limit);
v_tilde = clamp_vector(v - v_d, limits.velocity_error_limit);
v_r_dot = a_d - 2 * params.Omega_p * v_tilde - params.Omega_p^2 * p_tilde;
u_i = -diag(limits.integral_feedback_gain) * pos_integral;
u_v = -params.K_v * r_v + v_r_dot + h_v_est + u_i;

p_C_B = common_functions('compute_mass_center', q, params, 0);
R_current = common_functions('euler_to_rotation_matrix', att_current);

g_gra = [0;0;params.g];
af = -R_current * cross(omega_current, cross(omega_current, p_C_B));

f_desired_world = (params.m_B + params.m_M) * (g_gra + af + u_v);

f_max = 2 * (params.m_B + params.m_M) * params.g;
f_min = 0.1 * (params.m_B + params.m_M) * params.g;
f_desired_world(3) = max(f_min, min(f_max, f_desired_world(3)));

max_lateral_force = limits.max_lateral_force_ratio * f_desired_world(3);
f_xy_norm = norm(f_desired_world(1:2));
if f_xy_norm > max_lateral_force
    f_desired_world(1:2) = f_desired_world(1:2) * max_lateral_force / max(f_xy_norm, 1e-9);
end

f_total = norm(f_desired_world);
f_total = max(f_min, min(f_max, f_total));

function limits = get_position_controller_limits()
limits = struct( ...
    'integral_limit', [0.30; 0.30; 0.50], ...
    'integral_feedback_gain', [0.0; 0.0; 1.60], ...
    'position_error_limit', [1.0; 1.0; 1.0], ...
    'velocity_error_limit', [2.0; 2.0; 2.0], ...
    'max_lateral_force_ratio', 0.38);

try
    config = sim_tuning_runtime('get_config');
catch
    config = struct();
end

if isfield(config, 'controller') && isstruct(config.controller) && ...
        isfield(config.controller, 'position_limits')
    limits = merge_structs(limits, config.controller.position_limits);
end

function y = clamp_vector(x, limit_vec)
limit_vec = limit_vec(:);
y = max(-limit_vec, min(limit_vec, x));

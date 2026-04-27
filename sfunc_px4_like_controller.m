function [sys,x0,str,ts] = sfunc_px4_like_controller(t,x,u,flag)
%SFUNC_PX4_LIKE_CONTROLLER PX4-inspired position/velocity thrust stage.
%
% This S-function keeps the existing plant interface: it outputs total thrust
% and a world-frame thrust vector consumed by the existing attitude reference
% converter. The paired PX4-like attitude/rate behavior is selected in
% sfunc_attitude_controller through config.controller.type = 'px4_like'.

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
        error('px4_like_controller_sfunc: unhandled flag %d', flag);
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
v_current = u(4:6);
p_desired = u(7:9);
v_desired = u(10:12);

params = common_functions('get_system_params');
cfg = get_px4_like_config();
pos_error = p_desired - p_current;
vel_sp = v_desired + cfg.pos_p(:) .* pos_error;
vel_sp = clamp_vector(vel_sp, cfg.vel_limit(:));
vel_error = vel_sp - v_current;

acc_i_next = x(:) + params.dt * (cfg.vel_i(:) .* vel_error);
acc_i_next = clamp_vector(acc_i_next, cfg.acc_i_limit(:));
sys = acc_i_next;

function sys = mdlOutputs(t,x,u)
p_current = u(1:3);
v_current = u(4:6);
p_desired = u(7:9);
v_desired = u(10:12);
a_desired = u(25:27);

params = common_functions('get_system_params');
cfg = get_px4_like_config();

pos_error = p_desired - p_current;
vel_sp = v_desired + cfg.pos_p(:) .* pos_error;
vel_sp = clamp_vector(vel_sp, cfg.vel_limit(:));

vel_error = vel_sp - v_current;
acc_sp = a_desired + cfg.vel_p(:) .* vel_error + x(:);
acc_sp = clamp_vector(acc_sp, cfg.acc_limit(:));

gravity = [0; 0; params.g];
total_mass = params.m_B + params.m_M;
f_desired_world = total_mass * (gravity + acc_sp);

f_min = cfg.thrust_min_ratio * total_mass * params.g;
f_max = cfg.thrust_max_ratio * total_mass * params.g;
f_desired_world(3) = max(f_min, min(f_max, f_desired_world(3)));

max_tilt = tan(cfg.tilt_limit_rad) * max(f_desired_world(3), f_min);
lateral_norm = norm(f_desired_world(1:2));
if lateral_norm > max_tilt
    f_desired_world(1:2) = f_desired_world(1:2) * max_tilt / max(lateral_norm, 1e-9);
end

f_total = norm(f_desired_world);
f_total = max(f_min, min(f_max, f_total));

sim_tuning_runtime('log', 'p_actual', t, p_current);
sim_tuning_runtime('log', 'v_actual', t, v_current);
sim_tuning_runtime('log', 'f', t, f_total);
sim_tuning_runtime('log', 'f_desired_world', t, f_desired_world);
sim_tuning_runtime('log', 'px4_vel_sp', t, vel_sp);
sim_tuning_runtime('log', 'px4_acc_sp', t, acc_sp);

sys = [f_total; f_desired_world];

function cfg = get_px4_like_config()
cfg = struct( ...
    'pos_p', [1.15; 1.15; 1.60], ...
    'vel_p', [2.20; 2.20; 3.20], ...
    'vel_i', [0.08; 0.08; 0.18], ...
    'vel_limit', [2.0; 2.0; 2.0], ...
    'acc_limit', [3.5; 3.5; 4.0], ...
    'acc_i_limit', [0.7; 0.7; 1.2], ...
    'tilt_limit_rad', deg2rad(22), ...
    'thrust_min_ratio', 0.10, ...
    'thrust_max_ratio', 2.00);

try
    config = sim_tuning_runtime('get_config');
catch
    config = struct();
end

if isfield(config, 'controller') && isstruct(config.controller) && ...
        isfield(config.controller, 'px4_like')
    cfg = merge_structs(cfg, config.controller.px4_like);
end

function y = clamp_vector(x, limit_vec)
limit_vec = limit_vec(:);
y = max(-limit_vec, min(limit_vec, x(:)));

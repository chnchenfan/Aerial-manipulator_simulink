function [sys, x0, str, ts] = sfunc_input(t, x, u, flag)
%SFUNC_INPUT Reference generator for aerial manipulator experiments.

config = get_input_config();

switch flag
    case 0
        [sys, x0, str, ts] = mdlInitializeSizes();
    case 3
        sys = mdlOutputs(t, config);
    case {1, 2, 4, 9}
        sys = [];
    otherwise
        error(['Unhandled flag = ', num2str(flag)]);
end

function [sys, x0, str, ts] = mdlInitializeSizes()
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 15;
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0.01 0];

function sys = mdlOutputs(t, config)
qd = zeros(9,1);
qdDot = zeros(9,1);
qdDotDot = zeros(9,1);

mode = config.mode;
T = config.T;
z_target = config.z_target;
pos_target = config.pos_target(:);
angle_max = config.arm_angle_max;
omega = 2 * pi * config.arm_frequency_hz;

if t <= T
    switch mode
        case 1
            [qd, qdDot, qdDotDot] = build_hover_ramp(t, T, z_target, angle_max, omega, config.arm_ramp_time, false);
        case 2
            [qd, qdDot, qdDotDot] = build_point_ramp(t, T, pos_target, angle_max, omega, config.arm_ramp_time, false);
        case 3
            [qd, qdDot, qdDotDot] = build_hover_hold(t, z_target, angle_max, omega, config.arm_ramp_time, true);
        case 4
            [qd, qdDot, qdDotDot] = build_spiral(t, T, z_target, config);
        case 5
            [qd, qdDot, qdDotDot] = build_square_track_mission(t, T, pos_target, z_target, angle_max, omega, config.arm_ramp_time, config.square_pause_ratio, true);
        otherwise
            error('Unknown mode = %d. Valid modes are 1, 2, 3, 4, 5.', mode);
    end
else
    switch mode
        case 1
            [qd, qdDot, qdDotDot] = build_hover_hold(t, z_target, angle_max, omega, config.arm_ramp_time, false);
        case 2
            [qd, qdDot, qdDotDot] = build_point_hold(pos_target, angle_max, omega, config.arm_ramp_time, false);
        case 3
            [qd, qdDot, qdDotDot] = build_hover_hold(t, z_target, angle_max, omega, config.arm_ramp_time, true);
        case 4
            [qd, qdDot, qdDotDot] = build_spiral_hold(t, z_target, config);
        case 5
            [qd, qdDot, qdDotDot] = build_square_track_hold(z_target, angle_max, omega, config.arm_ramp_time, true, t);
        otherwise
            error('Unknown mode = %d. Valid modes are 1, 2, 3, 4, 5.', mode);
    end
end

sim_tuning_runtime('log', 'p_desired', t, qd(1:3));
sim_tuning_runtime('log', 'v_desired', t, qdDot(1:3));
sim_tuning_runtime('log', 'q_desired', t, qd(7:9));
sim_tuning_runtime('log', 'qd_desired', t, qdDot(7:9));
sim_tuning_runtime('log', 'a_desired', t, qdDotDot(1:3));

sys = [qd(1:3); qdDot(1:3); qd(7:9); qdDot(7:9); qdDotDot(1:3)];

function config = get_input_config()
config = struct( ...
    'mode', 3, ...
    'T', 80.0, ...
    'z_target', 5.0, ...
    'pos_target', [0.08; 0.08; 5], ...
    'arm_angle_max', 1.0, ...
    'arm_frequency_hz', 0.5, ...
    'arm_ramp_time', 1.0, ...
    'square_pause_ratio', 0.60, ...
    'spiral_radius', 2.0, ...
    'spiral_turns', 3);

try
    runtime_config = sim_tuning_runtime('get_config');
catch
    runtime_config = struct();
end

if isfield(runtime_config, 'input') && isstruct(runtime_config.input)
    config = merge_structs(config, runtime_config.input);
end

function [qd, qdDot, qdDotDot] = build_hover_ramp(t, T, z_target, angle_max, omega, arm_ramp_time, arm_active)
qd = zeros(9,1);
qdDot = zeros(9,1);
qdDotDot = zeros(9,1);

tau = t / T;
qd(3) = z_target * tau^2 * (3 - 2 * tau);
qdDot(3) = z_target * 6 * tau * (1 - tau) / T;
qdDotDot(3) = z_target * 6 * (1 - 2 * tau) / (T^2);

if arm_active
    [qd(7:9), qdDot(7:9), qdDotDot(7:9)] = build_arm_sine(t, angle_max, omega, arm_ramp_time);
end

function [qd, qdDot, qdDotDot] = build_point_ramp(t, T, pos_target, angle_max, omega, arm_ramp_time, arm_active)
qd = zeros(9,1);
qdDot = zeros(9,1);
qdDotDot = zeros(9,1);

tau = t / T;
shape = tau^2 * (3 - 2 * tau);
shape_dot = 6 * tau * (1 - tau) / T;
shape_ddot = 6 * (1 - 2 * tau) / (T^2);

qd(1:3) = pos_target * shape;
qdDot(1:3) = pos_target * shape_dot;
qdDotDot(1:3) = pos_target * shape_ddot;

if arm_active
    [qd(7:9), qdDot(7:9), qdDotDot(7:9)] = build_arm_sine(t, angle_max, omega, arm_ramp_time);
end

function [qd, qdDot, qdDotDot] = build_planar_track_ramp(t, T, pos_target, z_target, angle_max, omega, arm_ramp_time, arm_active)
qd = zeros(9,1);
qdDot = zeros(9,1);
qdDotDot = zeros(9,1);

tau = t / T;
shape = tau^2 * (3 - 2 * tau);
shape_dot = 6 * tau * (1 - tau) / T;
shape_ddot = 6 * (1 - 2 * tau) / (T^2);

qd(1:2) = pos_target(1:2) * shape;
qd(3) = z_target;
qdDot(1:2) = pos_target(1:2) * shape_dot;
qdDotDot(1:2) = pos_target(1:2) * shape_ddot;

if arm_active
    [qd(7:9), qdDot(7:9), qdDotDot(7:9)] = build_arm_sine(t, angle_max, omega, arm_ramp_time);
end

function [qd, qdDot, qdDotDot] = build_hover_hold(t, z_target, angle_max, omega, arm_ramp_time, arm_active)
qd = zeros(9,1);
qdDot = zeros(9,1);
qdDotDot = zeros(9,1);
qd(3) = z_target;

if arm_active
    [qd(7:9), qdDot(7:9), qdDotDot(7:9)] = build_arm_sine(t, angle_max, omega, arm_ramp_time);
end

function [qd, qdDot, qdDotDot] = build_point_hold(pos_target, angle_max, omega, arm_ramp_time, arm_active, t)
if nargin < 6
    t = 0;
end

qd = zeros(9,1);
qdDot = zeros(9,1);
qdDotDot = zeros(9,1);
qd(1:3) = pos_target;

if arm_active
    [qd(7:9), qdDot(7:9), qdDotDot(7:9)] = build_arm_sine(t, angle_max, omega, arm_ramp_time);
end

function [qd, qdDot, qdDotDot] = build_planar_track_hold(pos_target, z_target, angle_max, omega, arm_ramp_time, arm_active, t)
qd = zeros(9,1);
qdDot = zeros(9,1);
qdDotDot = zeros(9,1);
qd(1:2) = pos_target(1:2);
qd(3) = z_target;

if arm_active
    [qd(7:9), qdDot(7:9), qdDotDot(7:9)] = build_arm_sine(t, angle_max, omega, arm_ramp_time);
end

function [qd, qdDot, qdDotDot] = build_square_track_mission(t, T, pos_target, z_target, angle_max, omega, arm_ramp_time, square_pause_ratio, arm_active)
qd = zeros(9,1);
qdDot = zeros(9,1);
qdDotDot = zeros(9,1);

takeoff_time = min(max(5.0, 0.25 * T), max(T - 4.0, 5.0));
takeoff_time = min(takeoff_time, 0.5 * T);
side_x = pos_target(1);
side_y = pos_target(2);

if t <= takeoff_time
    [qd, qdDot, qdDotDot] = build_hover_ramp(t, takeoff_time, z_target, angle_max, omega, arm_ramp_time, arm_active);
    return;
end

qd(3) = z_target;

square_time = max(T - takeoff_time, 1e-6);
local_t = min(t - takeoff_time, square_time);
pause_ratio = min(max(square_pause_ratio, 0), 0.8);
move_time = square_time * (1 - pause_ratio) / 4;
pause_time = square_time * pause_ratio / 4;
phase_time = move_time + pause_time;
phase_idx = min(4, floor(local_t / phase_time) + 1);
phase_t = local_t - (phase_idx - 1) * phase_time;

corners = [0, side_x, side_x, 0, 0;
           0, 0,      side_y, side_y, 0];
p0 = corners(:, phase_idx);
p1 = corners(:, phase_idx + 1);

if phase_t <= move_time
    [shape, shape_dot, shape_ddot] = smooth_segment(phase_t, move_time);
    delta = p1 - p0;
    qd(1:2) = p0 + delta * shape;
    qdDot(1:2) = delta * shape_dot;
    qdDotDot(1:2) = delta * shape_ddot;
else
    qd(1:2) = p1;
end

if arm_active
    [qd(7:9), qdDot(7:9), qdDotDot(7:9)] = build_arm_sine(t, angle_max, omega, arm_ramp_time);
end

function [qd, qdDot, qdDotDot] = build_square_track_hold(z_target, angle_max, omega, arm_ramp_time, arm_active, t)
qd = zeros(9,1);
qdDot = zeros(9,1);
qdDotDot = zeros(9,1);
qd(3) = z_target;

if arm_active
    [qd(7:9), qdDot(7:9), qdDotDot(7:9)] = build_arm_sine(t, angle_max, omega, arm_ramp_time);
end

function [shape, shape_dot, shape_ddot] = smooth_segment(t, duration)
if duration <= 0
    shape = 1;
    shape_dot = 0;
    shape_ddot = 0;
    return;
end

tau = min(max(t / duration, 0), 1);
shape = 6 * tau^5 - 15 * tau^4 + 10 * tau^3;
shape_dot = (30 * tau^4 - 60 * tau^3 + 30 * tau^2) / duration;
shape_ddot = (120 * tau^3 - 180 * tau^2 + 60 * tau) / (duration^2);

function [qd, qdDot, qdDotDot] = build_spiral(t, T, z_target, config)
qd = zeros(9,1);
qdDot = zeros(9,1);
qdDotDot = zeros(9,1);

tau = t / T;
s = 6*tau^5 - 15*tau^4 + 10*tau^3;
s_dot = (30*tau^4 - 60*tau^3 + 30*tau^2) / T;
s_ddot = (120*tau^3 - 180*tau^2 + 60*tau) / (T^2);

spiral_omega = 2*pi*config.spiral_turns/T;
theta = spiral_omega * t;
r = config.spiral_radius * s;
r_dot = config.spiral_radius * s_dot;
r_ddot = config.spiral_radius * s_ddot;

qd(1) = r * cos(theta);
qd(2) = r * sin(theta);
qd(3) = z_target * s;

qdDot(1) = r_dot * cos(theta) - r * sin(theta) * spiral_omega;
qdDot(2) = r_dot * sin(theta) + r * cos(theta) * spiral_omega;
qdDot(3) = z_target * s_dot;

qdDotDot(1) = r_ddot * cos(theta) - 2 * r_dot * sin(theta) * spiral_omega - ...
    r * cos(theta) * spiral_omega^2;
qdDotDot(2) = r_ddot * sin(theta) + 2 * r_dot * cos(theta) * spiral_omega - ...
    r * sin(theta) * spiral_omega^2;
qdDotDot(3) = z_target * s_ddot;

if norm(qdDot(1:2)) > 1e-6
    qd(6) = atan2(qdDot(2), qdDot(1));
end

function [qd, qdDot, qdDotDot] = build_spiral_hold(t, z_target, config)
qd = zeros(9,1);
qdDot = zeros(9,1);
qdDotDot = zeros(9,1);

spiral_omega = 2*pi*config.spiral_turns/config.T;
theta = spiral_omega * t;
r = config.spiral_radius;

qd(1) = r * cos(theta);
qd(2) = r * sin(theta);
qd(3) = z_target;

qdDot(1) = -r * sin(theta) * spiral_omega;
qdDot(2) = r * cos(theta) * spiral_omega;
qd(6) = theta + pi/2;
qdDot(6) = spiral_omega;

qdDotDot(1) = -r * cos(theta) * spiral_omega^2;
qdDotDot(2) = -r * sin(theta) * spiral_omega^2;

function [q_arm, qd_arm, qdd_arm] = build_arm_sine(t, angle_max, omega, ramp_time)
[gain, gain_dot, gain_ddot] = smooth_ramp(t, ramp_time);

base_pos = sin(omega * t);
base_vel = omega * cos(omega * t);
base_acc = -omega^2 * sin(omega * t);

q_arm = angle_max * gain * base_pos * ones(3,1);
qd_arm = angle_max * (gain_dot * base_pos + gain * base_vel) * ones(3,1);
qdd_arm = angle_max * (gain_ddot * base_pos + 2 * gain_dot * base_vel + gain * base_acc) * ones(3,1);

function [s, s_dot, s_ddot] = smooth_ramp(t, ramp_time)
if ramp_time <= 0
    s = 1;
    s_dot = 0;
    s_ddot = 0;
    return;
end

if t <= 0
    s = 0;
    s_dot = 0;
    s_ddot = 0;
elseif t >= ramp_time
    s = 1;
    s_dot = 0;
    s_ddot = 0;
else
    tau = t / ramp_time;
    s = tau^2 * (3 - 2 * tau);
    s_dot = 6 * tau * (1 - tau) / ramp_time;
    s_ddot = 6 * (1 - 2 * tau) / (ramp_time^2);
end

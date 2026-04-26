function [sys,x0,str,ts] = sfunc_arm_controller(t,x,u,flag)
%SFUNC_ARM_CONTROLLER Arm PID controller discrete S-Function.

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 2
        sys = mdlUpdate(t,x,u);
    case 3
        sys = mdlOutputs(t,x,u);
    case {1,4,9}
        sys = [];
    otherwise
        error('arm_controller_sfunc: unhandled flag %d', flag);
end

function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 3;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 12;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

x0 = [0; 0; 0];
str = [];
ts = [0.01 0];

function sys = mdlUpdate(~,x,u)
q_current = u(1:3);
q_desired = u(7:9);
q_integral = x(1:3);

params = get_arm_controller_params();
q_error = q_current - q_desired;
q_integral_next = q_integral + params.dt * q_error;
q_integral_next = max(-params.integral_limit, ...
    min(params.integral_limit, q_integral_next));

sys = q_integral_next;

function sys = mdlOutputs(t,x,u)
q_current = u(1:3);
q_dot_current = u(4:6);
q_desired = u(7:9);
q_dot_desired = u(10:12);
q_integral = x(1:3);

params = get_arm_controller_params();
tau_M = compute_arm_pid_control(q_current, q_dot_current, q_desired, ...
    q_dot_desired, q_integral, params);

sim_tuning_runtime('log', 'tau_arm', t, tau_M);
sim_tuning_runtime('log', 'q_actual', t, q_current);
sim_tuning_runtime('log', 'q_desired_ctrl', t, q_desired);
sim_tuning_runtime('log', 'qd_actual', t, q_dot_current);
sim_tuning_runtime('log', 'qd_desired_ctrl', t, q_dot_desired);
sys = tau_M;

function params = get_arm_controller_params()
params.K_p = diag([11, 13, 11]);
params.K_d = diag([1.1, 1.4, 1.1]);
params.K_i = diag([7, 6, 7]);
params.tau_max = [10; 10; 10];
params.dt = 0.01;
params.integral_limit = deg2rad(10);

try
    config = sim_tuning_runtime('get_config');
catch
    config = struct();
end

if isfield(config, 'controller') && isstruct(config.controller) && ...
        isfield(config.controller, 'arm_params')
    arm_overrides = config.controller.arm_params;
    override_fields = fieldnames(arm_overrides);
    for i = 1:numel(override_fields)
        params.(override_fields{i}) = arm_overrides.(override_fields{i});
    end
end

function tau_M = compute_arm_pid_control(q, q_dot, q_d, q_dot_d, q_integral, params)
q_error = q - q_d;
q_dot_error = q_dot - q_dot_d;

tau_M = -params.K_p * q_error - params.K_d * q_dot_error - params.K_i * q_integral;
tau_M = max(-params.tau_max, min(params.tau_max, tau_M));

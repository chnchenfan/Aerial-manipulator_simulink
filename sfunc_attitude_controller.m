function [sys,x0,str,ts] = sfunc_attitude_controller(t,x,u,flag)
%SFUNC_ATTITUDE_CONTROLLER Discrete attitude controller S-Function.

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 2
        sys = [];
    case 3
        sys = mdlOutputs(t,u);
    case {1,4,9}
        sys = [];
    otherwise
        error('attitude_controller_sfunc: unhandled flag %d', flag);
end

function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 9;
sizes.NumInputs      = 27;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

x0 = [];
str = [];
ts = [0.01 0];

function sys = mdlOutputs(t,u)
att_current = u(1:3);
omega_current = u(4:6);
omega_desired = u(7:9);
h_omega_est = u(10:12);
q_current = u(13:15);
omega_d_dot = u(16:18);
Rd_element = u(19:27);

params = common_functions('get_system_params');
[tau_control,u_omega,beta_v] = compute_attitude_control(att_current, omega_current, ...
    Rd_element, omega_desired, omega_d_dot, h_omega_est, q_current, params);

sim_tuning_runtime('log', 'att', t, att_current);
sim_tuning_runtime('log', 'omega', t, omega_current);
sim_tuning_runtime('log', 'tau', t, tau_control);
sim_tuning_runtime('log', 'u_omega', t, u_omega);
sim_tuning_runtime('log', 'beta_v', t, beta_v);

sys = [tau_control;u_omega;beta_v];

function [tau_control,u_omega,beta_v] = compute_attitude_control(att, omega, Rd_element, ...
    omega_d, omega_d_dot, h_omega_est, q, params)
Rd = [Rd_element(1) Rd_element(4) Rd_element(7);
      Rd_element(2) Rd_element(5) Rd_element(8);
      Rd_element(3) Rd_element(6) Rd_element(9)];

[beta_v, ~] = compute_attitude_error_quaternion(att, Rd);
R_tilde = compute_error_rotation_matrix(att, Rd);
omega_r = R_tilde' * omega_d - 2 * params.Omega_q * beta_v;

r_omega = omega - omega_r;
omega_r_dot = compute_omega_ref_derivative(att, omega, Rd, omega_d, ...
    omega_d_dot, beta_v, params);

u_omega = omega_r_dot - h_omega_est - params.M_inv * params.K_omega * r_omega - ...
    params.M_inv * params.k_beta * beta_v;

gyro_torque = cross(omega, params.M * omega);
tau_s = compute_known_torque_coupling(att, omega, q, params);
tau_control = params.M * (omega_r_dot - h_omega_est) + gyro_torque - ...
    params.K_omega * r_omega - params.k_beta * beta_v - tau_s;

if size(tau_control, 1) ~= 3 || size(tau_control, 2) ~= 1
    tau_control = reshape(tau_control(1:3), 3, 1);
end

limits = get_attitude_controller_limits();
tau_max = limits.tau_max;
tau_control = max(-tau_max, min(tau_max, tau_control));

if any(~isfinite(tau_control))
    warning('Attitude controller output contains non-finite values, using zeros.');
    tau_control = [0; 0; 0];
end

tau_control(abs(tau_control) < 1e-10) = 0;

function limits = get_attitude_controller_limits()
limits = struct('tau_max', 22);

try
    config = sim_tuning_runtime('get_config');
catch
    config = struct();
end

if isfield(config, 'controller') && isstruct(config.controller) && ...
        isfield(config.controller, 'attitude_limits')
    limits = merge_structs(limits, config.controller.attitude_limits);
end

function [beta_v, beta_0] = compute_attitude_error_quaternion(att, Rd)
R = common_functions('euler_to_rotation_matrix', att);
R_tilde = Rd' * R;
trace_R = trace(R_tilde);

if trace_R > 3 - 1e-6
    beta_0 = 1;
    beta_v = 0.5 * [R_tilde(3,2) - R_tilde(2,3);
                    R_tilde(1,3) - R_tilde(3,1);
                    R_tilde(2,1) - R_tilde(1,2)];
else
    beta_0 = 0.5 * sqrt(1 + trace_R);
    if beta_0 > 1e-6
        beta_v = (1 / (4 * beta_0)) * [R_tilde(3,2) - R_tilde(2,3);
                                       R_tilde(1,3) - R_tilde(3,1);
                                       R_tilde(2,1) - R_tilde(1,2)];
    else
        [~, max_idx] = max(diag(R_tilde));
        switch max_idx
            case 1
                beta_1 = 0.5 * sqrt(1 + R_tilde(1,1) - R_tilde(2,2) - R_tilde(3,3));
                beta_v = [beta_1;
                          (R_tilde(1,2) + R_tilde(2,1))/(4*beta_1);
                          (R_tilde(1,3) + R_tilde(3,1))/(4*beta_1)];
            case 2
                beta_2 = 0.5 * sqrt(1 - R_tilde(1,1) + R_tilde(2,2) - R_tilde(3,3));
                beta_v = [(R_tilde(1,2) + R_tilde(2,1))/(4*beta_2);
                          beta_2;
                          (R_tilde(2,3) + R_tilde(3,2))/(4*beta_2)];
            otherwise
                beta_3 = 0.5 * sqrt(1 - R_tilde(1,1) - R_tilde(2,2) + R_tilde(3,3));
                beta_v = [(R_tilde(1,3) + R_tilde(3,1))/(4*beta_3);
                          (R_tilde(2,3) + R_tilde(3,2))/(4*beta_3);
                          beta_3];
        end
        beta_0 = norm([R_tilde(3,2) - R_tilde(2,3);
                       R_tilde(1,3) - R_tilde(3,1);
                       R_tilde(2,1) - R_tilde(1,2)]) / (4 * norm(beta_v));
    end
end

function R_tilde = compute_error_rotation_matrix(att, Rd)
R = common_functions('euler_to_rotation_matrix', att);
R_tilde = Rd' * R;

function omega_r_dot = compute_omega_ref_derivative(att, omega, Rd, omega_d, ...
    omega_d_dot, beta_v, params)
R_tilde = compute_error_rotation_matrix(att, Rd);
beta_0 = sqrt(max(0, 1 - dot(beta_v, beta_v)));
e_omega = omega - R_tilde' * omega_d;

beta_v_cross = common_functions('skew_symmetric', beta_v);
beta_v_dot = 0.5 * (beta_0 * eye(3) + beta_v_cross) * e_omega;
R = common_functions('euler_to_rotation_matrix', att);
R_tilde_dot = compute_R_tilde_derivative(R, Rd, omega, omega_d);
omega_r_dot = R_tilde_dot' * omega_d + R_tilde' * omega_d_dot - 2 * params.Omega_q * beta_v_dot;

function tau_s = compute_known_torque_coupling(att, omega, q, params)
R = common_functions('euler_to_rotation_matrix', att);
p_C_B = common_functions('compute_mass_center', q, params, 0);
M_M_B = common_functions('compute_arm_inertia', q, params);

gravity_world = [0; 0; -params.g];
gravity_body = R' * gravity_world;
total_mass = params.m_B + params.m_M;
tau_gravity = total_mass * cross(p_C_B, gravity_body);
tau_inertia = -cross(omega, M_M_B * omega);

tau_s = tau_gravity + tau_inertia;

function R_tilde_dot = compute_R_tilde_derivative(R, R_d, omega, omega_d)
R_dot = R * common_functions('skew_symmetric', omega);
R_d_dot = R_d * common_functions('skew_symmetric', omega_d);
R_tilde_dot = R_d_dot' * R + R_d' * R_dot;

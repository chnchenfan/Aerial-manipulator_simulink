function [sys,x0,str,ts] = sfunc_Tranformation(t,x,u,flag)
%SFUNC_TRANFORMATION Desired attitude generator from world-frame force.

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 2
        sys = x;
    case 3
        sys = mdlOutputs(u);
    case {1,4,9}
        sys = [];
    otherwise
        error('transformation_sfunc: unhandled flag %d', flag);
end

function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

x0 = [];
str = [];
ts = [0.01 0];

function sys = mdlOutputs(u)
f_desired_world = u(1:3);
att = u(4:6);
params = common_functions('get_system_params');

[b1_d, b2_d, b3_d, omega_desired] = compute_Rd(f_desired_world, att, params);
sys = [b1_d; b2_d; b3_d; omega_desired];

function [b1_d, b2_d, b3_d, omega_desired] = compute_Rd(f_desired_world, att, params)
b3_d = f_desired_world / norm(f_desired_world);
R_current = common_functions('euler_to_rotation_matrix', att);
psi_d = 0;
a_psi = [cos(psi_d); sin(psi_d); 0];

if abs(dot(b3_d, [0; 0; 1])) > 0.99
    b1_d = [1; 0; 0];
    b2_d = cross(b3_d, b1_d);
    if norm(b2_d) > 1e-6
        b2_d = b2_d / norm(b2_d);
        b1_d = cross(b2_d, b3_d);
    else
        b2_d = [0; 1; 0];
        b1_d = cross(b2_d, b3_d);
    end
else
    b2_d = cross(b3_d, a_psi);
    if norm(b2_d) > 1e-6
        b2_d = b2_d / norm(b2_d);
    else
        b2_d = [0; 1; 0];
    end
    b1_d = cross(b2_d, b3_d);
end

R_d = [b1_d, b2_d, b3_d];
omega_desired = compute_desired_angular_velocity(R_current, R_d, params);

function omega_d = compute_desired_angular_velocity(R_current, R_d, params)
persistent R_d_prev;

if isempty(R_d_prev)
    R_d_prev = R_d;
    omega_d = [0; 0; 0];
    return;
end

dt = params.dt;
R_d_dot = (R_d - R_d_prev) / dt;
omega_skew = R_current' * R_d_dot;
omega_d = [omega_skew(3,2); omega_skew(1,3); omega_skew(2,1)];
R_d_prev = R_d;

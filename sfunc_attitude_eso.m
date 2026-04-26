function [sys,x0,str,ts] = sfunc_attitude_eso(t,x,u,flag)
%SFUNC_ATTITUDE_ESO Discrete attitude ESO S-Function.

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 2
        sys = mdlUpdate(x,u);
    case 3
        sys = mdlOutputs(t,x);
    case {1,4,9}
        sys = [];
    otherwise
        error('attitude_eso_sfunc: unhandled flag %d', flag);
end

function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 6;
sizes.NumOutputs     = 6;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

x0 = [0; 0; 0; 0; 0; 0];
str = [];
ts = [0.01 0];

function sys = mdlUpdate(x,u)
u_omega = u(1:3);
omega_measured = u(4:6);

omega_hat = x(1:3);
h_omega_hat = x(4:6);

params = common_functions('get_system_params');
omega_error = omega_measured - omega_hat;
w_o = params.w_o;
dt = params.dt;

omega_hat_next = omega_hat + dt * (u_omega + h_omega_hat + 2 * diag(w_o) * omega_error);
h_omega_hat_next = h_omega_hat + dt * (diag(w_o.^2) * omega_error);

sys = [omega_hat_next; h_omega_hat_next];

function sys = mdlOutputs(t,x)
sim_tuning_runtime('log', 'omega_hat', t, x(1:3));
sim_tuning_runtime('log', 'h_omega_est', t, x(4:6));
sys = x;

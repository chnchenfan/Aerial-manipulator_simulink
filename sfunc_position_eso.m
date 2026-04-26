function [sys,x0,str,ts] = sfunc_position_eso(t,x,u,flag)
%SFUNC_POSITION_ESO Discrete position ESO S-Function.

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
        error('position_eso_sfunc: unhandled flag %d', flag);
end

function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 9;
sizes.NumOutputs     = 9;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

x0 = [0; 0; 3; 0; 0; 0; 0; 0; 0];
str = [];
ts = [0.01 0];

function sys = mdlUpdate(x,u)
p_measured = u(1:3);
u_v = u(4:6);

p_hat = x(1:3);
v_hat = x(4:6);
h_v_hat = x(7:9);

params = common_functions('get_system_params');
p_error = p_measured - p_hat;
w_p = params.w_p;
dt = params.dt;

p_hat_next = p_hat + dt * (v_hat + 3 * diag(w_p) * p_error);
v_hat_next = v_hat + dt * (u_v + h_v_hat + 3 * diag(w_p.^2) * p_error);
h_v_hat_next = h_v_hat + dt * (diag(w_p.^3) * p_error);

sys = [p_hat_next; v_hat_next; h_v_hat_next];

function sys = mdlOutputs(t,x)
sim_tuning_runtime('log', 'p_hat', t, x(1:3));
sim_tuning_runtime('log', 'v_hat', t, x(4:6));
sim_tuning_runtime('log', 'h_v_est', t, x(7:9));
sys = x;

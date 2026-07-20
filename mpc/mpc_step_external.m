function u_seq = mpc_step_external(state_0, u_warm, u_last, spline)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    params = evalin('base', 'params');

    refs = mpc_build_reference(state_0, spline, params);

    u_seq = mpc_solve_fmincon(state_0, u_warm, u_last, refs, params);

end
function [H, f, Aineq, bineq, lb, ub] = ...
    lmpc_build_qp( ...
        xe0, Phi, Gamma, ...
        u_ref, ...
        u_prev_phys, ...
        params)

%% ==============================================================
% Base state/input cost
% ==============================================================

[H, f] = ...
    lmpc_build_cost( ...
        xe0, Phi, Gamma, params);


%% ==============================================================
% Input magnitude bounds
% ==============================================================

[lb, ub] = ...
    lmpc_build_input_bounds( ...
        u_ref, params);


%% ==============================================================
% Physical input-rate model
% ==============================================================

[D, d] = ...
    lmpc_build_rate_model( ...
        u_ref, ...
        u_prev_phys, ...
        params);


%% ==============================================================
% Input-rate cost
% ==============================================================

[Hdu, fdu] = ...
    lmpc_build_rate_cost( ...
        D, d, params);

H = H + Hdu;
f = f + fdu;


%% ==============================================================
% Input-rate constraints
% ==============================================================

[Aineq, bineq] = ...
    lmpc_build_rate_constraints( ...
        D, d, params);


%% Numerical symmetry

H = 0.5 * (H + H');

end
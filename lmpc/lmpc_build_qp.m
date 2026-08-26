function [H, f, lb, ub] = ...
    lmpc_build_qp(xe0, Phi, Gamma, refs, params)

%% Cost
[H, f] = ...
    lmpc_build_cost(xe0, Phi, Gamma, params);

%% Physical actuator constraints
[lb, ub] = ...
    lmpc_build_input_bounds(refs, params);

end
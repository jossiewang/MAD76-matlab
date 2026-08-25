function [u_opt, exitflag, output] = mpc_solve_fmincon(state_0, u0, u_last, refs, params)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

state_0 = double(state_0(:));
u0 = double(u0(:));
u_last = double(u_last(:));

N = params.N;

%% input bounds
u_min = [params.un_min; params.deltan_min];
u_max = [params.un_max; params.deltan_max];

lb = double(repmat(u_min, N, 1));
ub = double(repmat(u_max, N, 1));

%% cost and constraints
cost_function = @(u_seq) mpc_cost(u_seq, state_0, u_last, refs, params);
nonlcon = @(u_seq) mpc_constraints(u_seq, state_0, refs, params);

[A_rate, b_rate] = mpc_rate_constraints(N, u_last, params);

options = params.fmincon_options;

%% Solve
[u_opt, ~, exitflag, output] = fmincon(cost_function, u0, A_rate, b_rate, [], [], lb, ub, nonlcon, options);

end
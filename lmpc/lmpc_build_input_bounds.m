function [lb, ub] = ...
    lmpc_build_input_bounds(u_ref, params)
%LMPC_BUILD_INPUT_BOUNDS
%
% u_ref : nu x N physical feedforward input sequence
%
% QP variable:
%
%   U = [u_e,0;
%        u_e,1;
%        ...
%        u_e,N-1]
%
% Physical input:
%
%   u_phys = u_ref + u_e
%
% Therefore:
%
%   u_min - u_ref <= u_e <= u_max - u_ref

% N  = params.N;
N = 20;
nu = 2;

u_min = params.u_min;
u_max = params.u_max;

lb = zeros(nu*N,1);
ub = zeros(nu*N,1);

for i = 1:N

    rows = (i-1)*nu + (1:nu);

    uref_i = u_ref(:,i);

    lb(rows) = u_min - uref_i;
    ub(rows) = u_max - uref_i;

end

end
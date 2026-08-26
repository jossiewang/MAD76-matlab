function [lb, ub] = lmpc_build_input_bounds(refs, params)
%LMPC_BUILD_INPUT_BOUNDS
%
% Converts physical actuator limits into bounds on the deviation-input
% decision vector U.
%
% Physical input:
%
%   u_phys = u_ref + u_e
%
% therefore
%
%   u_min - u_ref <= u_e <= u_max - u_ref
%
% Stacked QP decision vector:
%
%   U = [u_e,0;
%        u_e,1;
%        ...
%        u_e,N-1]

N  = params.N;
nu = 2;

u_min = params.u_min;
u_max = params.u_max;

lb = zeros(nu*N,1);
ub = zeros(nu*N,1);

for i = 1:N

    rows = (i-1)*nu + (1:nu);

    uref = refs.u_ref(:,i);

    lb(rows) = u_min - uref;
    ub(rows) = u_max - uref;

end

end
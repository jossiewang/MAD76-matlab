function [D, d] = ...
    lmpc_build_rate_model(refs, u_prev_phys, params)
%LMPC_BUILD_RATE_MODEL
%
% Builds
%
%   Delta U_phys = D*U + d
%
% where U is the stacked deviation-input sequence.

N  = params.N;
nu = 2;

%% Difference matrix

D = zeros(nu*N, nu*N);

for i = 1:N

    rows = (i-1)*nu + (1:nu);

    % current deviation input
    cols = (i-1)*nu + (1:nu);
    D(rows,cols) = eye(nu);

    if i > 1
        cols_prev = (i-2)*nu + (1:nu);
        D(rows,cols_prev) = -eye(nu);
    end

end

%% Stack reference physical inputs

Uref = zeros(nu*N,1);

for i = 1:N

    rows = (i-1)*nu + (1:nu);

    Uref(rows) = refs.u_ref(:,i);

end

%% Previous physical input

Euprev = zeros(nu*N,1);
Euprev(1:nu) = u_prev_phys;

%% Affine part

d = D*Uref - Euprev;

end
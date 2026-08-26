function [H, f] = ...
    lmpc_build_cost(xe0, Phi, Gamma, params)
%LMPC_BUILD_COST
%
% Build the quadratic cost
%
%   min 1/2 U' H U + f' U
%
% corresponding to
%
%   sum x_i'Qx_i + u_i'Ru_i + x_N'Px_N

N  = params.N;
nx = size(Phi,2);
nu = size(params.R,1);

Q = params.Q;
R = params.R;
P = params.P;

%% Stacked state weighting

Qbar = zeros(nx*N, nx*N);

for i = 1:N-1
    rows = (i-1)*nx + (1:nx);
    Qbar(rows,rows) = Q;
end

% Terminal block
rows = (N-1)*nx + (1:nx);
Qbar(rows,rows) = P;

%% Stacked input weighting

Rbar = kron(eye(N), R);

%% QP matrices

H = 2 * (Gamma' * Qbar * Gamma + Rbar);

f = 2 * Gamma' * Qbar * Phi * xe0;

%% Numerical symmetry protection

H = 0.5 * (H + H');

end
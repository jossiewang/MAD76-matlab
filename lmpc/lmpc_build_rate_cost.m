function [Hdu, fdu] = ...
    lmpc_build_rate_cost(D, d, params)
%LMPC_BUILD_RATE_COST
%
% Cost:
%
%   J_du = DeltaU_phys' * Rdu_bar * DeltaU_phys
%
% with
%
%   DeltaU_phys = D*U + d

N = params.N;

Rdu = params.Rdu;

%% Stacked rate weighting matrix

Rdu_bar = kron(eye(N), Rdu);

%% QP contribution

Hdu = 2 * (D' * Rdu_bar * D);

fdu = 2 * (D' * Rdu_bar * d);

%% Numerical symmetry

Hdu = 0.5 * (Hdu + Hdu');

end
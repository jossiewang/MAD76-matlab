function [Phi, Gamma] = ...
    lmpc_build_prediction_matrices(Ad, Bd, params)
%LMPC_BUILD_PREDICTION_MATRICES
%
% Constructs
%
%       X = Phi*x0 + Gamma*U
%
% where
%
%   X = [x_1;
%        x_2;
%        ...
%        x_N]
%
%   U = [u_0;
%        u_1;
%        ...
%        u_{N-1}]
%
% for the LTV system
%
%   x_{i+1} = A_i*x_i + B_i*u_i.
%
% Inputs:
%   Ad : nx x nx x N
%   Bd : nx x nu x N
%
% Outputs:
%   Phi   : (nx*N) x nx
%   Gamma : (nx*N) x (nu*N)

%% Dimensions

nx = size(Ad,1);
nu = size(Bd,2);
% N  = params.N;
N = 20;

%% Preallocate

Phi   = zeros(nx*N, nx);
Gamma = zeros(nx*N, nu*N);

%% Recursive construction

% Maps x0 -> current predicted state
Phi_current = eye(nx);

% Maps entire U -> current predicted state
Gamma_current = zeros(nx, nu*N);

for i = 1:N

    Ai = Ad(:,:,i);
    Bi = Bd(:,:,i);

    %% State transition from x0

    Phi_current = Ai * Phi_current;

    %% Input transition

    Gamma_current = Ai * Gamma_current;

    cols = (i-1)*nu + (1:nu);

    Gamma_current(:, cols) = Bi;

    %% Store current prediction

    rows = (i-1)*nx + (1:nx);

    Phi(rows,:) = Phi_current;

    Gamma(rows,:) = Gamma_current;

end

end
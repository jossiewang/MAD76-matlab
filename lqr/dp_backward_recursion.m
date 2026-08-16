function [K_matrix, S_matrix] = dp_backward_recursion(sys, weights, N)
%DP_BACKWARD_RECURSION
%
% Solves the finite-horizon LTV-LQR problem using backward
% dynamic programming.
%
% System:
%
%   x_{k+1} = A_k x_k + B_k u_k
%
% Cost:
%
%           N-1
%   J =     sum  (x_k' Q x_k + u_k' R u_k)
%           k=0
%
%       + x_N' P x_N
%
% Optimal feedback:
%
%   u_k = K_k x_k
%
%
% INPUTS
%
% sys.A:
%   n x n x N
%
%   sys.A(:,:,k+1) corresponds to A_k
%
% sys.B:
%   n x m x N
%
%   sys.B(:,:,k+1) corresponds to B_k
%
% weights.Q:
%   n x n state weighting matrix
%
% weights.R:
%   m x m input weighting matrix
%
% weights.P:
%   n x n terminal weighting matrix
%
% N:
%   horizon length
%
%
% OUTPUTS
%
% K_matrix:
%   m x n x N
%
%   K_matrix(:,:,k+1) = K_k
%
% S_matrix:
%   n x n x (N+1)
%
%   S_matrix(:,:,k+1) = S_k

%% ================================================================
% System dimensions
% ================================================================

[n, m] = size(sys.B(:,:,1));

%% ================================================================
% Preallocate
% ================================================================

K_matrix = zeros(m, n, N);
S_matrix = zeros(n, n, N+1);

%% Terminal value function
%
% V_N(x_N) = x_N' P x_N

S_matrix(:,:,N+1) = weights.P;

%% ================================================================
% Backward dynamic-programming recursion
%
% Matlab index:
%
%   index = N     -> k = N-1
%   index = N-1   -> k = N-2
%   ...
%   index = 1     -> k = 0
% ================================================================

for index = N:-1:1

    %% System matrices at stage k

    Ak = sys.A(:,:,index);
    Bk = sys.B(:,:,index);

    %% S_{k+1}

    Sk1 = S_matrix(:,:,index+1);

    %% ------------------------------------------------------------
    % Optimal feedback gain
    %
    % K_k =
    %
    % -(R + B_k' S_{k+1} B_k)^(-1)
    %       B_k' S_{k+1} A_k
    % -------------------------------------------------------------

    H = weights.R + Bk' * Sk1 * Bk;

    Kk = -H \ (Bk' * Sk1 * Ak);

    K_matrix(:,:,index) = Kk;

    %% ------------------------------------------------------------
    % Riccati recursion
    %
    % S_k =
    %
    % Q
    % + A_k' S_{k+1} A_k
    % + A_k' S_{k+1} B_k K_k
    % -------------------------------------------------------------

    S_matrix(:,:,index) = ...
          weights.Q ...
        + Ak' * Sk1 * Ak ...
        + Ak' * Sk1 * Bk * Kk;

end

end
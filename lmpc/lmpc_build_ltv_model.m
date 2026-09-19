function [Ad, Bd] = lmpc_build_ltv_model(refs, params)
%LMPC_BUILD_LTV_MODEL
% Build the discrete-time LTV error model along the reference horizon.
%
% Model:
%   x_e = [v_e;
%          s_c1e;
%          s_c2e;
%          psi_e]
%
%   u_e = [u_e;
%          delta_e]
%
% Dynamics:
%   x_{i+1} = Ad(:,:,i) * x_i + Bd(:,:,i) * u_i
%
% Inputs:
%   refs.v       : reference velocity along horizon
%   refs.kappa   : reference curvature along horizon
%   refs.delta   : reference steering angle [rad]
%
%   params.N
%   params.Ta
%   params.T
%   params.ku
%   params.l
%
% Outputs:
%   Ad : 4 x 4 x N
%   Bd : 4 x 2 x N

%% Dimensions

nx = 4;
nu = 2;
N  = params.N;

%% Parameters

Ta = params.Ta;
T  = params.T;
ku = params.ku;
l  = params.l;

%% Preallocate

Ad = zeros(nx, nx, N);
Bd = zeros(nx, nu, N);

%% Build LTV model along prediction horizon

for i = 1:N

    % Reference quantities at stage i
    v_ref     = refs.v(i);
    kappa_ref = refs.kappa(i);
    delta_ref = refs.delta(i);

    %% Continuous-time linearized model

    Ac = zeros(nx, nx);
    Bc = zeros(nx, nu);

    % v_e_dot
    Ac(1,1) = -1 / T;

    % s_c1e_dot
    Ac(2,1) = 1;
    Ac(2,3) = v_ref * kappa_ref;

    % s_c2e_dot
    Ac(3,2) = -v_ref * kappa_ref;
    Ac(3,4) = v_ref;

    % psi_e_dot
    Ac(4,1) = kappa_ref;

    % pedal input
    Bc(1,1) = ku / T;

    % steering input
    Bc(4,2) = v_ref / ...
        (l * cos(delta_ref)^2);

    %% Exact ZOH discretization

    M = [Ac, Bc;
        zeros(nu, nx + nu)];

    Md = expm(M * Ta);

    Ad(:,:,i) = Md(1:nx, 1:nx);
    Bd(:,:,i) = Md(1:nx, nx+1:nx+nu);

end

end
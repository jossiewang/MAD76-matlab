function [D, d] = ...
    lmpc_build_rate_model( ...
        u_ref, ...
        u_prev_phys, ...
        params)
%LMPC_BUILD_RATE_MODEL
%
% Builds
%
%   DeltaU_phys = D*U + d
%
% where U is the stacked deviation-input sequence.
%
% Physical input:
%
%   u_phys,i = u_ref,i + u_e,i
%
% First rate:
%
%   Delta u_phys,0 =
%       u_phys,0 - u_prev_phys
%
% Remaining:
%
%   Delta u_phys,i =
%       u_phys,i - u_phys,i-1

% N  = params.N;
N = 20;
nu = 2;

%% ==============================================================
% Difference matrix
% ==============================================================

D = zeros(nu*N, nu*N);

Iu = eye(nu);

% First input
D(1:nu,1:nu) = Iu;

for i = 2:N

    rows = (i-1)*nu + (1:nu);

    cols_prev = (i-2)*nu + (1:nu);
    cols_curr = (i-1)*nu + (1:nu);

    D(rows,cols_prev) = -Iu;
    D(rows,cols_curr) =  Iu;

end


%% ==============================================================
% Reference/previous-input offset
%
% d =
%
% [u_ref,0 - u_prev_phys;
%  u_ref,1 - u_ref,0;
%  ...
%  u_ref,N-1 - u_ref,N-2]
% ==============================================================

d = zeros(nu*N,1);

% First rate
d(1:nu) = ...
    u_ref(:,1) - u_prev_phys;

% Remaining rates
for i = 2:N

    rows = (i-1)*nu + (1:nu);

    d(rows) = ...
        u_ref(:,i) - ...
        u_ref(:,i-1);

end

end
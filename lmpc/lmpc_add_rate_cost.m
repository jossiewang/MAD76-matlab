function [H, f] = lmpc_add_rate_cost( ...
    H, f, refs, u_prev_phys, params)
%LMPC_ADD_RATE_COST
%
% Adds physical input-rate cost
%
% Jdu = sum_i Delta_u_phys(i)' * Rdu * Delta_u_phys(i)
%
% without explicitly constructing Du or Rdu_bar.

N   = params.N;
nu  = params.nu;
Rdu = params.Rdu;

%% ==============================================================
% Build reference-rate offsets d_i
%
% d0 = u_ref,0 - u_prev_phys
% di = u_ref,i - u_ref,i-1
% ==============================================================

d_prev = refs.u_ref(:,1) - u_prev_phys;

%% First stage contribution

idx = 1:nu;

% Delta u_0 = u_e,0 + d_0
H(idx,idx) = H(idx,idx) + 2*Rdu;

f(idx) = f(idx) + 2*Rdu*d_prev;


%% ==============================================================
% Remaining stages
% ==============================================================

for i = 2:N

    idx_prev = (i-2)*nu + (1:nu);
    idx_curr = (i-1)*nu + (1:nu);

    d_i = refs.u_ref(:,i) - refs.u_ref(:,i-1);

    % Delta u_i =
    %     u_e,i - u_e,i-1 + d_i
    %
    % Quadratic contributions

    H(idx_prev,idx_prev) = ...
        H(idx_prev,idx_prev) + 2*Rdu;

    H(idx_curr,idx_curr) = ...
        H(idx_curr,idx_curr) + 2*Rdu;

    H(idx_prev,idx_curr) = ...
        H(idx_prev,idx_curr) - 2*Rdu;

    H(idx_curr,idx_prev) = ...
        H(idx_curr,idx_prev) - 2*Rdu;

    % Linear contributions

    f(idx_prev) = ...
        f(idx_prev) - 2*Rdu*d_i;

    f(idx_curr) = ...
        f(idx_curr) + 2*Rdu*d_i;

end

end
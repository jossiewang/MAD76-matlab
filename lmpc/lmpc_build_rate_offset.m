function d = ...
    lmpc_build_rate_offset(refs, u_prev_phys, params)
%LMPC_BUILD_RATE_OFFSET
%
% Physical inputs:
%
%   Uphys = Uref + U
%
% Physical rate:
%
%   DeltaUphys = Du*U + d
%
% where
%
%   d = Du*Uref + [-u_prev_phys; 0; ...; 0]

N  = params.N;
nu = params.nu;

%% Stack feedforward/reference inputs

Uref = zeros(nu*N,1);

for i = 1:N

    rows = (i-1)*nu + (1:nu);

    Uref(rows) = refs.u_ref(:,i);

end

%% Reference contribution

d = params.Du * Uref;

%% Previous actually applied physical input

d(1:nu) = d(1:nu) - u_prev_phys;

end
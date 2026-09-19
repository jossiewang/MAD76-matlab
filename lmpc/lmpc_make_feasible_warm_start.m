function Uinit = ...
    lmpc_make_feasible_warm_start( ...
    Ushift, refs, u_prev_phys, params)

N  = params.N;
nu = 2;

Uinit = zeros(nu*N,1);

u_last = u_prev_phys;

for i = 1:N

    rows = (i-1)*nu + (1:nu);

    %% Shifted solution expressed as physical input
    u_des = refs.u_ref(:,i) + Ushift(rows);

    %% Physical magnitude limits
    u_des = min( ...
        max(u_des, params.u_min), ...
        params.u_max);

    %% Physical rate limits
    rate_lower = u_last - params.du_max;
    rate_upper = u_last + params.du_max;

    u_new = min( ...
        max(u_des, rate_lower), ...
        rate_upper);

    %% Magnitude limits again for safety
    u_new = min( ...
        max(u_new, params.u_min), ...
        params.u_max);

    %% Return to deviation coordinates
    Uinit(rows) = ...
        u_new - refs.u_ref(:,i);

    u_last = double(u_new);

end

end
function [c, ceq] = ...
    mpc_constraints(U, z0, u_delay, refs, params)

U       = double(U(:));
z       = double(z0(:));
u_delay = double(u_delay);

N  = params.N;
nd = size(u_delay,2);

v_min = params.v_min;
v_max = params.v_max;

track_half_width = params.track_half_width;

c = zeros(4*N,1);
ceq = [];

k = 1;

for i = 1:N

    %% ================================================================
    % Determine input ACTUALLY affecting vehicle

    if i <= nd

        u_applied = u_delay(:,i);

    else

        j = i - nd;

        idx = 2*j - 1;

        u_applied = U(idx:idx+1);

    end


    %% Predict next state
    z = mpc_model_step(z, u_applied, params);


    %% Speed constraints

    vr = z(1);

    c(k) = vr - v_max;
    k = k + 1;

    c(k) = v_min - vr;
    k = k + 1;


    %% Track corridor

    ds1 = z(2) - refs.s(1,i+1);
    ds2 = z(3) - refs.s(2,i+1);

    n1 = refs.n(1,i+1);
    n2 = refs.n(2,i+1);

    s_c2e = ds1*n1 + ds2*n2;

    c(k) = s_c2e - track_half_width;
    k = k + 1;

    c(k) = -s_c2e - track_half_width;
    k = k + 1;

end

end
function [c, ceq] = mpc_constraints(U, z0, u_last, refs, params)

    U      = double(U(:));
    z      = double(z0(:));
    u_prev = double(u_last(:));

    N = params.N;

    v_min = params.v_min;
    v_max = params.v_max;
    track_half_width = params.track_half_width;
    dun_max = params.dun_max;
    ddeltan_max = params.ddeltan_max;

    c = zeros(8*N, 1);
    ceq = [];

    k = 1;

    for i = 1:N
        idx = 2*i - 1;
        un_i     = U(idx);
        deltan_i = U(idx+1);

        du1 = un_i     - u_prev(1);
        du2 = deltan_i - u_prev(2);

        % Rate constraints
        c(k) =  du1 - dun_max;       k = k + 1;
        c(k) = -du1 - dun_max;       k = k + 1;
        c(k) =  du2 - ddeltan_max;   k = k + 1;
        c(k) = -du2 - ddeltan_max;   k = k + 1;

        % Predict next state
        u_i = [un_i; deltan_i];
        z = mpc_model_step(z, u_i, params);

        % Speed constraints
        vr = z(1);
        c(k) = vr - v_max;           k = k + 1;
        c(k) = v_min - vr;           k = k + 1;

        % Lateral corridor constraint
        ds1 = z(2) - refs.s(1, i+1);
        ds2 = z(3) - refs.s(2, i+1);

        n1 = refs.n(1, i+1);
        n2 = refs.n(2, i+1);

        s_c2e = ds1*n1 + ds2*n2;

        c(k) =  s_c2e - track_half_width;  k = k + 1;
        c(k) = -s_c2e - track_half_width;  k = k + 1;

        u_prev(1) = un_i;
        u_prev(2) = deltan_i;
    end
end
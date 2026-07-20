function [c, ceq] = mpc_constraints(U, z0, u_last, refs, params)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

    U      = double(U(:));
    z0     = double(z0(:));
    u_last = double(u_last(:));
    
    N = params.N;
    
    % initialization
    z = z0(:);
    u_prev = u_last(:);
    
    c   = zeros(0,1);
    ceq = zeros(0,1);

    % get params
    v_min = params.v_min;
    v_max = params.v_max;
    track_half_width = params.track_half_width;
    dun_max = params.dun_max;
    ddeltan_max = params.ddeltan_max;

    for i = 1:N
        idx = 2*i - 1;
        u_i = U(idx:idx+1);

        %% Rate constraints
        du = u_i - u_prev;

        c = [c;
            du(1)  - dun_max;
            -du(1)  - dun_max;
            du(2)  - ddeltan_max;
            -du(2)  - ddeltan_max];

        %% Predict next state
        z = mpc_model_step(z, u_i, params);

        %% Speed constraints
        vr = z(1);

        c = [c;
            vr - v_max;
            v_min - vr];

        %% Track/lateral corridor constraint
        s_pred = [z(2); z(3)];

        % Use reference index i+1 because z is already propagated
        s_ref = refs.s(:, i+1);
        n_ref = refs.n(:, i+1);

        ds = s_pred - s_ref;
        s_c2e = ds.' * n_ref;

        c = [c;
            s_c2e - track_half_width;
            -s_c2e - track_half_width];
        
        %% save u_prev
        u_prev = u_i;
    end

    c   = double(c);
    ceq = double(ceq);

end
function J = mpc_cost( ...
    u_seq, state_0, u_last, u_delay, refs, params)

U = double(u_seq(:));
state_0 = double(state_0(:));
u_last = double(u_last(:));
u_delay = double(u_delay);

N = params.N;

nd = size(u_delay,2);

%% weights
q_c1  = params.q_c1;
q_c2  = params.q_c2;
q_psi = params.q_psi;
q_v   = params.q_v;

r_un     = params.r_un;
r_delta  = params.r_delta;
r_dun    = params.r_dun;
r_ddelta = params.r_ddelta;

z = state_0;
u_prev = u_last;

J = 0.0;

for i = 1:N+1

    %% ================================================================
    % State cost

    s_pred = z(2:3);

    s_ref   = refs.s(:,i);
    psi_ref = refs.psi(i);
    v_ref   = refs.v(i);

    t_ref = refs.t(:,i);
    n_ref = refs.n(:,i);

    ds = s_pred - s_ref;

    s_c1e = ds.' * t_ref;
    s_c2e = ds.' * n_ref;

    psi_e = wrap_pi(z(4) - psi_ref);
    v_e   = z(1) - v_ref;

    J = J ...
        + q_c1  * s_c1e^2 ...
        + q_c2  * s_c2e^2 ...
        + q_psi * psi_e^2 ...
        + q_v   * v_e^2;


    if i <= N

        %% ============================================================
        % Newly commanded MPC input

        idx = 2*i - 1;

        u_cmd = U(idx:idx+1);

        du = u_cmd - u_prev;

        J = J ...
            + r_un     * u_cmd(1)^2 ...
            + r_delta  * u_cmd(2)^2 ...
            + r_dun    * du(1)^2 ...
            + r_ddelta * du(2)^2;

        u_prev = u_cmd;


        %% ============================================================
        % Input that ACTUALLY affects vehicle

        if i <= nd

            % already-issued command
            u_applied = u_delay(:,i);

        else

            % optimized command starts affecting vehicle after nd steps
            j = i - nd;

            idx_applied = 2*j - 1;

            u_applied = U(idx_applied:idx_applied+1);

        end


        %% Vehicle propagation
        z = mpc_model_step(z, u_applied, params);

    end

end

J = double(J);

end

function ang = wrap_pi(ang)

ang = mod(ang + pi, 2*pi) - pi;

end
function J = mpc_cost(u_seq, state_0, u_last, refs, params)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

U = double(u_seq(:));
state_0 = double(state_0(:));
u_last = double(u_last(:));

N = params.N;

% Weights
q_c1 = params.q_c1;
q_c2 = params.q_c2;
q_psi = params.q_psi;
q_v = params.q_v;

r_un = params.r_un;
r_delta = params.r_delta;
r_dun = params.r_dun;
r_ddelta = params.r_ddelta;

% initialization
z = state_0(:); % variable for state predictions
u_prev = u_last(:);
J = double(0);

for i = 1:N+1 % TODO: why N+1?
    
    s_pred = [z(2); z(3)];
    
    % ref at step i
    s_ref   = refs.s(:, i);
    psi_ref = refs.psi(i);
    v_ref   = refs.v(i);

    t_ref = refs.t(:, i);
    n_ref = refs.n(:, i);

    % errors
    ds = s_pred - s_ref;

    s_c1e = ds.' * t_ref;
    s_c2e = ds.' * n_ref;

    psi_e = wrap_pi(z(4) - psi_ref);
    v_e   = z(1) - v_ref;

    %% Tracking cost
    J = J ...
        + q_c1  * s_c1e^2 ...
        + q_c2  * s_c2e^2 ...
        + q_psi * psi_e^2 ...
        + q_v   * v_e^2;

    %% Input and rate cost
    if i <= N
        idx = 2*i - 1;
        u_i = U(idx:idx+1); % u is applied N times

        un_i     = u_i(1);
        deltan_i = u_i(2);

        du = u_i - u_prev;

        J = J ...
            + r_un     * un_i^2 ...
            + r_delta  * deltan_i^2 ...
            + r_dun    * du(1)^2 ...
            + r_ddelta * du(2)^2;

        %% Predict next state
        z = mpc_model_step(z, u_i, params);

        u_prev = u_i;
    end

end

J = double(J);

end

%% ========================================================================
function ang = wrap_pi(ang)

ang = mod(ang + pi, 2*pi) - pi;

end
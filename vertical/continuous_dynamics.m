function [state_dot] = continuous_dynamics(state, u)

    P=get_parameters();
    un = u(1);
    deltan = u(2);
    d_0 = P.P_p_kd0 + P.P_p_kd1 * abs(un * deltan);

    if un > d_0
        dist = d_0;
    elseif un < -d_0
        dist = -d_0;
    else
        dist = un;
    end
    
    %%
    %dist=0;
    %%
    vr = state(3);
    psi = state(5);
    dvr = -vr / P.P_p_T + P.P_p_k / P.P_p_T * (un - dist);
    dsr1 = vr * cos(psi);
    dsr2 = vr * sin(psi);

    % Sichere Berechnung von dpsi
    eps_den = 1e-8;
    eps_deltan = 1e-3;
    tan_arg_limit = 1e2;

    if abs(deltan) < eps_deltan
        dpsi = 0;
    else
        denom = P.P_p_l + P.P_p_EG * (vr^2);% * sign(vr);
        denom_safe = max(abs(denom), eps_den) * sign(denom);

        tan_arg = (P.P_p_delta_max * deltan) / P.P_p_deltan_max;
        tan_arg_clamped = max(min(tan_arg, tan_arg_limit), -tan_arg_limit);
        dpsi = double((vr / denom_safe) * tan(tan_arg_clamped));
    end

    omega_dot = 0;
    dx = vr;
    state_dot = [dsr1; dsr2; dvr; 0; dpsi; omega_dot; dx];
end
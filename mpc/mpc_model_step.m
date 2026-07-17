function z_next = mpc_model_step(z, u, params)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
    
    z = double(z(:));
    u = double(u(:));
    
    Ta = params.Ta;
    
    zdot = vehicle_ode(z, u, params);
    z_next = z + Ta * zdot;
    % TODO: try rk4 later

    z_next(4) = wrap_pi(z_next(4));

end

%% ========================================================================
function zdot = vehicle_ode(z, u, params)

vr  = z(1);
sr1 = z(2); %#ok<NASGU>
sr2 = z(3); %#ok<NASGU>
psi = z(4);

un     = u(1);
deltan = u(2);

%% Parameters
Ta = double(params.Ta);
T  = double(params.T);
ku = double(params.ku);
l  = double(params.l);
EG = double(params.EG);

delta_max  = double(params.delta_max);
deltan_max = double(params.deltan_max);

%% Steering angle conversion
delta = delta_max * deltan / deltan_max;

%% Optional disturbance/deadzone model
use_disturbance = get_param_value(params, 'use_disturbance', false);

if use_disturbance
    kd0 = get_param_value(params, 'kd0', 0.04);
    kd1 = get_param_value(params, 'kd1', 0.15);

    d0 = kd0 + kd1 * abs(un) * abs(deltan);

    if un > d0
        d = d0;
    elseif un < -d0
        d = -d0;
    else
        d = un;
    end
else
    d = 0.0;
end

%% Longitudinal dynamics
vr_dot = -vr/T + (ku/T) * (un - d);

%% Position dynamics
sr1_dot = vr * cos(psi);
sr2_dot = vr * sin(psi);

%% Yaw dynamics with Eigenlenkgradient
den = l + EG * vr^2 * sign_smooth(vr);
den = max(abs(den), 1e-6) * sign_smooth_nonzero(den);

psi_dot = (vr / den) * tan(delta);

zdot = [vr_dot;
        sr1_dot;
        sr2_dot;
        psi_dot];

end

%% ========================================================================
function val = get_param_value(params, name, default_value)

if isfield(params, name)
    val = params.(name);
else
    val = default_value;
end

end

%% ========================================================================
function y = sign_smooth(x)

if x > 0
    y = 1;
elseif x < 0
    y = -1;
else
    y = 0;
end

end

%% ========================================================================
function y = sign_smooth_nonzero(x)

if x >= 0
    y = 1;
else
    y = -1;
end

end

%% ========================================================================
function ang = wrap_pi(ang)

ang = mod(ang + pi, 2*pi) - pi;

end
%% Longitudinal speed control
% ramp speed reference
P_p_aMax = 1;
P_p_vRef = 0.2;

%% LQR params
params_lqr.N  = 20;          % prediction horizon
params_lqr.Ta = 25e-3;       % sampling time [s]
params_lqr.v_ref = P_p_vRef;     % desired speed [m/s], start conservative
params_lqr.Tt    = 100e-3;   % delay/lookahead time [s]

params_lqr.l  = 32.5e-3;     % wheelbase [m]

Q = diag([8000, 100]);
R = 1;
P = diag([8000, 100]);

params_lqr.weights = struct('Q', Q, 'R', R, 'P', P);

disp(params_lqr)             % check

%% Baseline params
P_p_Tw_steering = 0.175;
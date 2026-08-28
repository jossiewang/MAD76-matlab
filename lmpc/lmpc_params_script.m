% lmpc_params_lmpc

lmpc_params.Tt = P_p_Tt;
lmpc_params.v_ref = 0.2;
lmpc_params.Ta = 0.025;
lmpc_params.N  = 20;

lmpc_params.Nd = round( ...
    lmpc_params.Tt / lmpc_params.Ta);

lmpc_params.u_init = [
    0;
    0
    ];

assert(abs( ...
    lmpc_params.Nd*lmpc_params.Ta ...
    - lmpc_params.Tt) < 1e-9);

%% measured steady-state pedal-speed map
lmpc_params.un_table = [
    0.02
    0.03
    0.04
    0.05
    0.06
    0.07
    0.10
    0.12
    0.15
];

lmpc_params.vss_table = [
    0
    0.0377559
    0.0753484
    0.140501
    0.190577
    0.232501
    0.333488
    0.363376
    0.42811
];

%% vehicle model
lmpc_params.l  = P_p_l;
lmpc_params.T  = 0.0536;
lmpc_params.ku = P_p_k;

%% MPC weights
v_scale   = 0.10;   % [m/s]
sc1_scale = 0.05;   % [m]
sc2_scale = 0.06;   % [m]
psi_scale = 0.50;   % [rad]

% lmpc_params.Q = diag([1, 1, 1000, 10]);
lmpc_params.Q = diag([
    5    / v_scale^2
    0.2  / sc1_scale^2
    10   / sc2_scale^2
    5    / psi_scale^2
    ]);
lmpc_params.R = diag([1, 1]);
lmpc_params.Rdu = diag([100, 100]);
lmpc_params.P = lmpc_params.Q;

%% actuator constraints
lmpc_params.u_min = [-0.2; -deg2rad(22)]; % min is actually not used in code, symmetric assumed
lmpc_params.u_max = [ 0.2;  deg2rad(22)];

%% conversion
lmpc_params.delta_norm_gain = 0.93 / deg2rad(22);

%% rate constraints
lmpc_params.du_min = [-0.0936; -1.14/lmpc_params.delta_norm_gain];
lmpc_params.du_max = [0.0936; 1.14/lmpc_params.delta_norm_gain];
% 
% %% state constraints
% lmpc_params.x_min = [...];
% lmpc_params.x_max = [...];
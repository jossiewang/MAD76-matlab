% lmpc_params_lmpc

lmpc_params.Tt = P_p_Tt;
lmpc_params.v_ref = 0.5;
lmpc_params.Ta = 0.025;
lmpc_params.N  = 20;

%% vehicle model
lmpc_params.l  = P_p_l;
lmpc_params.T  = P_p_T;
lmpc_params.ku = P_p_k;

%% MPC weights
lmpc_params.Q = diag([1, 1, 1000, 10]);
lmpc_params.R = diag([1, 1]);
lmpc_params.P = lmpc_params.Q;

%% actuator constraints
lmpc_params.u_min = [-0.2; -deg2rad(22)];
lmpc_params.u_max = [ 0.2;  deg2rad(22)];

%% rate constraints
lmpc_params.du_min = [-0.0936; -1.14];
lmpc_params.du_max = [0.0936; 1.14];
% 
% %% state constraints
% lmpc_params.x_min = [...];
% lmpc_params.x_max = [...];

%% conversion
lmpc_params.delta_norm_gain = 0.93 / deg2rad(22);

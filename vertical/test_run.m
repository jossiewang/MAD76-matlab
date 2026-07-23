%% initialize and load model
clearvars;
clc;
close all;

rng(42);
addpath('../BO');
addpath('../mpc');
model = "s06_sig_template";
load_system(model);

s06_data;
mpc_params_script; % load it once here, and then change 'params' during BO process

% %% Cost weights (hand-tuned best)
% params.q_c1  = 0.0;      % longitudinal/tangential error
% params.q_c2  = 1000.0;    % lateral error
% params.q_psi = 20.0;     % yaw error
% params.q_v   = 5.0;      % speed error
% 
% params.r_un     = 0.1;   % motor effort
% params.r_delta  = 0.1;   % steering effort
% params.r_dun    = 2.0;   % motor rate change
% params.r_ddelta = 2.0;   % steering rate change
% J1 = run_mpc_simulation(model)
% 
% %% Cost weights (BO on IAE)
% params.q_c1  = 0.0;      % longitudinal/tangential error
% params.q_c2  = 706.4672;    % lateral error
% params.q_psi = 0.01;     % yaw error
% params.q_v   = 79.92;      % speed error
% 
% params.r_un     = 50;   % motor effort
% params.r_delta  = 0.005;   % steering effort
% params.r_dun    = 0.1;   % motor rate change
% params.r_ddelta = 0.05;   % steering rate change
% J2 = run_mpc_simulation(model)
% 
% %% Cost weights (BO on RMSE)
% params.q_c2  = 850.3888;    % lateral error
% params.q_psi = 0.1;     % yaw error
% params.q_v   = 7.4557;      % speed error
% 
% params.r_un     = 10;   % motor effort
% params.r_delta  = 0.01;   % steering effort
% params.r_dun    = 5.8332;   % motor rate change
% params.r_ddelta = 0.1;   % steering rate change
% J3 = run_mpc_simulation(model)


%% Cost weights (BO on laptime)
params.q_c2  = 479.2209;    % lateral error
params.q_psi = 4.7846;     % yaw error
params.q_v   = 28.4938;      % speed error

params.r_un     = 0.1;   % motor effort
params.r_delta  = 1;   % steering effort
params.r_dun    = 0.6083;   % motor rate change
params.r_ddelta = 2.8903;   % steering rate change
J4 = run_mpc_simulation(model)
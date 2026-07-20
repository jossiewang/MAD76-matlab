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
J1 = run_mpc_simulation(model)

%% Cost weights
params.q_c1  = 0.0;      % longitudinal/tangential error
params.q_c2  = 100.0;    % lateral error
params.q_psi = 10.0;     % yaw error
params.q_v   = 10.0;      % speed error

params.r_un     = 0.1;   % motor effort
params.r_delta  = 0.1;   % steering effort
params.r_dun    = 2.0;   % motor rate change
params.r_ddelta = 2.0;   % steering rate change
J2 = run_mpc_simulation(model)
%% test run sim
%% initialize and load model
clearvars;
clc;
close all;

rng(42);
addpath('../BO');
addpath('../lqr');
model = "c71_car0_template";
load_system(model);

s06_data;
lqr_params_script; % load it once here, and then change 'params' during BO process

% J = run_mpc_simulation(model)

%% Define BO Parameter Space

% first, test BO with simpler case: LQR gain
num_BOparams = 3;
lb = [100, 0.1, 0.1]; % Lower bounds for q_c2, q_psi, r_delta
ub = [10000, 300, 100]; % Upper bounds for q_c2, q_psi, r_delta

%% Initial Data
X = rand(2,num_BOparams) .* (ub - lb) + lb; % Initial arandom parameter samples
Y = zeros(size(X,1),1);
% all_trajectories = cell(size(X,1),1);
% all_controls = cell(size(X,1),1);
% 
% Evaluate the initial points
for i = 1:size(X,1)
    % params.q_c2  = X(i,1);
    % params.q_psi = X(i,2);
    % params.q_v   = X(i,3);
    % params.r_un     = X(i,4);
    % params.r_delta  = X(i,5);
    % params.r_dun    = X(i,6);
    % params.r_ddelta = X(i,7);
    Q = diag([X(i,1), X(i,2)]);
    R = X(i,3);
    P = diag([X(i,1), X(i,2)]);

    params_lqr.weights = struct('Q', Q, 'R', R, 'P', P);
    
    Y(i) = run_mpc_simulation(model);
end

fprintf('=== Starting Bayesian Optimization for MPC Tuning ===\n\n');

%% Initial Data
fprintf('Initial points evaluated:\n');
fprintf('params = [%.4f, %.4f, %.4f], cost = %.4f\n', X(1,:), Y(1));
fprintf('params = [%.4f, %.4f, %.4f], cost = %.4f\n\n', X(2,:), Y(2));
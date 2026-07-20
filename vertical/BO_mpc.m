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

%% Define BO Parameter Space

% first, test BO with simpler case: LQR gain
num_BOparams = 7;
lb = [1,    0.1, 1,   0.01, 0.01, 0.1, 0.1]; % Lower bounds for q_c2, q_psi, q_v, r_un, r_delta, r_dun, r_ddelta
ub = [1000, 100, 100, 10,   10,   10,  10]; % Upper bounds for q_c2, q_psi, q_v, r_un, r_delta, r_dun, r_ddelta

%% Initial Data
X = rand(2,num_BOparams) .* (ub - lb) + lb; % Initial random parameter samples
Y = zeros(size(X,1),1);
% all_trajectories = cell(size(X,1),1);
% all_controls = cell(size(X,1),1);
% 
% Evaluate the initial points
for i = 1:size(X,1)
    params.q_c2  = X(i,1);
    params.q_psi = X(i,2);
    params.q_v   = X(i,3);
    params.r_un     = X(i,4);
    params.r_delta  = X(i,5);
    params.r_dun    = X(i,6);
    params.r_ddelta = X(i,7);
    
    Y(i) = run_mpc_simulation(model);
end

fprintf('=== Starting Bayesian Optimization for MPC Tuning ===\n\n');

%% Initial Data
fprintf('Initial points evaluated:\n');
fprintf('params = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f], cost = %.4f\n', X(1,:), Y(1));
fprintf('params = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f], cost = %.4f\n\n', X(2,:), Y(2));

%% Fit Gaussian Process Model
gpModel = fitrgp(X, Y, 'KernelFunction', 'squaredexponential', ...
    'Standardize', true, ...
    'ConstantSigma', true, ...
    'SigmaLowerBound', 1e-8, ...
    'Sigma', 1e-8+1e-6);

%% Bayesian Optimization Loop
num_iterations = 70; % Number of BO iterations
num_starts = 10; % Number of multistart points for EI optimization

% Initialize cost tracking
best_cost_history = zeros(num_iterations, 1);
iteration_cost = zeros(num_iterations, 1);
best_so_far = Inf;

for iter = 1:num_iterations
    fprintf('Iteration %d/%d:\n', iter, num_iterations);

    % Define acquisition function (Expected Improvement)
    acq_fun = @(params) double(expected_improvement(params, gpModel, Y)); % required double

    % Optimize acquisition function
    x_next = optimize_acquisition(acq_fun, lb, ub, num_starts);

    % Evaluate the new parameter set
    params.q_c2  = x_next(1);
    params.q_psi = x_next(2);
    params.q_v   = x_next(3);
    params.r_un     = x_next(4);
    params.r_delta  = x_next(5);
    params.r_dun    = x_next(6);
    params.r_ddelta = x_next(7);

    y_next = run_mpc_simulation(model);

    % Store data
    X = [X; x_next'];
    Y = [Y; y_next];
    % all_trajectories{end+1} = state_trajectory;
    % all_controls{end+1} = control_input;

    % Update cost tracking
    best_so_far = min(best_so_far, y_next);
    best_cost_history(iter) = best_so_far;
    iteration_cost(iter) = y_next;

    % Print progress
    fprintf('Current params: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f], cost: %.4f\n', x_next, y_next);
    fprintf('Best cost so far: %.4f\n\n', min(Y));

    % Update GP Model
    gpModel = fitrgp(X, Y, 'KernelFunction', 'squaredexponential', ...
        'Standardize', true, ...
        'ConstantSigma', true, ...
        'SigmaLowerBound', 1e-8, ...
        'Sigma', 1e-8+1e-6);
end

% Print final results
[best_cost, best_idx] = min(Y);
improvement = (Y(1) - best_cost) / Y(1) * 100;

fprintf('\n=== Final Results ===\n');
fprintf('Initial best: %.4f\n', Y(1));
fprintf('Final best:   %.4f\n', best_cost);
fprintf('Best params:  [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', X(best_idx,:));
fprintf('===================\n\n');

% %% Find Best Parameters
% [~, best_idx] = min(Y);
% best_trajectory = all_trajectories{best_idx};
% best_control = all_controls{best_idx};
% initial_trajectory = all_trajectories{1};
% initial_control = all_controls{1};

% %% Visualization
% visualize_results(all_trajectories, all_controls, initial_trajectory, best_trajectory, initial_control, best_control);

%% Plot Cost Progress
figure;
hold on;
plot(1:num_iterations, best_cost_history, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 6);
scatter(1:num_iterations, iteration_cost, 100, 'r', 'filled');
xlabel('Iteration');
ylabel('Closed-loop Cost');
title('MPC Parameter Optimization Progress');
grid on;
legend({'Best Cost So Far', 'Iteration Cost'}, 'Location', 'best');
hold off;

disp('Bayesian Optimization Finished!');
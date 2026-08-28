%% Initialize
clearvars;
clc;
close all;

rng(42);
addpath('../BO');
addpath('../lqr');
model = "c71_car0_template";
load_system(model);

s06_data;
lqr_params_script;

%% BO Parameter Space
num_BOparams = 3;

lb = [100,   0.1,  0.01];
ub = [10000, 1000, 100];

num_initial_points = 2;
num_iterations = 30;
num_starts = 10;

%% Checkpoint file
checkpointFile = "BO_LQR_checkpoint.mat";

%% Load existing run if possible
if isfile(checkpointFile)

    fprintf('\n========================================\n');
    fprintf('Existing BO checkpoint found.\n');
    fprintf('Loading previous experiment...\n');
    fprintf('========================================\n\n');

    S = load(checkpointFile);

    X = S.X;
    Y = S.Y;

    if isfield(S, 'pending_x')
        pending_x = S.pending_x;
    else
        pending_x = [];
    end

    fprintf('Recovered %d completed hardware runs.\n', size(X,1));

    if ~isempty(pending_x)
        fprintf('MATLAB previously crashed while testing:\n');
        fprintf('  [%.6g %.6g %.6g]\n', pending_x);
    end

else

    fprintf('\nStarting new BO experiment.\n');

    X = zeros(0, num_BOparams);
    Y = zeros(0, 1);
    pending_x = [];

end

%% Initial experiments

while size(X,1) < num_initial_points

    runNumber = size(X,1) + 1;

    fprintf('\n========================================\n');
    fprintf('Initial hardware run %d/%d\n', ...
        runNumber, num_initial_points);
    fprintf('========================================\n');

    %% Generate point
    x_next = rand(1,num_BOparams) .* (ub-lb) + lb;

    %% Save BEFORE hardware experiment
    pending_x = x_next;
    save_BO_checkpoint(checkpointFile, X, Y, pending_x);

    fprintf('Testing parameters:\n');
    fprintf('q_c2    = %.6g\n', x_next(1));
    fprintf('q_psi   = %.6g\n', x_next(2));
    fprintf('r_delta = %.6g\n', x_next(3));

    %% Set LQR parameters
    Q = diag([x_next(1), x_next(2)]);
    R = x_next(3);
    P = Q;

    params_lqr.weights = struct( ...
        'Q', Q, ...
        'R', R, ...
        'P', P);

    %% Hardware run
    y_next = run_mpc_simulation(model);

    %% Store successful result
    X = [X; x_next];
    Y = [Y; y_next];

    pending_x = [];

    %% SAVE IMMEDIATELY
    save_BO_checkpoint(checkpointFile, X, Y, pending_x);

    fprintf('Cost = %.6g\n', y_next);

end

%% Fit Gaussian Process Model
gpModel = fitrgp(X, Y, 'KernelFunction', 'squaredexponential', ...
    'Standardize', true, ...
    'ConstantSigma', true, ...
    'SigmaLowerBound', 1e-8, ...
    'Sigma', 1e-8+1e-6);

%% Determine how many BO iterations have already been completed

num_completed_BO = size(X,1) - num_initial_points;

fprintf('\n========================================\n');
fprintf('Starting/resuming Bayesian Optimization\n');
fprintf('Completed BO iterations: %d/%d\n', ...
    num_completed_BO, num_iterations);
fprintf('========================================\n\n');

%% Cost history
best_cost_history = nan(num_iterations,1);
iteration_cost    = nan(num_iterations,1);

% Reconstruct previous history if resuming
if num_completed_BO > 0

    for k = 1:num_completed_BO

        idx = num_initial_points + k;

        iteration_cost(k) = Y(idx);
        best_cost_history(k) = min(Y(1:idx));

    end

end

%% Bayesian Optimization

for iter = num_completed_BO+1 : num_iterations

    fprintf('\n========================================\n');
    fprintf('BO iteration %d/%d\n', iter, num_iterations);
    fprintf('========================================\n');

    %% Expected Improvement
    acq_fun = @(x) double( ...
        expected_improvement(x, gpModel, Y));

    %% Find next query
    x_next = optimize_acquisition( ...
        acq_fun, lb, ub, num_starts);

    x_next = x_next(:).';

    fprintf('Next parameters:\n');
    fprintf('q_c2    = %.6g\n', x_next(1));
    fprintf('q_psi   = %.6g\n', x_next(2));
    fprintf('r_delta = %.6g\n', x_next(3));

    %% ------------------------------------------------------------
    % IMPORTANT:
    % save parameters BEFORE hardware experiment
    % ------------------------------------------------------------

    pending_x = x_next;

    save_BO_checkpoint( ...
        checkpointFile, X, Y, pending_x);

    %% Set controller parameters

    Q = diag([x_next(1), x_next(2)]);
    R = x_next(3);
    P = Q;

    params_lqr.weights = struct( ...
        'Q', Q, ...
        'R', R, ...
        'P', P);

    %% ------------------------------------------------------------
    % Hardware experiment
    % ------------------------------------------------------------

    y_next = run_mpc_simulation(model);

    %% ------------------------------------------------------------
    % Experiment completed successfully
    % ------------------------------------------------------------

    X = [X; x_next];
    Y = [Y; y_next];

    pending_x = [];

    %% SAVE IMMEDIATELY

    save_BO_checkpoint( ...
        checkpointFile, X, Y, pending_x);

    %% Update history

    iteration_cost(iter) = y_next;
    best_cost_history(iter) = min(Y);

    fprintf('Cost: %.6g\n', y_next);
    fprintf('Best cost so far: %.6g\n', min(Y));

    %% Refit GP

    gpModel = fitrgp(X, Y, ...
        'KernelFunction', 'squaredexponential', ...
        'Standardize', true, ...
        'ConstantSigma', true, ...
        'SigmaLowerBound', 1e-8, ...
        'Sigma', 1e-8 + 1e-6);

end

% Print final results
[best_cost, best_idx] = min(Y);
improvement = (Y(1) - best_cost) / Y(1) * 100;

fprintf('\n=== Final Results ===\n');
fprintf('Initial best: %.4f\n', Y(1));
fprintf('Final best:   %.4f\n', best_cost);
fprintf('Best params:  [%.4f, %.4f, %.4f]\n', X(best_idx,:));
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

%% plot query point
figure;

scatter3(X(:,1), X(:,2), X(:,3), ...
    70, Y, 'filled');

xlabel('X_1');
ylabel('X_2');
zlabel('X_3');

title('GPR Samples');

cb = colorbar;
cb.Label.String = 'Y';

grid on;
view(3);

%% function
function save_BO_checkpoint(filename, X, Y, pending_x)

% Save BO experiment state.
%
% pending_x:
%   []       -> no experiment currently running
%   [1 x n]  -> parameters of experiment currently running

rngState = rng;

tempFile = filename + ".tmp";

save(tempFile, ...
    'X', ...
    'Y', ...
    'pending_x', ...
    'rngState');

% Replace previous checkpoint only after save completed
movefile(tempFile, filename, 'f');

end
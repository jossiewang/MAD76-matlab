clear;
clc;

%% Parameters

params.N  = 20;
params.Ta = 0.025;

params.l  = 32.5e-3;

% Replace these with your actual identified values
params.T  = 0.0536;
params.ku = 3.3058;

N  = params.N;
nx = 4;
nu = 2;

%% Artificial reference trajectory

% refs.v = 0.3 * ones(N,1);
% 
% % Start with straight driving
% refs.kappa = zeros(N,1);
% 
% refs.delta = atan(params.l * refs.kappa);

refs.v = linspace(0.20, 0.40, N+1).';

refs.kappa = linspace(0, 8, N+1).';

refs.delta = atan(params.l * refs.kappa);

%% Build LTV model

[Ad, Bd] = lmpc_build_ltv_model(refs, params);

%% Build prediction matrices

[Phi, Gamma] = ...
    lmpc_build_prediction_matrices(Ad, Bd, params);

%% Initial error state

x0 = [
    0.03;      % velocity error [m/s]
    0.01;      % longitudinal path error [m]
    0.02;      % lateral error [m]
    0.10       % heading error [rad]
    ];

% x0 = [ % forced saturation
%     0;
%     0;
%     0.10;
%     0.7
%     ];

%% Arbitrary test input sequence

rng(1);

U = 0.05 * randn(nu*N,1);

%% ==============================================================
% Method 1:
% stacked prediction matrix
% ===============================================================

X_matrix = Phi*x0 + Gamma*U;

%% ==============================================================
% Method 2:
% direct recursive simulation
% ===============================================================

X_direct = zeros(nx*N,1);

x = x0;

for i = 1:N

    rows_u = (i-1)*nu + (1:nu);
    ui = U(rows_u);

    x = Ad(:,:,i)*x + Bd(:,:,i)*ui;

    rows_x = (i-1)*nx + (1:nx);

    X_direct(rows_x) = x;

end

%% Compare

err = X_matrix - X_direct;

fprintf('Maximum absolute prediction error = %.3e\n', ...
    max(abs(err)));

fprintf('Norm of prediction error = %.3e\n', ...
    norm(err));

%% Expected:
%
% errors should be approximately machine precision:
%
% ~1e-15 to 1e-13
%

%% test on building cost
params.Q = diag([1, 1, 1000, 10]);
params.R = diag([1, 1]);
params.P = params.Q;

%% Build cost
[H, f] = lmpc_build_cost(x0, Phi, Gamma, params);

%% Build Qbar and Rbar again for verification
Qbar = blkdiag( ...
    kron(eye(params.N-1), params.Q), ...
    params.P);

Rbar = kron(eye(params.N), params.R);

%% Predicted states
X = Phi*x0 + Gamma*U;

%% Original MPC cost
J_direct = X' * Qbar * X + ...
    U' * Rbar * U;

%% Constant term omitted by quadprog
c = x0' * Phi' * Qbar * Phi * x0;

%% QP-form cost
J_qp = 0.5 * U' * H * U + ...
    f' * U + c;

fprintf('J_direct = %.15g\n', J_direct);
fprintf('J_qp     = %.15g\n', J_qp);
fprintf('Cost error = %.3e\n', ...
    abs(J_direct - J_qp));


%% test the constraint transformation explicitly

params.u_min = [
    -0.2;
    -deg2rad(22)
    ];

params.u_max = [
    0.2;
    deg2rad(22)
    ];

refs.u_ref = zeros(2, N);

refs.u_ref(1,:) = ...
    (refs.v(1:N) / params.ku).';

refs.u_ref(2,:) = ...
    refs.delta(1:N).';

[H, f, lb, ub] = ...
    lmpc_build_qp(x0, Phi, Gamma, refs, params);

options = optimoptions( ...
    'quadprog', ...
    'Display', 'none');

[Uopt, Jopt, exitflag, output] = ...
    quadprog( ...
    H, ...
    f, ...
    [], [], ...
    [], [], ...
    lb, ub, ...
    [], ...
    options);

Uphys = zeros(2, params.N);

for i = 1:params.N

    rows = (i-1)*2 + (1:2);

    ue = Uopt(rows);

    Uphys(:,i) = refs.u_ref(:,i) + ue;

end

fprintf('Minimum physical pedal    = %.6f\n', ...
    min(Uphys(1,:)));

fprintf('Maximum physical pedal    = %.6f\n', ...
    max(Uphys(1,:)));

fprintf('Minimum physical steering = %.6f rad\n', ...
    min(Uphys(2,:)));

fprintf('Maximum physical steering = %.6f rad\n', ...
    max(Uphys(2,:)));

%% plots
%% ==============================================================
% Plot optimized predicted states
% ==============================================================

% Predicted optimal error-state sequence:
%
% Xopt = [x_1;
%         x_2;
%         ...
%         x_N]

Xopt = Phi*x0 + Gamma*Uopt;

% Convert stacked vector into nx x N matrix
Xopt_mat = reshape(Xopt, nx, N);

% Include initial state x0 so that we have N+1 samples
Xe = [x0, Xopt_mat];

% Time vector
t = (0:N) * params.Ta;

%% Reference quantities

v_ref_plot = refs.v(1:N+1).';

% Error-state references are zero
zero_ref = zeros(1, N+1);

%% Recover predicted physical velocity

v_pred = v_ref_plot + Xe(1,:);

%% Plot

figure('Position', [100 100 1000 800]);

tiledlayout(4,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');

%% Velocity
nexttile;

plot(t, v_pred, ...
    'LineWidth', 1.6);

hold on;

plot(t, v_ref_plot, '--', ...
    'LineWidth', 1.4);

grid on;

ylabel('v [m/s]');

legend('Predicted', 'Reference', ...
    'Location','best');

title('LMPC Predicted States');


%% Longitudinal path error
nexttile;

plot(t, Xe(2,:), ...
    'LineWidth', 1.6);

hold on;

plot(t, zero_ref, '--', ...
    'LineWidth', 1.2);

grid on;

ylabel('s_{c1e} [m]');

legend('Predicted', 'Reference', ...
    'Location','best');


%% Lateral path error
nexttile;

plot(t, Xe(3,:), ...
    'LineWidth', 1.6);

hold on;

plot(t, zero_ref, '--', ...
    'LineWidth', 1.2);

grid on;

ylabel('s_{c2e} [m]');

legend('Predicted', 'Reference', ...
    'Location','best');


%% Heading error
nexttile;

plot(t, Xe(4,:), ...
    'LineWidth', 1.6);

hold on;

plot(t, zero_ref, '--', ...
    'LineWidth', 1.2);

grid on;

ylabel('\psi_e [rad]');
xlabel('Prediction time [s]');

legend('Predicted', 'Reference', ...
    'Location','best');

%% ==============================================================
% Plot optimized physical inputs
% ==============================================================

t_u = (0:N-1) * params.Ta;

figure('Position', [150 150 1000 500]);

tiledlayout(2,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');


%% Pedal
nexttile;

stairs(t_u, Uphys(1,:), ...
    'LineWidth', 1.6);

hold on;

stairs(t_u, refs.u_ref(1,:), '--', ...
    'LineWidth', 1.3);

yline(params.u_max(1), ':');
yline(params.u_min(1), ':');

grid on;

ylabel('Pedal');

legend( ...
    'Optimized physical input', ...
    'Feedforward reference', ...
    'Location','best');

title('LMPC Optimized Inputs');


%% Steering
nexttile;

stairs(t_u, rad2deg(Uphys(2,:)), ...
    'LineWidth', 1.6);

hold on;

stairs(t_u, rad2deg(refs.u_ref(2,:)), '--', ...
    'LineWidth', 1.3);

yline(rad2deg(params.u_max(2)), ':');
yline(rad2deg(params.u_min(2)), ':');

grid on;

ylabel('\delta [deg]');
xlabel('Prediction time [s]');

legend( ...
    'Optimized physical input', ...
    'Feedforward reference', ...
    'Location','best');
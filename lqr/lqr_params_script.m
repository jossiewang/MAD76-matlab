%% params_lqr used in mpc

%% Horizon and sampling
params_lqr.N  = 20;          % prediction horizon
params_lqr.Ta = 25e-3;       % sampling time [s]

%% Reference generator
params_lqr.v_ref = P_p_vRef;     % desired speed [m/s], start conservative
params_lqr.Tt    = 100e-3;   % delay/lookahead time [s]
% params_lqr.periodic = true;  % true for closed race track

%% Input limits from MAD76 slides
% params_lqr.un_min = -0.2;    % motor signal minimum
% params_lqr.un_max =  0.2;    % motor signal maximum
% 
% params_lqr.deltan_min = -0.93;   % steering signal minimum
% params_lqr.deltan_max =  0.93;   % steering signal maximum

%% Vehicle model parameters
% params_lqr.T  = 120e-3;      % longitudinal PT1 time constant [s]
% params_lqr.ku = 3.4;         % gain from motor signal to speed [m/s]
% params_lqr.Tt_vehicle = 100e-3; % vehicle/remote-control delay [s], optional
% 
params_lqr.l  = 32.5e-3;     % wheelbase [m]
% params_lqr.delta_max = 22*pi/180;  % maximum steering angle [rad]
% params_lqr.EG = 0.01;        % Eigenlenkgradient / understeering parameter

%% Disturbance / deadzone model
% params_lqr.use_disturbance = false;  % start false for easier MPC debugging
% params_lqr.kd0 = 0.04;       % static friction / deadzone
% params_lqr.kd1 = 0.15;       % curve resistance due to steering

%% Numerical integration
% params_lqr.integrator = 'euler';     % 'euler' first, later try 'rk4'

%% weight matrices
% b
% Q = diag([500, 500, 1]);
% R = 1;
% P = diag([1000, 1000, 1]);

% best lap time (hand), vRef=0.4
% best tracking (hand), vRef=0.3
% Q = diag([8000, 8000, 10]);
% R = 5;
% P = diag([8000, 8000, 10]);

Q = diag([8000, 100]);
R = 1;
P = diag([8000, 100]);

params_lqr.weights = struct('Q', Q, 'R', R, 'P', P);

%% Optional constraints
% params_lqr.v_min = 0.0;      % minimum speed [m/s]
% params_lqr.v_max = 0.5;      % maximum speed [m/s]
% 
% params_lqr.dun_max     = 0.0936;  % max motor change per sample, 120% baseline speed control
% params_lqr.ddeltan_max = 1.14;  % max steering change per sample, suppose 3 rev/sec

% Optional track corridor constraint.
% Comment this out first if fmincon has difficulty.
% params_lqr.track_half_width = 0.06;  % [m]

%% fmincon options
% params_lqr.fmincon_options = optimoptions('fmincon', ...
%     'Algorithm', 'sqp', ...
%     'Display', 'iter-detailed', ...
%     'MaxIterations', 13, ...
%     'MaxFunctionEvaluations', 250, ...
%     'ConstraintTolerance', 1e-4, ...
%     'OptimalityTolerance', 1e-3, ...
%     'StepTolerance', 1e-6);

%% Important check
disp(params_lqr)
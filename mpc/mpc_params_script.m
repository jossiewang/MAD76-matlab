%% params used in mpc

%% Horizon and sampling
params.N  = 20;          % prediction horizon
params.Ta = 25e-3;       % sampling time [s]

%% Reference generator
params.v_ref = 0.4;     % desired speed [m/s], start conservative
params.Tt    = 100e-3;   % delay/lookahead time [s]
params.periodic = true;  % true for closed race track

%% Input limits from MAD76 slides
params.un_min = -0.2;    % motor signal minimum
params.un_max =  0.2;    % motor signal maximum

params.deltan_min = -0.93;   % steering signal minimum
params.deltan_max =  0.93;   % steering signal maximum

%% Vehicle model parameters
params.T  = 120e-3;      % longitudinal PT1 time constant [s]
params.ku = 3.4;         % gain from motor signal to speed [m/s]
params.Tt_vehicle = 100e-3; % vehicle/remote-control delay [s], optional

params.l  = 32.5e-3;     % wheelbase [m]
params.delta_max = 22*pi/180;  % maximum steering angle [rad]
params.EG = 0.01;        % Eigenlenkgradient / understeering parameter

%% Disturbance / deadzone model
params.use_disturbance = false;  % start false for easier MPC debugging
params.kd0 = 0.04;       % static friction / deadzone
params.kd1 = 0.15;       % curve resistance due to steering

%% Numerical integration
params.integrator = 'euler';     % 'euler' first, later try 'rk4'

%% Cost weights
params.q_c1  = 0.0;      % longitudinal/tangential error
params.q_c2  = 200.0;    % lateral error
params.q_psi = 20.0;     % yaw error
params.q_v   = 5.0;      % speed error

params.r_un     = 0.1;   % motor effort
params.r_delta  = 0.1;   % steering effort
params.r_dun    = 2.0;   % motor rate change
params.r_ddelta = 2.0;   % steering rate change

%% Optional constraints
params.v_min = 0.0;      % minimum speed [m/s]
params.v_max = 0.5;      % maximum speed [m/s]

params.dun_max     = 0.03;  % max motor change per sample
params.ddeltan_max = 0.15;  % max steering change per sample

% Optional track corridor constraint.
% Comment this out first if fmincon has difficulty.
params.track_half_width = 0.08;  % [m]

%% fmincon options
params.fmincon_options = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...
    'Display', 'none', ...
    'MaxIterations', 50, ...
    'MaxFunctionEvaluations', 3000, ...
    'ConstraintTolerance', 1e-4, ...
    'OptimalityTolerance', 1e-4, ...
    'StepTolerance', 1e-6);

%% Important check
disp(params)
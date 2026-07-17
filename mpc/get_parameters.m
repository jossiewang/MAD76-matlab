function P = get_parameters()
%% Mini-Auto-Drive MAD76 - Vehicle Dynamics Parameters
%% Converted to a function for MATLAB Coder compatibility

% Vehicle Parameters
P.P_p_m = 13e-3;              % mass [kg]
P.P_p_J = 3e-6;               % moment of inertia (yaw) [kg*m^2]

% Longitudinal Dynamics
P.P_p_k = 3.4;                % gain [m/s]
P.P_p_T = 120e-3;             % time constant [s]
P.P_p_uTt = 100e-3;           % input dead time [s]
P.P_p_un_max = 0.2;           % maximum motor input signal [1]
P.P_p_un_min = -0.2;% minimum motor input signal [1]
P.P_p_kd0 = 0.04;             % disturbance const. static friction []
P.P_p_kd1 = 0.15;             % disturbance const. cornering resistance []

P.P_p_un_cmd_en = false;      % enable CmdHalt, CmdForward, CmdReverse

% Longitudinal disturbance
P.P_p_un_friction_kd0 = 0.04; % dead zone
P.P_p_un_friction_kd1 = 0.15; % curve resistance factor

% Lateral Dynamics
P.P_p_delta_max = 22/180*pi;  % maximum steering angle [rad]
P.P_p_delta_min = -22/180*pi;
P.P_p_delta_bias = 0;         % bias in steering [1]
P.P_p_deltan_max = 0.93;      % maximum normalized steering angle [1]
P.P_p_deltan_min = -0.93;
P.P_p_EG = 0.02;              % Eigenlenkgradient [1]
P.P_p_l = 32.5e-3;            % wheel base [m]
P.P_p_lr = 0.5 * 32.5e-3;     % distance to rear axle [m]
P.P_p_lf = 0.5 * 32.5e-3;     % distance to front axle [m]
P.P_p_delta_Tt = 100e-3;   % servo dead time [s]

% Geometry
P.P_p_rearaxle_1 = 0.5 * 32.5e-3;  % distance between IMU and rear axle [m]
P.P_p_frontaxle_1 = 0.5 * 32.5e-3; % distance between IMU and front axle [m]
P.P_p_c_1 = 49e-3;            % length of car [m]
P.P_p_c_2 = 22e-3;            % width of car [m]
P.P_p_rear_1 = -6e-3;         % position of rear end of car [m]

% Computer vision
P.P_p_output_Tt = 0e-3;       % image processing dead time [s]
P.P_p_sstd = 0;               % [m]
P.P_p_psistd = 0;             % [rad]

% Total dead time
P.P_p_Tt = 100e-3 + 0e-3;



%% Initialization script
close all;
clear all;
clc;
rng(0);

% Add the functions folder to the path
addpath(genpath(pwd));

% Set the default plotting parameters
set(0,'defaulttextinterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

% Set the default font size
set(0,'defaultAxesFontSize', 20);
set(0,'DefaultLegendFontSize', 20);
set(0,'DefaultFigureWindowStyle','normal');
set(0,'DefaultUicontrolFontsize', 14);


% Simulation parameters
sim_param = struct(...
    'dt',            0.1,        ...    % [s]   Time step
    'max_iter',      1e4,        ...    % [-]   Maximum number of iterations
    'N_agents',      3,          ...    % [-]   Number of agents
    'ENV_SIZE',      60,         ...    % [m]   Size of the environment
    'AGENT_RADIUS',  1,        ...    % [m]   Agent radius
    'WHEEL_RADIUS',  0.01,       ...    % [m]   Wheel radius
    'AXLE_LENGTH',   0.05,       ...    % [m]   Inter-Axle length
    'std_dyn_xy',    0.1,        ...    % [m]   Standard deviation of the dynamic noise
    'std_dyn_theta', 1*(pi/180), ...    % [rad] Standard deviation of the dynamic noise
    'std_gps',       0.1,        ...    % [m]   Standard deviation of the GPS noise
    'MAX_LIN_VEL',   1e-6,       ...    % [m/s] Maximum linear velocity
    'MAX_LIN_ACC',   0.5,        ...    % [m/s^2] Maximum linear acceleration
    'MAX_ANG_VEL',   0.3,        ...    % [rad/s] Maximum angular velocity
    'GAINS',         [1 1],      ...    % [-]   Gains of the forward control
    'RADIUS_SEPARATION', 0.2,    ...    % [m] Minimum separation distance between agents
    'RADIUS_ALIGNMENT', 0.3,     ...    % [m] Radius of the alignment function
    'RADIUS_COHESION', 0.4,      ...    % [m] Radius of the cohesion function
    'FORCE_SEPARATION', 0.1,     ...    % [N] Force of the separation function
    'FORCE_ALIGNMENT', 0.2,      ...    % [N] Force of the alignment function
    'FORCE_COHESION', 0.3        ...    % [N] Force of the cohesion function
    );

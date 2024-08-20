%% Simulation parameters
params = struct(...
    'dt',            0.1,                   ...    % [s]   Time step                                            SIMULATION AND ENV PARAMS     
    'max_iter',      1e4,                   ...    % [-]   Maximum number of iterations
    'D',             @(x,y) 10 * exp(-((x - (8)).^2 + (y - (5)).^2) / (2*300)), ... % []   Field function
    'x_max_peak',    8,                     ...    % [m]    For simulation purposes, x-peak coordinate
    'y_max_peak',    5,                     ...    % [m]    For simulation purposes, y-peak coordinate
    'ENV_SIZE',      60,                    ...    % [m]   Size of the environment
    'ENV_STEP',      200,                   ...    % [-]   Number of steps for the environment
    'q0',            [-20, 20, deg2rad(30)],  ...    % [m]   Initial position and orientation of the agent      ROBOT PARAMS
    'control_alg',   'Matveev-v1',          ...    % [-]   Control algorithm.  
    'MAX_LIN_VEL',   0.7,                   ...    % [m/s] Maximum linear velocity of the agent 5
    'MAX_ANG_VEL',   0.8,                   ...    % [rad/s] Maximum angular velocity of the agent 3
    'V_STAR',        0.04,                  ...    % [-]   Matveev main parameter
    'AGENT_RADIUS',  1,                     ...    % [m]   Agent radius
    'WHEEL_RADIUS',  0.01,                  ...    % [m]   Wheel radius
    'AXLE_LENGTH',   0.05,                  ...    % [m]   Inter-Axle length
    'std_dyn_xy',    0.5,                   ...    % [m]   Standard deviation of the dynamic noise
    'std_dyn_theta', 1*(pi/180),            ...    % [rad] Standard deviation of the dynamic noise
    'std_gps',       5,                     ...    % [m]   Standard deviation of the GPS noise
    'N_agents',      3,                     ...    % [-]   Number of agents'                                    DRONES PARAMS
    'DRONE_RADIUS',  1,                     ...    % [m]     Drone radius
    'std_drones',    5,                   ...    % [m]     Standard deviation of the noise affecting the TDOA measurements
    'N_min_agents',  8,                     ...    % [-]     Minimum number of agents
    'N_max_agents',  16,                    ...    % [-]     Maximum number of agents
    'radius',        40                     ...    % [m]     Radius of region for agents initialization
    );

% Generate a number of drones between range [N_min_agents, N_max_agents]
params.N_agents = randi([params.N_min_agents,params.N_max_agents]);
% % Define the radius for drones' initializaiton
% params.radius = floor(params.ENV_SIZE/4 + params.N_agents);

%% Change the field D with these possible functions:
% @(x,y) 18 * exp(-((x - (-20)).^2 + (y - (-18)).^2) / 600) - 10*exp(-((x-10).^2+(y-8).^2)/200)

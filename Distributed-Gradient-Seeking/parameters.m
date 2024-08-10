%% Simulation parameters
params = struct(...
    'dt',            0.1,                   ...    % [s]     Time step
    'max_iter',      1e3,                   ...    % [-]     Maximum number of iterations
    'D',             @(x,y) 18 * exp(-((x - (-20)).^2 + (y - (-18)).^2) / 600) - 10*exp(-((x-10).^2+(y-8).^2)/200), ... % []   Field function
    'ENV_SIZE',      60,                    ...    % [m]     Size of the environment
    'ENV_STEP',      200,                   ...    % [-]     Number of steps for the environment
    'q0',            [0, 20, deg2rad(30)],  ...    % [m]     Initial position and orientation
    'control_alg',   'Matveev',             ...    % [-]     Control algorithm
    'DRONE_RADIUS',  1,                     ...    % [m]     Drone radius
    'AGENT_RADIUS',  1,                     ...    % [m]     Agent radius
    'WHEEL_RADIUS',  0.01,                  ...    % [m]     Wheel radius
    'AXLE_LENGTH',   0.05,                  ...    % [m]     Inter-Axle length
    'std_dyn_xy',    0.2,                   ...    % [m]     Standard deviation of the dynamic noise
    'std_dyn_theta', 1*(pi/180),            ...    % [rad]   Standard deviation of the dynamic noise
    'std_gps',       3,                     ...    % [m]     Standard deviation of the GPS noise
    'std_drones',    0.1,                   ...    % [m]     Standard deviation of the noise affecting the TDOA measurements
    'MAX_LIN_VEL',   1e-6,                  ...    % [m/s]   Maximum linear velocity
    'MAX_LIN_ACC',   0.5,                   ...    % [m/s^2] Maximum linear acceleration
    'MAX_ANG_VEL',   0.3,                   ...    % [rad/s] Maximum angular velocity
    'GAINS',         [1 1],                 ...    % [-]     Gains of the forward control
    'N_min_agents',  8,                     ...    % [-]     Minimum number of agents'
    'N_max_agents',  16                     ...    % [-]     Maximum number of agents'
    );

% Generate a number of drones between range [4,10]
params.N_agents = randi([params.N_min_agents,params.N_max_agents]);
% Define the radius for drones' initializaiton
params.radius = floor(params.ENV_SIZE/4 + params.N_agents);

%% Change the field D with these possible functions:
% @(x,y) 10 * exp(-((x - (-20)).^2 + (y - (-18)).^2) / (2*300))
%% Initialization
init

%% Get parameters
parameters

%% Simulation parameters and simulation initialization
dt = params.dt;                     % [s] - time step
iterations = params.max_iter;       % [#] - number of iteration
T = iterations/dt;                  % [s] - simulation duration
ANIMATE=0;
PLOT_ERROR_ELLIPSES=1;
MISSING_MEASUREMENT_ITER=[100:1:150];        % [-] - Array of values for which is simulated missing measurement 100:1:150
WITH_DRONES=0;                      % [-] - Simulation with drones (otherwise with simple gps)

% Environment/Field definition
environment;

% According to Matveev paper, make hypotesis regarding the field and check
% robot parameters
R_star = 3;                         % [m] - Desired distance of arrival from the maximum point of the field
q_minus = 5;                        % [-] - Lower bound of the intensity of the field
sigma2_minus = 100;               % [m^2] - Lower limit of the field variance
sigma2_plus = 500;                % [m^2] - Upper limit of the field variance
R_est = 9;                        % [m] - Estimated started distance from the field center
checkMatveevCond;                 % Checking Matveev parameters

% Start Simulation ?
fprintf('\nPress any key to continue or "q" to quit:');
keyPressed = input('', 's');  % Wait for key press
% Check if the user pressed 'q' or any other key
if strcmpi(keyPressed, 'q')
    disp('Execution stopped.');
    return;  % Stops the execution of the script
else
    disp('Continuing execution...');
end

% Create ROBOT agent with the initial position and orientation;
robot = AGENT(params.q0,1,'unicycle',params);

% Create DRONE agents with initial position
drone_pose = Drone_Init(params);
for idx = 1:params.N_agents
    drone(idx) = DRONE(drone_pose(idx,:)',idx,'linear',params);
end

% Variables to calculate RMSE value
diff_rd = 0; % between real pose and estimated pose by the drones
diff_rk = 0; % between real pose and estimated pose by the robot's kalman filter

% Arrays to store outputs for plotting
time = zeros(1, iterations);
q_ROBOT_real_vals = zeros(3, iterations);                   % [m, m, rad] - ROBOT Real pose
q_ROBOT_est_vals = zeros(3, iterations);                    % [m, m, rad] - ROBOT Estimated pose
v_ROBOT_vals = zeros(1, iterations);                        % [m/s] - ROBOT Linear Velocity values
w_ROBOT_vals = zeros(1, iterations);                        % [rad/s] - ROBOT Angular Velocity values
D_ROBOT_vals = zeros(1, iterations);                        % [-] - ROBOT Field measured values
k_ROBOT_vals = zeros(1, iterations);                        % [-] - ROBOT Field Curvature values
P_ROBOT_vals = cell(1, iterations);                         % [-] - ROBOT Covariance values
dw_ROBOT_vals = zeros(1,iterations);

q_real_drones_vals = zeros(3,iterations,params.N_agents);   % [m, m] - DRONE Real pose of the drones
x_est_vals = zeros(3,iterations);                           % [-] - DRONE Estimated state of robot
P_est_vals = cell(1,iterations);                            % [-] - DRONE Covariance values of drones


%%  ============ Main Loop Simulation =====================================
for i = 1:iterations
    fprintf("<strong>Iteration</strong> [%d]\n",i);
    t=(i-1)*dt;
    time(i)=t;
    
    % Save parameters for plots
    q_ROBOT_est_vals(:,i) = robot.q_est;
    q_ROBOT_real_vals(:,i) = robot.q_real;
    D_ROBOT_vals(i) = robot.D_new;
    k_ROBOT_vals(i) = curvature(Z,[robot.q_real(1),robot.q_real(2)],x_range,y_range);
    P_ROBOT_vals{i} = robot.P;
    
    
    %% Start loop
    
    if WITH_DRONES
        % Find neighbors for each drone
        neighbors = Find_Neighbors(params,drone);
        for idx = 1:params.N_agents
            % Assign number of neighboring drones
            drone(idx).N_neighbors = length(neighbors{idx});
            % Assign neighboring drones
            drone(idx).neighbors = neighbors{idx};
            % Check for drones without neighbors and disable them
            Drone_OnOff(idx,drone);
            % Update measurement noise matrix based on number of neighboring drones
            ComputeMeasNoiseMatrix(drone(idx));
        end
    
        % Estimate robot position through TDOA measurements
        for idx = 1:params.N_agents
            IEKF(drone,idx,i,[robot.q_real(1:2);0]);
        end
        % Update the estimated robot's position and the estimated covariance
        % matrix inside drone class
        [x_est, P_est] = Drones_Update(params,i,drone);
        % Save values
        x_est_vals(:,i)=x_est; P_est_vals{i}=P_est;
    end

    % High level control -  Calculate u that depend on the field value
    u = robot.compute_control();
    % Save control vals
    v_ROBOT_vals(i)=u(1); w_ROBOT_vals(i)=u(2); dw_ROBOT_vals(i)=robot.dw;
    

    % Low level control - Actuate the robot with the high level control and update the state based on the system state and GPS measurement
    if WITH_DRONES
        EKF(robot, u, ismember(i,MISSING_MEASUREMENT_ITER), x_est(1:2), P_est(1:2,1:2));
    else
        EKF(robot, u, ismember(i,MISSING_MEASUREMENT_ITER));
    end

    %% End Loop

    % Stop the simulation if the robot reaches the peak
    if norm([params.x_max_peak;params.y_max_peak]'-q_ROBOT_est_vals(1:2,i)')<R_star
        iter_break = i;
        break;
    end
end
iter_break = i;

%% Plotting results
plot_simulation;
if WITH_DRONES plot_simulation_DRONES; end
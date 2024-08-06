%% Initialization
init

%% Get parameters
parameters

%% Simulation parameters and simulation initialization
dt = params.dt;
iterations = params.max_iter;
T = iterations/dt;
ANIMATE = 0;

% Environment/Field definition
environment;

% Create a robot agent with the initial position and orientation;
robot = AGENT(params.q0,1,'unicycle',params);

% Create drone agents with the initial position
for i = 1:params.N_agents
    q0_drone = Drone_Init(params);
    drone(i) = DRONE(q0_drone,i,'unicycle',params);
end

% Arrays to store outputs for plotting
time = zeros(1, iterations);
q_real_vals = zeros(3, iterations);     % Real pose of agent
q_est_vals = zeros(3, iterations);      % Estimated pose of agent
v_vals = zeros(1, iterations);          % Linear Velocity values
w_vals = zeros(1, iterations);          % Angular Velocity values
D_vals = zeros(1, iterations);          % Field measured values
P_vals = cell(1,iterations);            % Covariance values
x_est_drones = zeros(3, iterations);    % Agent position estimated from TDOA
q_real_drones_vals = zeros(3, iterations, params.N_agents); % Real pose of the drones

%% Main Loop Simulation
% Initial real position of target
x = [robot.q_init(1:2);0];
% Initial guess for the position of target --> center of environment
x_est = zeros(3,1);
% Initial guess for the covariance matrix
P = eye(3).*100;

% Matrix to store estimated target position by each drone
x_est_EKF = zeros(3,params.N_agents);
% 3D matrix to store estimated covariance matrix
P_est_EKF = zeros(3,3,params.N_agents);
for idx = 1:params.N_agents
    P_est_EKF(:,:,idx) = eye(3).*100;
end
P_idx = zeros(3);
% Variable to count RMSE value
diff = 0;

for i = 1:iterations

    % DEBUG:
    fprintf("Iteration: %d\n",i);

    t=(i-1)*dt;
    time(i)=t;

    % Save parameters for plots
    q_est_vals(:,i) = robot.q_est;
    q_real_vals(:,i) = robot.q_real;
    D_vals(i) = robot.D_new;
    P_vals{i} = robot.P;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Start loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % High level control -  Calculate u that depend on the field value
    u = robot.compute_control();

    %TODO: Richiamare qua la funzione per il calcolo della posizione del robot tramite triangolazione.
    % si sostituirà quindi SOLO la funzione agent.gps_measurement() dentro a EKF, in modo tale che vada a sostituire la misura attuale del GPS
    for idx = 1:params.N_agents
        [x_est,P_est] = EFK_TDOA(params,drone,idx,x,x_est,P);
        x_est_EKF(:,idx) = x_est;
        P_est_EKF(:,:,idx) = P_est;
    end
    % Compute the mean of estimated covariance matrices
    for idx = 1:params.N_agents
        P_idx = P_est_EKF(:,:,idx) + P_idx;
    end
    P = P_idx/params.N_agents; 
    % Compute the mean of estimated positions
    x_est = mean(x_est_EKF,2);
    x_est_drones(:,i) = x_est;

    % RMSE counter
    diff_i = (x(1)-x_est(1))^2+(x(2)-x_est(2))^2;
    diff = diff + diff_i;

    % High level control for the drones
    if i > 1
        for idx = 1:params.N_agents
            % Compute the controls for each drone
            u_drone = drone(idx).compute_control(x_est_drones,i);
            % Low level control for the drones
            EKF_Drone(drone(idx), u_drone);
        end
    end

    % Low level control - Actuate the robot with the high level control and update the state based on the system state and GPS measurement
    EKF(robot, u, x_est(1:2));
    x(1:2) = robot.q_real(1:2);
    x(3) = 0;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% End Loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Stop the simulation if the robot reaches the peak
    if norm([-20;-18]'-q_est_vals(1:2,i)')<1.5
        iter_break = i;
        break;
    end
end
iter_break = i;

% RMSE
RMSE = sqrt((1/iter_break)*diff);

%% Plotting results
% Flag to plot all figures
% PLOTALL = 1;
% plot_simulation;

% Plot the circular region and drone positions
theta = linspace(0, 2*pi, 100);
circle_x = params.q0(1) + params.radius*cos(theta);
circle_y = params.q0(2) + params.radius*sin(theta);
figure
hold on
h(1) = plot(circle_x, circle_y, 'k--', 'DisplayName','Drones initialization region');
contour(X, Y, Z, 30, 'DisplayName','Scalar Field')
colorbar
h(2) = plot(x_est_drones(1,1:iter_break), x_est_drones(2,1:iter_break), 'm-', 'LineWidth', 2.0, 'DisplayName','Drones Est Trajectory');
h(3) = plot(q_est_vals(1,1:iter_break), q_est_vals(2,1:iter_break), 'r--', 'LineWidth', 1.5, 'DisplayName','Robot Est Trajectory');
h(4) = plot(q_real_vals(1,1:iter_break), q_real_vals(2,1:iter_break), 'b-', 'LineWidth', 1.5, 'DisplayName','Robot Real Trajectory');
xlabel('X Coordinate')
ylabel('Y Coordinate')
title('Unicycle Robot Trajectory with ESC')
legend(h(1:4),'Location','best')
grid on
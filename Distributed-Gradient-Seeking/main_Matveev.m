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

% Create robot agent with initial position and orientation;
robot = AGENT(params.q0,1,'unicycle',params);

% Create drone agents with initial position
drone_pose = Drone_Init(params);
for idx = 1:params.N_agents
    drone(idx) = DRONE(drone_pose(idx,:)',idx,'linear',params);
end

% Arrays to store outputs for plotting
time = zeros(1,iterations);
q_real_vals = zeros(3,iterations);                        % Real pose of robot
q_est_vals = zeros(3,iterations);                         % Estimated pose of robot
v_vals = zeros(1,iterations);                             % Linear Velocity values
w_vals = zeros(1,iterations);                             % Angular Velocity values
D_vals = zeros(1,iterations);                             % Field measured values
P_vals = cell(1,iterations);                              % Covariance values of robot
q_real_drones_vals = zeros(3,iterations,params.N_agents); % Real pose of the drones
P_est_vals = cell(1,iterations);                          % Covariance values of drones
x_est_vals = zeros(3,iterations);                         % Estimated state of robot by the drones

%% Main Loop Simulation

% Final goal position of the robot
x_obj = [-20;-18];
% Variables to calculate RMSE value
diff_rd = 0; % between real pose and estimated pose by the drones
diff_rk = 0; % between real pose and estimated pose by the robot's kalman filter

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MAIN LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:iterations

    % DEBUG:
    fprintf("<strong>Iteration:</strong> %d\n",i);

    t=(i-1)*dt;
    time(i)=t;

    % Save parameters for plots
    q_est_vals(:,i) = robot.q_est;
    q_real_vals(:,i) = robot.q_real;
    D_vals(i) = robot.D_new;
    P_vals{i} = robot.P;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Start loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
    Drones_Update(params,i,drone);

    gdop = GeometricDiluitionOfPrecision(params,drone,i);
    GDOP(:,i) = gdop;

    % Save parameters for plots
    % P_est_vals{i} = drone(1).P;
    % x_est_vals(:,i) = x_est;

    % RMSE counter
    % diff_rd = RMSE(robot.q_real,x_est,diff_rd);       % real pose minus estimate of drones
    % diff_rk = RMSE(robot.q_real,robot.q_est,diff_rk); % real pose minus estimate of robot's kalman filter

    % High level control - Calculate u for the robot based on the field value
    u = robot.compute_control();

    % % Compute the controls and actuate the drones
    % if i == 1
    %     for idx = 1:params.N_agents
    %         % Extract the real pose of the drones
    %         q_real_drones_vals(:,i,idx) = drone(idx).q_real;
    %     end
    % elseif i > 1
    %     for idx = 1:params.N_agents
    %         % High level control - Calculate u to move the drones
    %         u_drone = drone(idx).compute_control(q_real_vals,i);
    %         % Low level control - Actuate the drones based on the calculated controls
    %         drone(idx).dynamics(u_drone);
    %         % Extract the real pose of the drones
    %         q_real_drones_vals(:,i,idx) = drone(idx).q_real;
    %     end
    % end

    % Low level control - Actuate the robot with the high level control and
    % update the state based on the system state and drones' estimate
    % EKF(robot, u, drone(1).x_est(1:2,i+1));
    EKF(robot, u);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% End Loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Stop the simulation if the robot reaches the peak
    if norm(x_obj'-q_est_vals(1:2,i)') < 1.5
        iter_break = i;
        % % Root Mean Square Error: standard deviation of the residuals
        % RMSE_rd = sqrt((1/iter_break)*diff_rd); % [m] RMSE between real pose and estimated one by the drones
        % RMSE_rk = sqrt((1/iter_break)*diff_rk); % [m] RMSE between real pose and estimated one by robot's kalman filter
        break;
    end
end
iter_break = i;

%% Plotting results
% Flag to plot all figures
% PLOTALL = 0;
% plot_simulation;

% % Plot the circular region and drone positions
% theta = linspace(0, 2*pi, 100);
% circle_x = params.radius*cos(theta);
% circle_y = params.radius*sin(theta);
% figure;
% hold on;
% plot(circle_x, circle_y, 'k--', 'DisplayName','Drones initialization region');
% axis([-params.ENV_SIZE, params.ENV_SIZE, -params.ENV_SIZE,  params.ENV_SIZE]);
% xlabel('X');
% ylabel('Y');
% title('Unicycle Robot Gradient Descent to Find Gaussian Peak');
% grid on;
% %contourf(X, Y, D(X,Y), 50, 'LineStyle', 'none');
% contour(X, Y, Z, 30);
% colorbar;
% for i = 1:length(time(1:iter_break))
%     agent=robot.PlotAgent(q_real_vals(:,i));
%     plot(q_real_vals(1, 1:i), q_real_vals(2, 1:i), 'b-', 'LineWidth', 2);
%     for idx = 1:params.N_agents
%         DRONE(idx) = drone(idx).PlotDrone(q_real_drones_vals(:,i,idx));
%         plot(q_real_drones_vals(1,1:i,idx),q_real_drones_vals(2,1:i,idx), 'r-','LineWidth',1.5);
%     end
%     pause(dt);
%     if ishandle(agent)
%         delete(agent);
%     end
%     if ishandle(DRONE)
%         delete(DRONE);
%     end
% end
% scatter(drone(1).x_est(1,1:iter_break), drone(1).x_est(2,1:iter_break), 'Marker','o','LineWidth', 0.1);

% Plot the circular region and drone positions
theta = linspace(0, 2*pi, 100);
circle_x = params.radius*cos(theta);
circle_y = params.radius*sin(theta);
for idx = 1:params.N_agents
    figure
    hold on
    h(1) = plot(circle_x, circle_y, 'k--', 'DisplayName','Drones initialization region');
    contour(X, Y, Z, 30, 'DisplayName','Scalar Field')
    colorbar
    h(2) = plot(q_est_vals(1,1:iter_break), q_est_vals(2,1:iter_break), 'k--', 'LineWidth', 1.5, 'DisplayName','Robot Est Trajectory');
    h(3) = plot(q_real_vals(1,1:iter_break), q_real_vals(2,1:iter_break), 'b-', 'LineWidth', 1.5, 'DisplayName','Robot Real Trajectory');
    drone(idx).PlotDrone(drone(idx).q_init);
    h(4) = plot(drone(idx).x_est(1,1:iter_break), drone(idx).x_est(2,1:iter_break), 'ro','LineWidth', 0.1, 'DisplayName',sprintf("Est. Drone %d",idx));
    % scatter(drone(idx).x_est(1,1:iter_break), drone(idx).x_est(2,1:iter_break), 'Marker','o','LineWidth', 0.1);
    xlabel('X Coordinate')
    ylabel('Y Coordinate')
    title('Unicycle Robot Trajectory with ESC')
    legend(h(1:4),'Location','best')
    grid on
end
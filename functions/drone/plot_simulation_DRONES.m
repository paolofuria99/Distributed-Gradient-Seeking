if ANIMATE_DRONES
    % Plot the circular region and drone positions
    theta = linspace(0, 2*pi, 100);
    circle_x = params.radius*cos(theta);
    circle_y = params.radius*sin(theta);
    kfig = kfig + 1;
    figure(kfig);
    %set(gcf, 'WindowState', 'maximized');
    set(gcf, 'Position', [100 100 1200 720]);
    plot(circle_x, circle_y, 'k--', 'DisplayName','Drones initialization region');
    hold on;
    axis([-params.ENV_SIZE, params.ENV_SIZE, -params.ENV_SIZE, params.ENV_SIZE]);
    xlabel('X');
    ylabel('Y');
    title('Unicycle Robot Gradient Descent to Find Gaussian Peak');
    contour(X, Y, Z, 30);  % Create the contour plot
    colorbar;
    grid on;
    
    % Preallocate plot handles for the trajectories
    h_real_traj = plot(NaN, NaN, 'b-', 'LineWidth', 2, 'DisplayName', 'Robot Real Traj.');
    h_est_traj = plot(NaN, NaN, 'r--', 'LineWidth', 2, 'DisplayName', 'Robot Est Traj.');
    
    for i = 1:length(time(1:iter_break))
        % Update the trajectories
        set(h_real_traj, 'XData', q_ROBOT_real_vals(1, 1:i), 'YData', q_ROBOT_real_vals(2, 1:i));
        set(h_est_traj, 'XData', q_ROBOT_est_vals(1, 1:i), 'YData', q_ROBOT_est_vals(2, 1:i));
        
        % Plot the agent and keep the handle
        agent = robot.PlotAgent(q_ROBOT_real_vals(:,i));
        % Plot the drones and jeep the handle
        for idx = 1:params.N_agents
            DRONE(idx) = drone(idx).PlotDrone(q_real_drones_vals(:,i,idx));
            plot(q_real_drones_vals(1,1:i,idx),q_real_drones_vals(2,1:i,idx), 'r-','LineWidth',1.5);
        end
        
        % Redraw the plot only every few iterations or after significant changes
        if mod(i, 3) == 0 || i == length(time(1:iter_break))
            drawnow;
        end
        
        % Only delete the agent handle if necessary
        if ishandle(agent)
            delete(agent);
        end
        if ishandle(DRONE)
            delete(DRONE);
        end
    end
end

% Initialize empty arrays for edges
sourceNodes = [];
targetNodes = [];

% Populate sourceNodes and targetNodes based on the connections
for i = 1:length(neighbors)
    for j = 1:length(neighbors{i})
        sourceNodes = [sourceNodes; i]; % Add the current node
        targetNodes = [targetNodes; neighbors{i}(j)]; % Add the connected node
    end
end

% Create the graph object
G = graph(sourceNodes, targetNodes);

% Plot the graph
kfig = kfig + 1;
figure(kfig);
plot(G, 'Layout', 'force');
title('Graph Representation of Connections');



% Plot the circular region and drone positions
theta = linspace(0, 2*pi, 100);
circle_x = params.radius*cos(theta);
circle_y = params.radius*sin(theta);
kfig = kfig + 1;
figure(kfig);
hold on
h(1) = plot(circle_x, circle_y, 'k--', 'DisplayName','Drones initialization region');
contour(X, Y, Z, 30, 'DisplayName','Scalar Field')
colorbar
h(2) = plot(q_ROBOT_est_vals(1,1:iter_break), q_ROBOT_est_vals(2,1:iter_break), 'k--', 'LineWidth', 1.5, 'DisplayName','Robot Est Trajectory');
h(3) = plot(q_ROBOT_real_vals(1,1:iter_break), q_ROBOT_real_vals(2,1:iter_break), 'b-', 'LineWidth', 1.5, 'DisplayName','Robot Real Trajectory');
h(4) = plot(drone(1).x_est(1,1:iter_break), drone(1).x_est(2,1:iter_break), 'ro','LineWidth', 0.1, 'DisplayName',"TDOA-based Est");
for idx = 1:params.N_agents
    drone(idx).PlotDrone(drone(idx).q_init);
end
xlabel('X Coordinate')
ylabel('Y Coordinate')
title('Unicycle Robot Trajectory with ESC')
legend(h(1:4),'Location','best')
grid on

% Plot norm on estimation error before and after consensus
kfig = kfig + 1;
figure(kfig);
for idx = 1:params.N_agents
    h(idx) = plot(1:iter_break,error(:,:,idx),"LineWidth",0.1,"DisplayName",sprintf("Drone %d est.",idx));
    hold on;
end
h(params.N_agents+1) = plot(1:iter_break,error_consensus,"-k","LineWidth",1.5,"DisplayName","consensus est.");
xlabel("iteration");
ylabel("$||\tilde{x}||$");
legend(h);
title("Norm of estimation error before and after consensus");
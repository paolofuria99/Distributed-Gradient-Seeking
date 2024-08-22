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
%     agent=robot.PlotAgent(q_ROBOT_real_vals(:,i));
%     plot(q_ROBOT_real_vals(1, 1:i), q_ROBOT_real_vals(2, 1:i), 'b-', 'LineWidth', 2);
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
figure;
plot(G, 'Layout', 'force');
title('Graph Representation of Connections');



% Plot the circular region and drone positions
theta = linspace(0, 2*pi, 100);
circle_x = params.radius*cos(theta);
circle_y = params.radius*sin(theta);
figure
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
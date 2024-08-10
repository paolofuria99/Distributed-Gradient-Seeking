function Drones_Update(params,i,drone)
%DRONES_UPDATE This function computes the average estimated state of the
%robot and the average estimated covariance matrix and assigns them to the
%classes that identify each drone in the system
%   Detailed explanation goes here

% Matrix to store weighted average of estimated robot's states and
% covariance matrices
x_est_average = zeros(3,params.N_agents);
P_est_average = zeros(3,3,params.N_agents);
for j = 1:params.N_agents
    if drone(j).Connection == "on"
        % Weight of drone j
        w_jj = drone(j).N_neighbors;
        x_est_counter = w_jj*drone(j).x_est(:,i+1);
        P_counter = w_jj*drone(j).P;
        for k = 1:drone(j).N_neighbors
            % Weight of each k-th neighboring drone of drone j
            w_jk(k) = drone(drone(j).neighbors(k)).N_neighbors;
            x_est_counter = x_est_counter + w_jk(k)*drone(drone(j).neighbors(k)).x_est(:,i+1);
            P_counter = P_counter + w_jk(k)*drone(drone(j).neighbors(k)).P;
        end
        x_est = x_est_counter/(sum(w_jk)+drone(j).N_neighbors);
        x_est_average(:,j) = x_est;
        P = P_counter/(sum(w_jk)+drone(j).N_neighbors);
        P_est_average(:,:,j) = P;
    else
        x_est_average(:,j) = drone(j).x_est(:,i+1);
        P_est_average(:,:,j) = drone(j).P;
    end
end

% Assign weighted average of robot's estimated state and covariance matrix
% to each drone
for j = 1:params.N_agents
    if drone(j).Connection == "on"
        drone(j).x_est(:,i+1) = x_est_average(:,j);
        drone(j).P = P_est_average(:,:,j);
    end
end

end

% % Variable to calculate the mean of all covariance matrices estimated by
% % the drones when running IEKF
% P_counter = zeros(3);
% x_est_counter = zeros(3,1);
% % Compute the mean of estimated covariance matrices
% for idx = 1:params.N_agents
%     P_counter = drone(idx).P + P_counter;
%     x_est_counter = drone(idx).x_est(:,i+1) + x_est_counter;
% end
% P = P_counter/params.N_agents;
% x_est = x_est_counter/params.N_agents;

% fprintf("x_est_mean: %g %g %g\n", x_est);

% % Assign the mean estimated covariance matrix and mean estimated robot's state to each drone 
% for idx = 1:params.N_agents
%     drone(idx).P = P;
%     drone(idx).x_est = x_est;
% end


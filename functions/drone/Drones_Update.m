function [x_est, P_est] = Drones_Update(params,i,drone)
%DRONES_UPDATE This function computes the average estimated state of the
%robot and the average estimated covariance matrix and assigns them to the
%classes that identify each drone in the system
% INPUTS:
% params - Parameters of the simulation
% i -      Iteration number
% drone -  Drones classes
% OUTPUTS:
% x_est -  Robot's estimated state from weighted-averaging consensus
% P_est -  Estimated covariance matrix from weighted-averaging consensus

% Iterate weighted-averaging between estimated state by the drones until
% convergence is reached and all drones agree on the same estimate
for index = 1:100
    % Matrix to store weighted average of estimated robot's states and
    % covariance matrices
    x_est_average = zeros(3,params.N_agents);
    P_est_average = zeros(3,3,params.N_agents);
    % Initialization variables
    w_jj = zeros(1,params.N_agents); % Weight of drone j
    x_est_counter = zeros(3,params.N_agents);
    P_counter = zeros(3,3,params.N_agents);
    for j = 1:params.N_agents
        if drone(j).Connection{i} == "on"
            % Weight of drone j
            w_jj(j) = drone(j).N_neighbors;
            x_est_counter(:,j) = w_jj(j)*drone(j).x_est(:,i+1);
            P_counter(:,:,j) = w_jj(j)*drone(j).P;
            w_jk = zeros(1,drone(j).N_neighbors);
            for k = 1:drone(j).N_neighbors
                % Weight of each k-th neighboring drone of drone j
                w_jk(k) = drone(drone(j).neighbors(k)).N_neighbors;
                x_est_counter(:,j) = x_est_counter(:,j) + w_jk(k)*drone(drone(j).neighbors(k)).x_est(:,i+1);
                P_counter(:,:,j) = P_counter(:,:,j) + w_jk(k)*drone(drone(j).neighbors(k)).P;
            end
            x_est = x_est_counter(:,j)/(sum(w_jk)+w_jj(j));
            x_est_average(:,j) = x_est;
            P = P_counter(:,:,j)/(sum(w_jk)+w_jj(j));
            P_est_average(:,:,j) = P;
        else
            x_est_average(:,j) = drone(j).x_est(:,i+1);
            P_est_average(:,:,j) = drone(j).P;
        end
    end
    
    % Assign weighted average of robot's estimated state and covariance matrix
    % to each drone
    for j = 1:params.N_agents
        if drone(j).Connection{i} == "on"
            drone(j).x_est(:,i+1) = x_est_average(:,j);
            drone(j).P = P_est_average(:,:,j);
        end
    end
end

% Since one estimate has been agreed by the system of drones, x_est can be
% taken arbitrarily by a random drone in the environment
x_est = drone(1).x_est(:,i+1);
P_est = drone(1).P;

end
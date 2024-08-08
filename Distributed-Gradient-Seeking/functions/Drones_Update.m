function x_est = Drones_Update(params,drone)
%DRONES_UPDATE This function computes the average estimated state of the
%robot and the average estimated covariance matrix and assigns them to the
%classes that identify each drone in the system
%   Detailed explanation goes here

% Variable to calculate the mean of all covariance matrices estimated by
% the drones when running IEKF
P_counter = zeros(3);
x_est_counter = zeros(3,1);
% Compute the mean of estimated covariance matrices
for idx = 1:params.N_agents
    P_counter = drone(idx).P + P_counter;
    x_est_counter = drone(idx).x_est + x_est_counter;
end
P = P_counter/params.N_agents;
x_est = x_est_counter/params.N_agents;
% Assign the mean estimated covariance matrix and mean estimated robot's state to each drone 
for idx = 1:params.N_agents
    drone(idx).P = P;
    drone(idx).x_est = x_est;
end

end


function [error,error_norm,min_error,idx_min_error,x_est] = BestEstimate(params, drone, i)
%BESTESTIMATE Summary of this function goes here
%   Detailed explanation goes here

% Array to store errors between robot's estimated position and real position
error = zeros(3, params.N_agents);
% Array to store errors norm
error_norm = zeros(1, params.N_agents);
for idx = 1:params.N_agents
    error(:, idx) = drone(idx).Delta_x(:, i);
    error_norm(idx) = norm(error(:, idx));
end
[min_error, idx_min_error] = min(error_norm);
% Best overall estimate
x_est = drone(idx_min_error).x_est(:, i+1);


end


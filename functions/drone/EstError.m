function [error,error_consensus] = EstError(params,iter_break,drone,q_ROBOT_real_vals,x_est_vals)
%ESTERROR Compute the norm of the estimation error for each drone before
%consensus and the norm of the erroron the overall estimate after consensus
% with other drones
% INPUTS:
% params -            Parameters of the simulation
% iter_break -        Total number of iterations
% drone -             Drones classes
% q_ROBOT_real_vals - Robot real pose
% x_est_vals -        Estimated pose of the robot after consensus
% OUTPUTS:
% error -             Estimation error before consensus for each drone
% error_consensus -   Estimation error after consensus for overall estimate

x_est_EKF_drones = zeros(3,iter_break,params.N_agents);
for idx = 1:params.N_agents
    x_est_EKF_drones(:,:,idx) = drone(idx).x_est_EKF(:,2:iter_break+1);
end
q_ROBOT_real_vals_with_z = [q_ROBOT_real_vals(1:2,1:iter_break);zeros(1,iter_break)];
dist = zeros(1,iter_break,params.N_agents);
for k = 1:iter_break
    for idx = 1:params.N_agents
        error(1,k,idx) = sqrt((q_ROBOT_real_vals_with_z(1,k)-x_est_EKF_drones(1,k,idx))^2 + ...
                              (q_ROBOT_real_vals_with_z(2,k)-x_est_EKF_drones(2,k,idx))^2 + ...
                              (q_ROBOT_real_vals_with_z(3,k)-x_est_EKF_drones(3,k,idx))^2);
    end
end

for k = 1:iter_break
    error_consensus(k) = sqrt((q_ROBOT_real_vals_with_z(1,k)-x_est_vals(1,k))^2 + ...
                              (q_ROBOT_real_vals_with_z(2,k)-x_est_vals(2,k))^2 + ...
                              (q_ROBOT_real_vals_with_z(3,k)-x_est_vals(3,k))^2);
end

end


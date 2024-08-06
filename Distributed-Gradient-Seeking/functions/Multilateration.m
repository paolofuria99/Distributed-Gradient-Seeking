function x_est = Multilateration(params,drone,x_est,x)
%MULTILATERATION Summary of this function goes here
%   Detailed explanation goes here

% Parameter to exit NLS (Non-linear Least Squares)
epsilon = 1;
% Variable to save all estimates
x_est_drones = zeros(3,1,params.N_agents);

% NLS recursion
for idx = 1:params.N_agents
    x_est = zeros(3,1);
    while true
        h = TimeDifferenceOfArrival(idx,params,x,drone);
        measurement_noise = MeasurementNoise(drone(idx));
        h_noise = h + measurement_noise;
        h_true = h_noise;
        h_est = TimeDifferenceOfArrival(idx,params,x_est,drone);
        Deltay = h_true - h_est;
        H_est = Jacobian_measModel(idx,params,drone,x_est);
        Deltax = (H_est'*H_est)\H_est'*Deltay';
        fprintf("%g %g %g\n",Deltax);
        x_est = x_est + Deltax;
        if norm(x-x_est) <= epsilon
            x_est_drones(:,:,idx) = x_est;
            break;
        end
    end
end

x_est_x = x_est_drones(1,:,:);
x_est(1) = mean(x_est_x);
x_est_y = x_est_drones(2,:,:);
x_est(2) = mean(x_est_y);
x_est_z = x_est_drones(3,:,:);
x_est(3) = mean(x_est_z);

end


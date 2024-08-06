function [x_est,P] = EFK_TDOA(params,drone,idx,x,x_est,P)
%EFK_TDOA Summary of this function goes here
%   Detailed explanation goes here

while true
    %% 1) Prediction
    % Initial guess
    x_dyn = x_est;
    P_dyn = P;
    % Evaluate Jacobian matrix at the estimated state
    H = Jacobian_measModel(idx,params,drone,x_dyn);

    %% 2) Update
    % Update the innovation covariance matrix
    S = H * P_dyn * H' + drone(idx).Z_drones;
    % Update the Kalman gain
    K = (P_dyn * H') / S;
    % Update the state of the system
    tmp = K * (TimeDifferenceOfArrival(idx,params,x,drone)+MeasurementNoise(drone(idx)) - ...
               TimeDifferenceOfArrival(idx,params,x_dyn,drone))'; % Innovation
    x_est = x_dyn + tmp;
    P = (eye(3) - K * H) * P_dyn * (eye(3) - K * H)' + K * drone(idx).Z_drones * K';

    % Convergence check
    if norm(x_est-x_dyn) < 1e-3
        break;
    end
end

end


function IEKF(params,drone,idx,x)
%EFK_TDOA Summary of this function goes here
%   Detailed explanation goes here

% Extrapolate necessary parameters / EKF inputs
P = drone(idx).P;
x_est = drone(idx).x_est;
Q = drone(idx).Q;

% Counter for the iterative EKF
i = 1;
% Max number of iterations
iterations = 1;
while i <= iterations
    %% 1) Prediction
    % Initial guess
    x_dyn = x_est;
    P_dyn = P + Q;
    % Evaluate Jacobian matrix at the estimated state
    H = Jacobian_measModel(idx,params,drone,x_dyn);

    %% 2) Update
    % Update the innovation covariance matrix
    S = H * P_dyn * H' + drone(idx).R;
    % Update the Kalman gain
    K = (P_dyn * H') / S;
    % Update the state of the system
    tmp = K * (TimeDifferenceOfArrival(idx,params,x,drone)+MeasurementNoise(drone(idx)) - ...
               TimeDifferenceOfArrival(idx,params,x_dyn,drone))'; % Innovation
    x_est = x_dyn + tmp;
    P = (eye(3) - K * H) * P_dyn * (eye(3) - K * H)' + K * drone(idx).R * K';

    % Update the counter
    i = i + 1;
    
end

%% Update the drone's measurements and covariance matrix/ EKF outputs
drone(idx).P = P;
drone(idx).x_est = x_est;

end

% % Convergence check
% if norm(x_est-x_dyn) < 1e-3
%     break;
% end

function IEKF(drone,idx,i,x)
%EFK_TDOA Summary of this function goes here
%   Detailed explanation goes here

if drone(idx).Connection == "on"
    % Extrapolate necessary parameters / EKF inputs
    P = drone(idx).P;
    x_est = drone(idx).x_est(:,i);
    Q = drone(idx).Q;
    noise = MeasurementNoise(drone(idx));
    R = drone(idx).R;
    
    %% 1) Prediction
    x_dyn = x_est;
    P_dyn = P + Q;
    
    % Linearize around current estimated state
    H = Jacobian_measModel(idx,drone,x_dyn);
    
    %% 2) Update
    % Update the innovation covariance matrix
    S = H * P_dyn * H' + R;
    % Update the Kalman gain
    K = (P_dyn * H') / S;
    % Update the state of the system
    tmp = K * (TimeDifferenceOfArrival(idx,x,drone) + noise - ...
               TimeDifferenceOfArrival(idx,x_dyn,drone))'; % Innovation
    x_est = x_est + tmp;
    P = (eye(3) - K * H) * P_dyn * (eye(3) - K * H)' + K * R * K';
    
    %% 3) Linear correction to estimated state
    % Linearize around current estimated state
    % H = Jacobian_measModel(idx,drone,x_est);
    % WLS_matrix = (H' * (R \ H));
    % lambda = 1e-10;                       % Regularization parameter
    % reg = lambda * eye(size(WLS_matrix)); % Regularization term
    % WLS_matrix = WLS_matrix + reg;
    % Delta_x = -inv(WLS_matrix) * (H' / R) * (TimeDifferenceOfArrival(idx,x,drone) - TimeDifferenceOfArrival(idx,x_est,drone))';
    % % Update the drone's error
    % drone(idx).Delta_x(:,i) = Delta_x;
    % x_est = x_est - Delta_x;
else
    % Keep the last estimate
    x_est = drone(idx).x_est(:,i);
    P = drone(idx).P;
end

%% Update the drone's measurements and covariance matrix/ EKF outputs
drone(idx).P = P;
drone(idx).x_est(:,i+1) = x_est;
% fprintf("Drone %d: %g %g %g\n",idx,x_est);

end
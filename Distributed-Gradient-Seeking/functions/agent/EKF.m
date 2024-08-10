% The model of the analized problem is:
%     
%     q(k+1,k) = g(x(k,k),u(k),v(k))    -> Qk=[v(k)*v(k)'] Process Noise covariance matrix
%     z(k) = h(x(k,k),w(k))             -> Rk=[w(k)*w(k)'] Measurement Noise covariance matrix
% 		
% The EKF is divided into steps: 
%     - predicion:
%         q_est(k+1,k) = g(q_est(k,k), u(k),0)
%         P_est(k+1,k) = Jg_q(k) * P_est(k,k) * Jg_q(k)' + Jg_v(k) * Q(k) * Jg_v(k)'
% 
%         where:
%         Jg_q(k) is the jacobian of g(q,u,nu) w.r.t the state e and evaluated in the q_est(k,k) and u(k) with v = 0
%         Jg_v(k) is the jacobian of g(q,u,nu) w.r.t v and evaluated in the q_est(k,k) and u(k) with v = 0
%      
%     time delay k->k-1 (  e.g. P_est(k+1,k) => P_est(k,k-1) )

%     - update:
%         S(k,k-1) = Jh_q(k) * P_est(k,k-1) * Jh_q(k)' + R(k) 				                        % S(k) is the innovation covariance matrix
%         K(k) = P_est(k,k-1) * Jh_q(k)' * inv(S(k,k-1))						                    % W(k) is the kalman gain
%         q_est(k,k) = q_est(k,k-1) + W(k) (z(k) - h(q_est(k,k-1),0) )	                            % q_est(k,k) is the updated state
%         P_est(k,k) = (I - W(k) * Jh_q(k))) * P_est(k,k-1)(I - W(k) * Jh_q(k)))' + K(k)R(k)K(k)'	% P_est(k,k) is the updated covariance matrix


function EKF(agent, u, x_est)
    %EKF Extended Kalman Filter init
    
    % Extrapolate necessary parameters / EKF inputs
    Jg_q = agent.get_Jg_q(u);
    Jg_v = agent.get_Jg_v();
    Jh_q = agent.get_Jh_q();
    Jh_w = agent.get_Jh_w();
    P = agent.P;
    Q = agent.Q;
    R = agent.R_gps;
    I = eye(size(P)); 
    
    %% 1) Prediction
    
    % Predict the next state of the system through the dynamics
    agent.dynamics(u);
    q_est = agent.q_est;
    % Predict the covariance matrix of the agent at the next step
    P = Jg_q * P * Jg_q' + Jg_v * Q * Jg_v';
    
    %% 2) Update
    
    % Update the innovation covariance matrix
    S = Jh_q * P * Jh_q' + Jh_w * R * Jh_w';
    % Update the Kalman gain
    K = P * Jh_q' * inv(S);
    % Update the next state of the system
    tmp = K*(agent.gps_measurement() - q_est(1:2)); % Innovation
    % tmp = K*(x_est - q_est(1:2)); % Innovation
    q_est(1:2) = q_est(1:2) + tmp(1:2);
    q_est(3) = wrapTo2Pi(q_est(3) + wrapTo2Pi(tmp(end)));
    P = (I- K * Jh_q) * P * (I- K * Jh_q)' + K * R * K';

    %% Update the agent's state and covariance matrix/ EKF outputs
    agent.q_est = q_est;
    agent.P = P;
    

end
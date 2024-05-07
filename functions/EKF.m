% The model of the analized problem is:
%     
%     q(k) = g(x(k-1),u(k),w(k))    -> Qk=[w(k)*w(k)'] Process Noise covariance matrix
%     z(k) = h(x(k),v(k))           -> Rk=[v(k)*v(k)'] Measurement Noise covariance matrix
% 		
% The EKF is divided into steps: 
%     - predicion:
%         q_est(k) = g(q_est(k-1), u(k),0)
%         P_est(k) = Jg_q(k-1) * P_est(k-1) * Jg_q(k-1)' + Jg_w(k-1) * Q(k-1) * Jg_w(k-1)'
% 
%         where:
%         Jg_q(k) is the jacobian of g(q,u,nu) w.r.t the state e and evaluated in the q_est(k-1) and u(k) with w = 0
%         Jg_w(k) is the jacobian of g(q,u,nu) w.r.t w e and evaluated in the q_est(k-1) and u(k-1) with w = 0
%      
%      - update:
%         S(k) = Jh_q(k) * P_est(k) * Jh_q(k)' + R(k) 				    % S(k) is the innovation covariance matrix
%         K(k) = P_est(k) * Jh_q(k)' / S(k)						        % W(k) is the kalman gain
%         q_est(k) = q_est(k) + W(k) ( z(k) - h(x_est(k+1),0) )	        % q_est(k) is the updated state
%         P_est(k) = (I - W(k) * Jh_q(k))) * P_est(k)				    % P_est(k) is the updated covariance matrix


function EKF(agent, u )
    %EKF Extended Kalman Filter init
    
    Jg_q = agent.get_Jg_q();
    Jg_w = agent.get_Jg_w();
    Jh_q = agent.get_Jh_q();
    Jh_v = agent.get_Jh_v();
    
    
    %% 1) Propagation
    
    % Predict the next state of the system through the dynamics
    agent.dynamics(u);
    % Predict the covariance matrix of the agent at the next step
    agent.P = Jg_q * agent.P * Jg_q' + Jg_w * agent.Q * Jg_w';
    
    %% 2) Correction
    
    % Update the innovation covariance matrix
    S = Jh_q * agent.P * Jh_q' + Jh_v * agent.R_gps * Jh_v';
    % Update the Kalman gain
    K = agent.P * Jh_q' / S;
    % Update the next state of the system
    tmp = K*(agent.gps_measurement() - agent.q_est(1:2));
    agent.q_est(1:2) = agent.q_est(1:2) + tmp(1:2);
    agent.q_est(3) = wrapTo2Pi(agent.q_est(3) + wrapTo2Pi(tmp(end)));
    agent.P = (eye(size(agent.P)) - K * Jh_q) * agent.P;

end
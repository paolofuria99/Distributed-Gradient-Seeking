classdef AGENT < matlab.mixin.Copyable
    %AGENT This class is used to create an agent object
    %  Detailed explanation goes here

    properties(Access=public)
        type                % [string]              Type of agent (unicycle, etc.)
        id                  % [-]   (1x1 int)       ID of the agent
        agent_radius        % [m]   (1x1 double)    Radius of the agent

        % State of the agent
        x_est               % [m]   (2x1 double)    Estimated state of the agent
        th_est              % [rad] (1x1 double)    Estimated angle of the agent
        P                   % Covariance matrix of the state

        % Real state of the agent
        x_real              % [m]   (2x1 double)    Real state of the agent
        th_real             % [rad] (1x1 double)    Real angle of the agent
        x_init              % [m]                   Initial state of the agent
        
        % Noise matrices
        R_gps               % Covariance matrix of the GPS measurements
        Q                   % Uncertainty matrix of the dynamics model

        % Jacobians
        J_H                 % Jacobian of the measurement model

        % Target information
        target_est          % [m]   (2x1 double)    Estimated target position
        target_real         % [m]   (2x1 double)    Real target position
        target_cov_pos      % Covariance of the target position
        
        % Communication
        neighbors; 			% list of the neighbors of the agent
		all_agent_pos; 	    % [m]                   List of the neighbors positions (also target)
		all_agent_pos_cov; 	% covariance of the neighbors positions (also target)

        % Control gains
        gains               % [-]   (2x1 double)    Control gains of the agent

        % Maximum velocities
        v_max               % [m/s]     (1x1 double)    Maximum linear velocity of the agent
        w_max               % [rad/s]   (1x1 double)    Maximum angular velocity of the agent

        % Max acceleration
        a_max               % [m/s^2]   Maximum linear acceleration of the agent
    end

    methods
        function obj = AGENT(q, id, type, params)
            %AGENT Construct Initialize an agent object
            %  Input:
            %   q (3x1 double): Generalized coordinates of the agent
            %   id (int):       ID of the agent
            %   type:           Type of dynamic of the agent
            %   params:         Parameters of the agent
            
            obj.type = type;
            obj.id = id;

            switch obj.type
                case 'unicycle'
                    % Initialize the state of the agent
                    obj.x_real = zeros(2,1);
                    obj.x_real(1) = q(1);
                    obj.x_real(2) = q(2);
                    obj.x_init = obj.x_real;
                    obj.x_est = obj.x_real;

                    % Initialize the angle of the agent
                    obj.th_real = q(3);
                    obj.th_est = q(3);
                    obj.P = eye(length(q));
                    
                    % Initialize the noise matrices
                    obj.Q = zeros(3,3);
                    obj.Q(1:2,1:2) = (rand(2,2)-0.5)*params.std_dyn_xy;
                    obj.Q(1:2,1:2) = obj.Q(1:2,1:2)*obj.Q(1:2,1:2)';
                    obj.Q(3,3) = (rand())*(params.std_dyn_theta)^2;
                    
                    % Initialize the Jacobian of the measurement model
                    obj.J_H = [1 0 0; 0 1 0];

                    % Initialize the control limits
                    obj.v_max = params.MAX_LIN_VEL;
                    obj.w_max = params.MAX_ANG_VEL;
                    obj.a_max = params.MAX_LIN_ACC;

                    % Initialize the control gains
                    obj.gains = params.GAINS;
                otherwise
                    error('Unknown agent type');
            end
            
            % Measurement model noise
            obj.R_gps = (rand(2,2)-0.5)*params.std_gps;
            obj.R_gps = obj.R_gps*obj.R_gps';

            % Agent radius
            obj.agent_radius = params.AGENT_RADIUS;

            % Target information
            obj.target_est = zeros(2,1);
            obj.target_real = zeros(2,1);
            obj.target_cov_pos = eye(2);

            % Neighbors
            obj.neighbors = [];
            obj.all_agent_pos = ones(params.N_agents,2)*1e6;
            obj.all_agent_pos_cov = eye(params.N_agents,2)*1e6;

        end

        function obj = dynamics(obj, u)
            %DYNAMICS Update the state of the agent based on the control input
            %  Input:
            %   u (2x1 double): Control input of the agent
            switch obj.type
                case 'unicycle'
                    % Considering to have u=[v*dt; w*dt]
                    % where v is the driving velocity and w is the angular velocity
                    % q_new = q_old + G(q)*u + noise
                    % x_new = x_old + cos(theta)*v*dt + noise_x
                    % y_new = y_old + sin(theta)*v*dt + noise_y
                    % theta_new = theta_old + w*dt + noise_theta
                    
                    x_old = obj.x_est(1);
                    y_old = obj.x_est(2);
                    theta_old = obj.th_est;

                    % Dynamics with noise
                    G = [cos(theta_old) 0;sin(theta_old) 0; 0 1];
                    q_new = [x_old;y_old;theta_old] + G*u + mvnrnd(zeros(3,1),obj.Q)';

                    obj.x_est = q_new(1:2);
                    obj.th_est = wrapTo2Pi(q_new(3));

                    % Dynamics without noise
                    q_new = [x_old;y_old;theta_old] + G*u;

                    obj.x_real = q_new(1:2);
                    obj.th_real = wrapTo2Pi(q_new(3));

                otherwise
                    error('Unknown agent type');
            end   
            
            
        end

        function F_x = Jacobian_PredictedState_PreviousState(obj)
            %JACOBIAN_PREDICTEDSTATE_PREVIOUSTATE Compute the Jacobian of the dynamics wrt state
            %  Output:
            %   F_x (3x3 double): Jacobian of the state
            switch obj.type
                case 'unicycle'
                    F_x = [1 0 -sin(obj.th_est); 0 1 cos(obj.th_est); 0 0 1];
                otherwise
                    error('Unknown agent type - Jacobian_PredictedState_PreviousState');
            end
        end

        function F_w = Jacobian_PredictedState_Noise(obj)
            %JACOBIAN_PREDICTEDSTATE_NOISE Compute the Jacobian of the dynamics wrt noise
            %  Output:
            %   F_w (3x3 double): Jacobian of the noise
            switch obj.type
                case 'unicycle'
                    F_w = eye(3);
                otherwise
                    error('Unknown agent type - Jacobian_PredictedState_Noise');
            end
        end

        function H_x = Jacobian_Measurement_State(obj)
            %JACOBIAN_MEASUREMENT_STATE Compute the Jacobian of the measurement wrt state
            %  Output:
            %   H_x (2x3 double): Jacobian of the measurement
            switch obj.type
                case 'unicycle'
                    % The measurement model is h(x) = [x; y] 
                    % The derivative wrt q=[x,y,th] is [1 0 0; 0 1 0]
                    H_x = obj.H_x;
                otherwise
                    error('Unknown agent type - Jacobian_Measurement_State');
            end
        end

        function H_w = Jacobian_Measurement_Noise(obj)
            %JACOBIAN_MEASUREMENT_NOISE Compute the Jacobian of the measurement wrt noise
            %  Output:
            %   H_w (2x3 double): Jacobian of the measurement
            switch obj.type
                case 'unicycle'
                    % The measurement model is h(x) = [x; y] 
                    % The derivative wrt q=[x,y,th] is [1 0 0; 0 1 0]
                    H_w = obj.H_w;
                otherwise
                    error('Unknown agent type - Jacobian_Measurement_Noise');
            end
        end

        function Z_gps = gps_measurement(obj)
            %GPS_MEASUREMENT Compute the GPS measurement
            %  Output:
            %   Z_gps (2x1 double): GPS measurement

            Z_gps = obj.x_real(1:2) + mvnrnd(zeros(2,1),obj.R_gps)';

        end

        function x_initialization(obj)
            %X_INITIALIZATION Initialize the state of the agent
            obj.x_real = obj.x_init;
        end

        function update_target(obj, target)
            %UPDATE_TARGET Update the target position
            %  Input:
            %   target (2x1 double): Target position
            obj.target_est = target;
        end

        function compute_control(obj, param)
            %COMPUTE_CONTROL Compute the control input for the robot based on Boyd model
            %  Input:
            %   param (struct): Simulation parameters
            goal = obj.target_est;
            dt = param.dt;
            
            x = obj.x_est;
            theta = obj.th_est;
            
            kv = obj.gains(1); 
            kw = obj.gains(2);

            if norm(goal-x) < 0.5
                v = 0;
                w = 0;
            else
                v = kv * ([cos(theta) sin(theta)]*(goal-x));
                w = kw * atan2( [-sin(theta) cos(theta)]*(goal-x) , [cos(theta) sin(theta)]*(goal-x) );
            end
            
            
            % Limit the control input
             v = min(param.MAX_LIN_VEL, v);
            % w = min(param.MAX_ANG_VEL, w);
            % Apply the control input
            u = [v*dt; w*dt];
            obj.dynamics(u);
            
        end

        % Calculate neighbors relative distances
        % all_agent_pos = obj.all_agent_pos;
        % all_agent_dist = sqrt(all_agent_pos(:,1).^2+all_agent_pos(:,2).^2);

        % % Depending on the distance to the neighbors, the agent will
        % % choose the control input
        % agents_in_separation = find(all_agent_dist<param.RADIUS_SEPARATION); 
        % agents_in_alignment = find(all_agent_dist<param.RADIUS_ALIGNMENT);
        % agents_in_cohesion = find(all_agent_dist<param.RADIUS_COHESION);

        % % Separation
        % u_separation = all_agent_dist(agents_in_separation)/norm(all_agent_pos(agents_in_separation));
        

    end
end
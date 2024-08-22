classdef AGENT < matlab.mixin.Copyable
    %AGENT This class is used to create an agent object
    %  This is the robot agent class
    
    properties(Access=public)
        type                % [string]              Type of agent (unicycle, etc.)
        id                  % [-]   (1x1 int)       ID of the agent

        % Max velocities
        v_max               % [m/s]         (1x1 double)    Max linear velocity of the agent
        w_max               % [rad/s]       (1x1 double)    Max angular velocity of the agent
        
        % Initial state of the agent
        q_init              % [m;m;rad]     (3x1 double)    Initial state of the agent

        % State of the agent
        q_est               % [m;m;rad]     (3x1 double)    Estimated state of the agent

        % Real state of the agent
        q_real              % [m;m;rad]     (3x1 double)    Real state of the agent
        P                   % []           (3x3 double)    Covariance matrix of the state
        Q                   % []           (3x3 double)    Uncertainty matrix of the dynamics model

        % Measuremente noise matrices
        R_gps               %               (2x2 double)    Covariance matrix of the GPS measurements
        
        % Jacobians
        Jg_q                % Jacobian of the motion model wrt the state
        Jg_v                % Jacobian of the motion model wrt the process noise
        Jh_q                % Jacobian of the measurement model wrt the state
        Jh_w                % Jacobian of the measurement model wrt the measurement noise

        % Field Sensor
        D                   % [?]           (-)    Field function
        D_new               % [?]           (1xiterations double)    Field value
        D_old               % [?]           (1xiterations double)    Field value
        d_est               % [?]           (2x1 double)    Estimated field value
        d_real              % [?]           (2x1 double)    Real field value
        d_std               % std of the sensor

        % Parameters
        params              % [struct]              Simulation parameters

        % Parameters for GF-mom algorithm
        % D_now
        % D_delta_x
        % D_delta_y
        % theta_now
        % k
        % vx
        % vy
        % onlyone

        % Parameters for Matveev-v3
        v_star;
        integral_error  % Integral of the error
        previous_error  % Previous error for derivative calculation
        dw
        
    end
    
    % This function are public i.e. externally accessible
    methods(Access=public)
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
                    obj.v_max = params.MAX_LIN_VEL/params.dt;
                    obj.w_max = params.MAX_ANG_VEL/params.dt;

                    % Reshape the input to a column vector 
                    q=reshape(q,[3,1]); 

                    % Initialize the state of the agent
                    obj.q_real = q;
                    obj.q_init = q;
                    obj.q_est = [0,0,0];
                    
                    % Initialize the initial state covariance matrix
                    obj.P = eye(length(q)).*100;
                    
                    % Initialize the noise matrices
                    obj.Q = zeros(3,3);
                    obj.Q(1:2,1:2) = (rand(2,2)-0.5)*params.std_dyn_xy;
                    obj.Q(1:2,1:2) = obj.Q(1:2,1:2)*obj.Q(1:2,1:2)';
                    obj.Q(3,3) = (rand())*(params.std_dyn_theta)^2;

                otherwise
                    error('Unknown agent type');
            end
            
            % Measurement model noise
            obj.R_gps = (rand(2,2)-0.5)*params.std_gps;
            obj.R_gps = obj.R_gps*obj.R_gps';

            % Field Sensor initialization
            obj.D = params.D;
            obj.D_new = 0;
            obj.D_old = 0;

            % Target information
            obj.d_est = 0;
            obj.d_real= 0;
            % TODO: d_std to be imported from params.
            obj.d_std = 0;
             
            % Initialize the parameters
            obj.params = params;

            % Initialize the parameters for GS-mom algorithm
            % obj.D_now = 0;
            % obj.D_delta_x = 0;
            % obj.D_delta_y = 0;
            % obj.vx = 0;
            % obj.vy = 0;
            % obj.k = 0;
            % obj.onlyone = true;

            obj.integral_error=0;
            obj.previous_error=0;
            obj.v_star=0;
            obj.dw=0;
        end

        function obj = dynamics(obj, u)
            %DYNAMICS Update the state of the agent based on the control input
            %   Input:
            %       u (2x1 double): Control input of the agent

            u = reshape(u, [2,1]);

            switch obj.type
                case 'unicycle'
                    % Considering to have u=[v*dt; w*dt]
                    % where v is the driving velocity [m/s] and w is the angular velocity [rad/s]
                    
                    % Matrix form:
                    % q_new = q_old + G(q_old)*u + noise
                    
                    % Equation form:
                    % x_new = x_old + cos(theta)*v*dt + noise_x
                    % y_new = y_old + sin(theta)*v*dt + noise_y
                    % theta_new = theta_old + w*dt + noise_theta

                    % Estimated Dynamics without noise
                    q_old = reshape(obj.q_est,[3,1]);
                    x_old = q_old(1);
                    y_old = q_old(2);
                    theta_old = q_old(3);
                    G = [cos(theta_old) 0;sin(theta_old) 0; 0 1];
                    q_new = q_old+ G*u;
                    
                    obj.q_est = [q_new(1:2); wrapTo2Pi(q_new(3))];

                    % True Dynamics with noise
                    q_old = reshape(obj.q_real,[3,1]);
                    x_old = q_old(1);
                    y_old = q_old(2);
                    theta_old = q_old(3);
                    G = [cos(theta_old) 0; sin(theta_old) 0; 0 1];
                    q_new = q_old + G*u + mvnrnd(zeros(3,1),obj.Q)';
                    
                    obj.q_real = [q_new(1:2);wrapTo2Pi(q_new(3))];

                otherwise
                    error('Unknown agent type');
            end   
        end
        
        function Jg_q = get_Jg_q(obj, u)
            %GET_JG_Q Compute the Jacobian of the motion model wrt state
            %   Input:
            %       - u (2x1 double): control input
            %   Output:
            %       - Jg_q (3x3 double): Jacobian of the state
            u=reshape(u,[2,1]);
            switch obj.type
                case 'unicycle'
                    Jg_q = [1 0 -u(1)*sin(obj.q_est(3)); 0 1 u(1)*cos(obj.q_est(3)); 0 0 1];
                otherwise
                    error('Unknown agent type - get_Jg_q');
            end
        end

        function Jg_v = get_Jg_v(obj)
            %GET_JG_V Compute the Jacobian of the motion model wrt process noise
            %  Output:
            %   Jg_v (3x3 double): Jacobian of the noise
            switch obj.type
                case 'unicycle'
                    Jg_v = eye(3);
                otherwise
                    error('Unknown agent type - get_Jg_v');
            end
        end

        function Jh_q = get_Jh_q(obj)
            %GET_JH_Q Compute the Jacobian of the measurement wrt state
            %  Output:
            %   Jh_q (2x3 double): Jacobian of the measurement
            switch obj.type
                case 'unicycle'
                    % The measurement model is h(x) = [x; y] 
                    % The derivative wrt q=[x,y,th] is [1 0 0; 0 1 0]
                    Jh_q = [1 0 0; 0 1 0];
                otherwise
                    error('Unknown agent type - get_Jh_q');
            end
        end

        function Jh_w = get_Jh_w(obj)
            %GET_JH_W Compute the Jacobian of the measurement wrt measurement noise
            %  Output:
            %   Jh_w (2x3 double): Jacobian of the measurement
            switch obj.type
                case 'unicycle'
                    % The measurement model is h(x) = [x; y] 
                    % The derivative wrt q=[x,y,th] is [1 0 0; 0 1 0]
                    Jh_w = eye(2);
                otherwise
                    error('Unknown agent type - get_Jh_w');
            end
        end
        
        function Z_gps = gps_measurement(obj)
            %GPS_MEASUREMENT Compute the GPS measurement
            %  Output:
            %   Z_gps (2x1 double): GPS measurement

            Z_gps = obj.q_real(1:2) + mvnrnd(zeros(2,1),obj.R_gps)';

        end
        

        function u = compute_control(obj)
            %COMPUTE_CONTROL Compute the control input for the robot 
            %  Input:
            %   obj (AGENT): Agent object
            %  Output:
            %   u (2x1 double): Control input
            switch obj.params.control_alg
                case 'Matveev-v1'
                    % Control algorithm parameters based on Matveev's paper
                    dv = obj.v_max*obj.params.dt; 
                    v_star = obj.params.V_STAR; 
                    dw = @(dd) (obj.w_max*obj.params.dt*sign(dd - v_star));
                    
                    % Measure the field
                    obj.D_new = obj.D(obj.q_real(1), obj.q_real(2));
                    D_dot = obj.D_new - obj.D_old;
                    
                    % Compute the control input
                    u = [dv; dw(D_dot)];
                    obj.dw = dw(D_dot); % For debug purposes
                    
                    % Update the field value
                    obj.D_old = obj.D_new;
                case 'Matveev-v2'
                    % Initialization of parameters and variables
                    dv = obj.v_max* obj.params.dt; 
                    v_star_base = obj.params.V_STAR; % Base value of v_star
                    k_p = 0.1; % Proportional gain for adjusting v_star
                    previous_D = obj.D_old;
                    
                    % Measure the field
                    obj.D_new = obj.D(obj.q_real(1), obj.q_real(2));
                    D_dot = obj.D_new - previous_D;
                    
                    % Adaptive v_star based on the field gradient
                    obj.v_star = v_star_base + k_p * D_dot;
                    
                    % Control law for angular velocity with sliding mode
                    dw = @(dd) (obj.w_max * obj.params.dt * sign(dd - obj.v_star));

                    % Compute the control input
                    u = [dv; dw(D_dot)];
                    obj.dw = dw(D_dot); % For debug purposes
                    
                    % Update the field value and previous_D
                    obj.D_old = obj.D_new;
                case 'Matveev-v3'
                    % Initialization of parameters and variables
                    dv = obj.v_max * obj.params.dt; 
                    v_star_base = obj.params.V_STAR; % Base value of v_star
                    k_p = 0.1; % Proportional gain
                    k_i = 0.05; % Integral gain 0.05
                    k_d = 0.01; % Derivative gain 0.01
                    previous_D = obj.D_old;
                    integral_error = obj.integral_error; % Integral of the error
                    previous_error = obj.previous_error; % Previous error for derivative calculation
                    
                    % Measure the field
                    obj.D_new = obj.D(obj.q_real(1), obj.q_real(2));
                    D_dot = obj.D_new - previous_D;
                    
                    % Calculate the error (difference between current and base field value)
                    error = D_dot;
                    
                    % Update the integral of the error
                    integral_error = integral_error + error * obj.params.dt;
                    
                    % Calculate the derivative of the error
                    derivative_error = (error - previous_error) / obj.params.dt;
                    
                    % PID control to adjust v_star v_star_base/2
                    obj.v_star =  + k_p * error + k_i * integral_error + k_d * derivative_error; 
                    %obj.v_star =  max(min(obj.v_star, v_star_base), 0);
                    obj.v_star =  max(obj.v_star, 0);

                    % Control law for angular velocity with sliding mode
                    dw = @(dd) (obj.w_max * obj.params.dt * sign(dd - obj.v_star));
                    
                    % Compute the control input
                    u = [dv; dw(D_dot)];
                    obj.dw = dw(D_dot); % For debug purposes
                    
                    % Update the field value, previous_D, and previous_error
                    obj.D_old = obj.D_new;
                    obj.previous_error = error;
                    obj.integral_error = integral_error;


                % case 'GS-mom'   % Gradient seeking
                %   % GS parameters
                %     delta_v = 0.5; % [m/s] Small step for finite difference approximation
                %     delta_w = 0.5; % [rad/s] Small step for finite difference approximation
                %     alpha = 0.1; % Proportional constant for speed
                %     beta = 0.9; % Momentum coefficient
                % 
                %     % Iterations:
                %     %   0 - The robot measures the field in the initial position
                %     %   1 - The robot moves in the x direction by delta and measures the field
                %     %   2 - The robot moves back in the x direction by delta without measuring the field
                %     %   3 - The robot rotates and moves in the y direction by delta and measures the field
                %     %   4 - The robot rotates back to the x direction without measuring the field
                %     %   5 - The robot performs the gradient-seeking algorithm and moves in the direction of the gradient
                % 
                %     % Create the holding variables
                % 
                %     switch obj.k
                %         case 0
                %             % Measure the field at the initial position
                %             obj.D_new = obj.D(obj.q_real(1), obj.q_real(2));
                %             obj.D_now = obj.D_new;
                % 
                %             % Return the control input for the next iteration
                %             u = [delta_v; 0];
                %             obj.k = obj.k + 1;
                % 
                %         case 1
                %             % Measure the field after moving in the x direction
                %             obj.D_new = obj.D(obj.q_real(1), obj.q_real(2));
                %             obj.D_delta_x = obj.D_new;
                % 
                %             % Return the control input for the next iteration
                %             u = [-delta_v; 0];
                %             obj.k = obj.k + 1;
                % 
                %         case 2
                %             % Rotate the robot to move in the y direction
                %             if obj.onlyone
                %                 obj.theta_now = obj.q_est(3);
                %                 obj.onlyone = false;
                %             end
                % 
                %             if ~obj.onlyone
                %                 u = [0; delta_w];
                %                 % Check if the robot has rotated by 90 degrees
                %                 if abs(obj.theta_now + pi/2 - obj.q_est(3)) < 0.05
                %                     obj.onlyone = true;
                %                     obj.k = obj.k + 1;
                %                     u = [+delta_v; 0];
                %                 end
                %             end
                % 
                %         case 3
                %             % Measure the field after moving in the y direction
                %             obj.D_new = obj.D(obj.q_real(1), obj.q_real(2));
                %             obj.D_delta_y = obj.D_new;
                % 
                %             % Return the control input for the next iteration
                %             u = [0; -delta_v];
                %             obj.k = obj.k + 1;
                % 
                %         case 4
                %             % Rotate the robot back to the x direction
                %             if obj.onlyone
                %                 obj.theta_now = obj.q_est(3);
                %                 obj.onlyone = false;
                %             end
                % 
                %             if ~obj.onlyone
                %                 u = [0; -delta_w];
                %                 % Check if the robot has rotated back to the x direction
                %                 if abs(obj.theta_now - pi/2 - obj.q_est(3)) < 0.05
                %                     obj.onlyone = true;
                %                     obj.k = obj.k + 1;
                %                     u = [+delta_v; 0];
                %                 end
                %             end
                % 
                %         case 5
                %             % Compute the gradient
                %             grad_x = (obj.D_delta_x - obj.D_now) / delta_v;
                %             grad_y = (obj.D_delta_y - obj.D_now) / delta_v;
                %             grad_norm = norm([grad_x; grad_y]);
                % 
                %             % Avoid division by zero
                %             if grad_norm > 0
                %                 obj.vx = beta * obj.vx - alpha * grad_x / grad_norm;
                %                 obj.vy = beta * obj.vy - alpha * grad_y / grad_norm;
                %             else
                %                 obj.vx = beta * obj.vx;
                %                 obj.vy = beta * obj.vy;
                %             end
                % 
                %             % Compute the control input
                %             u = [sqrt(obj.vx^2 + obj.vy^2); atan2(obj.vy, obj.vx)];
                % 
                %             % Reset variables
                %             obj.D_now = 0;
                %             obj.D_delta_x = 0;
                %             obj.D_delta_y = 0;
                %             obj.theta_now = 0;
                %             obj.k = 0;
                % 
                %         otherwise
                %             error('Unknown iteration');
                %     end
                otherwise
                    error('Unknown control algorithm');
            end
        end

        function agent = PlotAgent(obj,state)
            %PLOTAGENT Plot the agent
            %  This function plots the agent in the workspace
            
            if nargin == 1
                % State of the agent
                X = obj.q_real;
            else
                X = state;
            end
            X = reshape(X,[1,3]);
            % Robot radius
            R = obj.params.AGENT_RADIUS;

            % Robot Visualization
            th = linspace(0,2*pi,100); % Angle samples for the visualization of robot body
            thwl = linspace(5*pi/6, pi/6, 60); % Angle samples for the visualization of the left robot wheel
            thwr = linspace(7*pi/6, 11*pi/6, 60); % Angle sample for the visualization of the right robot wheel

            % Robot body
            body = patch('XData', X(1,1) + R*cos(th),'YData', X(1,2) + R*sin(th), 'FaceColor',  'red');
            % Left wheel
            wheel_left = patch('XData', X(1,1) + R*cos(thwl+X(1,3)), 'YData',  X(1,2) + R*sin(thwl+X(1,3)), 'FaceColor', 'k');
            % Right wheel
            wheel_right= patch('XData', X(1,1) + R*cos(thwr+X(1,3)), 'YData',  X(1,2) + R*sin(thwr+X(1,3)), 'FaceColor', 'k');
            % Direction indicator
            thick = 0.05;
            thdir = [ pi/2, -pi/2, linspace(-asin(thick/R), asin(thick/R), 60)];
            mdir = [repmat(thick, 1, 2), repmat(R, 1, 60)];
            arrow = patch('XData', X(1,1)+mdir.*cos(thdir+X(1,3)),'YData', X(1,2) + mdir.*sin(thdir+X(1,3)), 'FaceColor', 'k');

            % Return the handle 
            agent = [body, wheel_left, wheel_right, arrow];
        end

    end
end
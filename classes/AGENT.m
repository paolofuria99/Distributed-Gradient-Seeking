classdef AGENT < matlab.mixin.Copyable
    %AGENT This class is used to create an agent object
    %  This is the robot agent class
    
    %TODO: Alcune proprietà saranno private, ovvero non accessibili
    %dall'esterno. Al momento sono pubbliche perché
    properties(Access=public)
        type                % [string]              Type of agent (unicycle, etc.)
        id                  % [-]   (1x1 int)       ID of the agent
        
        % Initial state of the agent
        q_init              % [m;m;rad]     (3x1 double)    Initial state of the agent

        % State of the agent
        q_est               % [m;m;rad]     (3x1 double)    Estimated state of the agent
        P                   %               (3x3 double)    Covariance matrix of the state
        Q                   %               (3x3 double)    Uncertainty matrix of the dynamics model

        % Real state of the agent
        q_real              % [m;m;rad]     (3x1 double)    Real state of the agent
        
        % Measuremente noise matrices
        R_gps               %               (2x2 double)    Covariance matrix of the GPS measurements
        
        % Jacobians
        Jg_q                % Jacobian of the motion model wrt the state
        Jg_w                % Jacobian of the motion model wrt the process noise
        Jh_q                % Jacobian of the measurement model wrt the state
        Jh_v                % Jacobian of the measurement model wrt the measurement noise

        % Sensor information
        d_est               % [?]           (2x1 double)    Estimated field value
        d_real              % [?]           (2x1 double)    Real field value
        d_std               % std of the sensor

        % Parameters
        params              % [struct]              Simulation parameters
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
                    % Reshape the input to a column vector 
                    q=reshape(q,[3,1]); 

                    % Initialize the state of the agent
                    obj.q_real = q;
                    obj.q_init = obj.q_real;
                    obj.q_est = obj.q_real;
                    
                    % Initialize the initial state covariance matrix
                    obj.P = eye(length(q));
                    
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

            % Target information
            obj.d_est = 0;
            obj.d_real= 0;
            % TODO: d_std to be imported from params.
            obj.d_std = 0;
             
            % Initialize the parameters
            obj.params = params;
        end

        % TODO: This function will be private no more accessible
        function obj = dynamics(obj, u)
            %DYNAMICS Update the state of the agent based on the control input
            %  Input:
            %   u (2x1 double): Control input of the agent
            switch obj.type
                case 'unicycle'
                    % Considering to have u=[v*dt; w*dt]
                    % where v is the driving velocity and w is the angular velocity
                    
                    % Matrix form:
                    % q_new = q_old + G(q)*u + noise
                    
                    % Equation form:
                    % x_new = x_old + cos(theta)*v*dt + noise_x
                    % y_new = y_old + sin(theta)*v*dt + noise_y
                    % theta_new = theta_old + w*dt + noise_theta

                    q_old = obj.q_est;
                    x_old = q_old(1);
                    y_old = q_old(2);
                    theta_old = q_old(3);

                    % Dynamics with noise
                    G = [cos(theta_old) 0;sin(theta_old) 0; 0 1];
                    q_new = q_old+ G*u + mvnrnd(zeros(3,1),obj.Q)';
                    
                    obj.q_est = [q_new(1:2); wrapTo2Pi(q_new(3))];

                    % Dynamics without noise
                    q_new = q_old + G*u;
                    
                    obj.q_real = [q_new(1:2);wrapTo2Pi(q_new(3))];

                otherwise
                    error('Unknown agent type');
            end   
        end
        
        % TODO: Da controllare: dovrebbe esserci anche la velocità v
        % nell'equazione
        function Jg_q = get_Jg_q(obj)
            %GET_JG_Q Compute the Jacobian of the motion model wrt state
            %  Output:
            %   Jg_q (3x3 double): Jacobian of the state
            switch obj.type
                case 'unicycle'
                    Jg_q = [1 0 -sin(obj.th_est); 0 1 cos(obj.th_est); 0 0 1];
                otherwise
                    error('Unknown agent type - get_Jg_q');
            end
        end

        function Jg_w = get_Jg_w(obj)
            %GET_JG_W Compute the Jacobian of the motion model wrt process noise
            %  Output:
            %   Jg_w (3x3 double): Jacobian of the noise
            switch obj.type
                case 'unicycle'
                    Jg_w = eye(3);
                otherwise
                    error('Unknown agent type - Jacobian_PredictedState_Noise');
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
                    error('Unknown agent type - Jacobian_Measurement_State');
            end
        end

        function Jh_v = get_Jh_v(obj)
            %GET_JH_V Compute the Jacobian of the measurement wrt measurement noise
            %  Output:
            %   Jh_v (2x3 double): Jacobian of the measurement
            switch obj.type
                case 'unicycle'
                    % The measurement model is h(x) = [x; y] 
                    % The derivative wrt q=[x,y,th] is [1 0 0; 0 1 0]
                    Jh_v = eye(2);
                otherwise
                    error('Unknown agent type - Jacobian_Measurement_Noise');
            end
        end
        
        function Z_gps = gps_measurement(obj)
            %GPS_MEASUREMENT Compute the GPS measurement
            %  Output:
            %   Z_gps (2x1 double): GPS measurement

            Z_gps = obj.q_real(1:2) + mvnrnd(zeros(2,1),obj.R_gps)';

        end

        function q_initialization(obj)
            %X_INITIALIZATION Initialize the state of the agent
            obj.q_real(1:2) = obj.q_init(1:2);
        end
        
        function compute_measure(obj, x)
            %COMPUTE_MEASURE Compute a measurement of the field

            % TODO: Da sistemare aggiungendo anche rumore
       
            obj.z_real = x;
            obj.z_est = x;
        end

        function compute_control(obj)
            %COMPUTE_CONTROL Compute the control input for the robot based on Boyd model
            %  Input:
            %   param (struct): Simulation parameters
            
            
            % TODO: Implement here the algorithm to actuate the robot
            
            % goal = obj.target_est;
            % dt = param.dt;
            
            % x = obj.x_est;
            % theta = obj.th_est;
            
            % kv = obj.gains(1); 
            % kw = obj.gains(2);

            % if norm(goal-x) < 0.5
            %     v = 0;
            %     w = 0;
            % else
            %     v = kv * ([cos(theta) sin(theta)]*(goal-x));
            %     w = kw * atan2( [-sin(theta) cos(theta)]*(goal-x) , [cos(theta) sin(theta)]*(goal-x) );
            % end
            
            
            % % Limit the control input
            %  v = min(param.MAX_LIN_VEL, v);
            % % w = min(param.MAX_ANG_VEL, w);
            % % Apply the control input
            % u = [v*dt; w*dt];
            % obj.dynamics(u);
            
        end

        function agent = PlotAgent(obj,state)
            %PLOTAGENT Plot the agent
            %  Detailed explanation goes here
            
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
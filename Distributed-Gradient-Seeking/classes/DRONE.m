classdef DRONE < matlab.mixin.Copyable
    %DRONE This class is used to create an drone object
    %  This is the drone class
    
    properties(Access=public)
        type                % [string]      Type of drone (unicycle, etc.)
        id                  % [-]           (1x1 int)       ID of the drone
        
        % Initial state of the drone
        q_init              % [m;m;m]       (3x1 double)    Initial state of the drone

        % State of the drone
        q_est               % [m;m;m]       (3x1 double)    Estimated state of the drone

        % Real state of the drone
        q_real              % [m;m;m]       (3x1 double)    Real state of the drone
        P                   % []            (3x3 double)    Covariance matrix of the state
        Q                   % [?]           (3x3 double)    Uncertainty matrix of the dynamics model

        % Measuremente noise matrices
        R_gps               %               (3x3 double)    Covariance matrix of the GPS measurements

        % Noise on drone location and TDOA measurement
        Z_drones            %               (NxN double)    Covariance matrix of the noise on TDOA measurement
        
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
    end
    
    % This function are public i.e. externally accessible
    methods(Access=public)
        function obj = DRONE(q, id, type, params)
            %DRONE Construct Initialize an drone object
            %  Input:
            %   q (4x1 double): Generalized coordinates of the drone
            %   id (int):       ID of the drone
            %   type:           Type of dynamic of the drone
            %   params:         Parameters of the drone
            
            obj.type = type;
            obj.id = id;

            switch obj.type
                case 'unicycle'
                    % Reshape the input to a column vector 
                    q=reshape(q,[3,1]); 

                    % Initialize the state of the drone
                    obj.q_real = q;
                    obj.q_init = q;
                    obj.q_est = q;
                    
                    % Initialize the initial state covariance matrix
                    obj.P = eye(length(q)).*100;
                    
                    % Initialize the noise matrices
                    obj.Q = zeros(3,3);
                    obj.Q = (rand(3,3)-0.5)*params.std_dyn_xy;
                    obj.Q = obj.Q*obj.Q';

                otherwise
                    error('Unknown drone type');
            end
            
            % Measurement model noise
            obj.R_gps = (rand(3,3)-0.5)*params.std_gps;
            obj.R_gps = obj.R_gps*obj.R_gps';

            % TDOA measurement model noise
            obj.Z_drones = (rand(params.N_agents)-0.5)*params.std_drones*params.c;
            obj.Z_drones = obj.Z_drones*obj.Z_drones';

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
        end

        function obj = dynamics(obj, u)
            %DYNAMICS Update the state of the drone based on the control input
            %  Input:
            %   u (2x1 double): Control input of the drone
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
                    q_old = obj.q_est;
                    x_old = q_old(1);
                    y_old = q_old(2);
                    z_old = q_old(3);
                    A = [1 0 0;0 1 0;0 0 1];
                    B = [obj.params.dt 0 0;0 obj.params.dt 0;0 0 0];
                    q_new = A*q_old + B*u;
                    
                    obj.q_est = q_new;

                    % True Dynamics with noise
                    q_old = obj.q_real;
                    x_old = q_old(1);
                    y_old = q_old(2);
                    z_old = q_old(3);
                    A = [1 0 0;0 1 0;0 0 1];
                    B = [obj.params.dt 0 0;0 obj.params.dt 0;0 0 0];
                    q_new = A*q_old + B*u + mvnrnd(zeros(3,1),obj.Q)';
                    
                    obj.q_real = q_new;

                otherwise
                    error('Unknown drone type');
            end   
        end
        
        function Jg_q = get_Jg_q(obj)
            %GET_JG_Q Compute the Jacobian of the motion model wrt state
            %  Output:
            %   Jg_q (3x3 double): Jacobian of the state
            switch obj.type
                case 'unicycle'
                    Jg_q = eye(3);
                otherwise
                    error('Unknown drone type - get_Jg_q');
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
                    error('Unknown drone type - get_Jg_v');
            end
        end

        function Jh_q = get_Jh_q(obj)
            %GET_JH_Q Compute the Jacobian of the measurement wrt state
            %  Output:
            %   Jh_q (2x3 double): Jacobian of the measurement
            switch obj.type
                case 'unicycle'
                    % The measurement model is h(x) = [x; y; z] 
                    % The derivative wrt q=[x,y,z] is [1 0 0; 0 1 0; 0 0 1]
                    Jh_q = eye(3);
                otherwise
                    error('Unknown drone type - get_Jh_q');
            end
        end

        function Jh_w = get_Jh_w(obj)
            %GET_JH_W Compute the Jacobian of the measurement wrt measurement noise
            %  Output:
            %   Jh_w (2x3 double): Jacobian of the measurement
            switch obj.type
                case 'unicycle'
                    % The measurement model is h(x) = [x; y; z] 
                    % The derivative wrt q=[x,y,z] is [1 0 0; 0 1 0; 0 0 1]
                    Jh_w = eye(3);
                otherwise
                    error('Unknown drone type - get_Jh_w');
            end
        end
        
        function Z_gps = gps_measurement(obj)
            %GPS_MEASUREMENT Compute the GPS measurement
            %  Output:
            %   Z_gps (3x1 double): GPS measurement

            Z_gps = obj.q_real(1:3) + mvnrnd(zeros(3,1),obj.R_gps)';

        end
        

        function u = compute_control(obj,x,index)
            %COMPUTE_CONTROL Compute the control input for the drone 
            %  Input:
            %   obj (DRONE): Drone object
            %   x (2,iterations): estimated position of target by the system of drones
            %  Output:
            %   u (2x1 double): Control input
            
            % Check if index is within bounds
            if index <= 1
                error('Index must be greater than 1 to compute direction.');
            end

            direction = [x(1,index)-x(1,index-1),x(2,index)-x(2,index-1),0];
            direction_norm = norm(direction);
            u = direction./direction_norm;
            u = u';

        end


        function distance = Distance_RobotDrone(obj,x)
            %DISTANCE_ROBOTDRONE Relative distance between each pair
            %robot/drone
            % This function calculates the relative distance between the
            % moving target and each drone in the environment
            %  Input:
            %   obj (DRONE): Drone object
            %   x (3x1 double): Robot current state
            %   noise: flag to enable noise on drone position
            %  Output:
            %   distance (1x1 double): Euclidean distance between pair robot/drone

            distance = sqrt((x(1)-obj.q_real(1))^2+(x(2)-obj.q_real(2))^2+(x(3)-obj.q_real(3))^2);

        end


        function measurement_noise = MeasurementNoise(obj)
            %MEASUREMENTNOISE Noise on the TDOA measurement model
            % This function defines the noise on the TDOA measurement model of each drone
            %  Input:
            %   obj (DRONE): Drone object
            %  Output:
            %   measurement_noise (Nx1 double): Noise on TDOA measurements
            measurement_noise = mvnrnd(zeros(obj.params.N_agents,1),obj.Z_drones);
        end


        function drone = PlotDrone(obj,state)
            %PLOTDRONE Plot the drone
            %  This function plots the drone in the workspace
            
            if nargin == 1
                % State of the drone
                X = obj.q_real;
            else
                X = state;
            end
            X = reshape(X,[1,3]);
            % Drone radius
            R = obj.params.DRONE_RADIUS;

            % Drone Visualization
            th = linspace(0,2*pi,100); % Angle samples for the visualization of drone body

            % Drone body
            body = patch('XData', X(1,1) + R*cos(th),'YData', X(1,2) + R*sin(th), 'ZData', repmat(X(3),[1,100]), 'FaceColor',  'red');

            % Return the handle 
            drone = body;
        end

        % function drone = PlotDrone(obj)
        %     %PLOTDRONE Plot the drone
        %     %  Detailed explanation goes here
        % 
        %     % State of the drone
        %     X = [obj.x_real];
        %     X = reshape(X, [1, 3]); % [x y z]
        %     % Robot radius
        %     R = obj.params.DRONE_RADIUS;
        % 
        %     % Robot Visualization
        %     th = linspace(0,2*pi,100); % Angle samples for the visualization of robot body
        %     % thwl = linspace(5*pi/6, pi/6, 60); % Angle samples for the visualization of the left robot wheel
        %     % thwr = linspace(7*pi/6, 11*pi/6, 60); % Angle sample for the visualization of the right robot wheel
        % 
        %     % Robot body
        %     body = patch('XData', X(1) + R*cos(th),'YData', X(2) + R*sin(th), 'ZData', repmat(X(3),[1,100]), 'FaceColor',  'red');
        %     % % Left wheel
        %     % wheel_left = patch('XData', X(1) + R*cos(thwl+X(3)), 'YData',  X(2) + R*sin(thwl+X(3)), 'FaceColor', 'k');
        %     % % Right wheel
        %     % wheel_right= patch('XData', X(1) + R*cos(thwr+X(3)), 'YData',  X(2) + R*sin(thwr+X(3)), 'FaceColor', 'k');
        %     % Direction indicator
        %     % thick = 0.05;
        %     % thdir = [ pi/2, -pi/2, linspace(-asin(thick/R), asin(thick/R), 60)];
        %     % mdir = [repmat(thick, 1, 2), repmat(R, 1, 60)];
        %     % arrow = patch('XData', X(1)+mdir.*cos(thdir+X(4)),'YData', X(2) + mdir.*sin(thdir+X(4)), 'FaceColor', 'k');
        % 
        %     % Return the handle 
        %     % agent = [body, wheel_left, wheel_right, arrow];
        %     drone = [body];
        % end

    end
end
classdef DRONE < matlab.mixin.Copyable
    %DRONE This class is used to create an agent object
    %  This is the drone class
    
    properties(Access=public)
        type                % [string]              Type of agent (linear, unicycle, etc.)
        id                  % [-]   (1x1 int)       ID of the drone
        
        % Initial state of the drone
        x_init              % [m]   (3x1 double)    Initial state of the drone

        % State of the drone
        x_est               % [m]   (3x1 double)    Estimated state of the drone
        th_est              % [rad] (1x1 double)    Estimated angle of the drone
        P                   % Covariance matrix of the state

        % Real state of the drone
        x_real              % [m]   (3x1 double)    Real state of the drone
        th_real             % [rad] (1x1 double)    Real angle of the drone
        
        % Noise matrices
        R_gps               % Covariance matrix of the GPS measurements
        Q                   % Uncertainty matrix of the dynamics model

        % Jacobians
        J_H                 % Jacobian of the measurement model

        % Sensor information
        d_est               % [?]   (3x1 double)    Estimated field value
        d_real              % [?]   (3x1 double)    Real field value
        d_std               % std of the sensor

        % Parameters
        params              % [struct]              Simulation parameters
    end
    
    % This function are public i.e. externally accessible
    methods(Access=public)
        function obj = DRONE(q, id, type, params)
            %DRONE Construct Initialize an agent object
            %  Input:
            %   q (4x1 double): Generalized coordinates of the drone
            %   id (int):       ID of the drone
            %   type:           Type of dynamic of the drone
            %   params:         Parameters of the drone
            
            obj.type = type;
            obj.id = id;

            switch obj.type
                % DA RIVEDERE PER ADATTARLA EVENTUALMENTE AL DRONE
                case 'unicycle'
                    % Reshape the input to a column vector 
                    reshape(q,[4,1]); 

                    % Initialize the state of the drone
                    obj.x_real = zeros(3,1);
                    obj.x_real(1) = q(1);
                    obj.x_real(2) = q(2);
                    obj.x_real(3) = q(3);
                    obj.x_init = obj.x_real;
                    obj.x_est = obj.x_real;

                    % Initialize the angle of the drone
                    obj.th_real = q(4);
                    obj.th_est = q(4);
                    obj.P = eye(length(q));
                    
                    % Initialize the noise matrices
                    obj.Q = zeros(3,3);
                    obj.Q(1:2,1:2) = (rand(2,2)-0.5)*params.std_dyn_xy;
                    obj.Q(1:2,1:2) = obj.Q(1:2,1:2)*obj.Q(1:2,1:2)';
                    obj.Q(3,3) = (rand())*(params.std_dyn_theta)^2;
                    
                    % Initialize the Jacobian of the measurement model
                    obj.J_H = [1 0 0; 0 1 0];

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

        % TODO: This function will be private, no more accessible
        % DA RIVEDERE PER ADATTARLA EVENTUALMENTE AL DRONE
        function obj = dynamics(obj, u)
            %DYNAMICS Update the state of the drone based on the control input
            %  Input:
            %   u (2x1 double): Control input of the drone
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
        
        function compute_measure(obj, x)
            %COMPUTE_MEASURE Compute a measurement of the field

            % TODO: Da sistemare aggiungendo anche rumore
       
            obj.z_real = x;
            obj.z_est = x;
        end

        function drone = PlotDrone(obj)
            %PLOTDRONE Plot the drone
            %  Detailed explanation goes here
            
            % State of the drone
            X = [obj.x_real];
            X = reshape(X, [1, 3]); % [x y z]
            % Robot radius
            R = obj.params.DRONE_RADIUS;

            % Robot Visualization
            th = linspace(0,2*pi,100); % Angle samples for the visualization of robot body
            % thwl = linspace(5*pi/6, pi/6, 60); % Angle samples for the visualization of the left robot wheel
            % thwr = linspace(7*pi/6, 11*pi/6, 60); % Angle sample for the visualization of the right robot wheel

            % Robot body
            body = patch('XData', X(1) + R*cos(th),'YData', X(2) + R*sin(th), 'ZData', repmat(X(3),[1,100]), 'FaceColor',  'red');
            % % Left wheel
            % wheel_left = patch('XData', X(1) + R*cos(thwl+X(3)), 'YData',  X(2) + R*sin(thwl+X(3)), 'FaceColor', 'k');
            % % Right wheel
            % wheel_right= patch('XData', X(1) + R*cos(thwr+X(3)), 'YData',  X(2) + R*sin(thwr+X(3)), 'FaceColor', 'k');
            % Direction indicator
            % thick = 0.05;
            % thdir = [ pi/2, -pi/2, linspace(-asin(thick/R), asin(thick/R), 60)];
            % mdir = [repmat(thick, 1, 2), repmat(R, 1, 60)];
            % arrow = patch('XData', X(1)+mdir.*cos(thdir+X(4)),'YData', X(2) + mdir.*sin(thdir+X(4)), 'FaceColor', 'k');

            % Return the handle 
            % agent = [body, wheel_left, wheel_right, arrow];
            drone = [body];
        end

    end
end
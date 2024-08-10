classdef DRONE < matlab.mixin.Copyable
    %DRONE This class is used to create an drone object
    %  This is the drone class
    
    properties(Access=public)
        type                % [string]                                       Type of drone (linear)
        id                  % [-]           (1x1 int)                        ID of the drone
        
        % Initial state of the drone
        q_init              % [m;m;m]       (3x1 double)                     Initial state of the drone

        % Real state of the drone
        q_real              % [m;m;m]       (3x1 double)                     Real state of the drone
        P                   % []            (3x3 double)                     Covariance matrix of the state

        % Estimated state of the robot by the drone
        x_est               % [m;m;m]       (3xiterations double)            Estimated state of the robot by the drone

        % Process noise of robot's dynamics
        Q                   % []            (3x3 double)                     Noise on the dynamics model of the robot

        % Number of neighbor drones
        N_neighbors         % []            (1x1 double)                     Number of neighbors

        neighbors           % []            (N_neighborsx1 double)           Neighboring drones

        % Noise on TDOA measurement
        R                   % []            (N_neighborsxN_neighbors double) Covariance matrix of the noise on TDOA measurement

        % Flag of drone operation
        Connection          % []            (1x1 string)                     Drone on/off

        % Parameters
        params              % [struct]                                       Simulation parameters
    end
    
    % This function are public i.e. externally accessible
    methods(Access=public)
        function obj = DRONE(q, id, type, params)
            %DRONE Construct Initialize an drone object
            %  Input:
            %   q (3x1 double): Generalized coordinates of the drone
            %   id (int):       ID of the drone
            %   type:           Type of dynamic of the drone
            %   params:         Parameters of the drone
            
            obj.type = type;
            obj.id = id;

            switch obj.type
                case 'linear'
                    % Reshape the input to a column vector 
                    q=reshape(q,[3,1]); 

                    % Initialize the state of the drone
                    obj.q_real = q;
                    obj.q_init = q;
                    
                    % Initialize the initial state covariance matrix
                    obj.P = eye(length(q)).*100;

                    % Initialize the initial estimate of the robot by the
                    % drone
                    obj.x_est = zeros(3,params.max_iter+1);

                    % Initialize the process noise
                    % obj.Q = (randn(3,3)-0.5)*params.std_dyn_xy;
                    % obj.Q = obj.Q*obj.Q';
                    obj.Q = eye(3,3)*params.std_dyn_xy;
                    obj.Q = obj.Q*obj.Q';

                    % Initialize number of neighbor drones
                    obj.N_neighbors = params.N_agents-1;

                    % Initialize neighboring drones
                    obj.neighbors = zeros(obj.N_neighbors,1);

                    % TDOA measurement model noise
                    obj.R = (rand(obj.N_neighbors)-0.5)*params.std_drones;
                    obj.R = obj.R*obj.R';
                    % Drone on/off
                    obj.Connection = 'on';

                otherwise
                    error('Unknown drone type');
            end
             
            % Initialize the parameters
            obj.params = params;
        end

        function obj = dynamics(obj, u)
            %DYNAMICS Update the state of the drone based on the control input
            %  Input:
            %   obj (DRONE):    Drone object
            %   u (2x1 double): Control input of the drone
            switch obj.type
                case 'linear'
                    % Considering to have u=[vx; vy; vz]
                    % where vx, vy and vz are the driving velocities [m/s] 
                    
                    % Matrix form:
                    % q_new = A*q_old + B*u + noise
                    
                    % Equation form:
                    % x_new = x_old + vx*dt 
                    % y_new = y_old + vy*dt 
                    % z_new = z_old

                    % True Dynamics with noise
                    q_old = obj.q_real;
                    x_old = q_old(1);
                    y_old = q_old(2);
                    z_old = q_old(3);
                    A = [1 0 0;0 1 0;0 0 1];
                    B = [1 0 0;0 1 0;0 0 1];
                    q_new = A*q_old + B*u;

                    obj.q_real = q_new;

                otherwise
                    error('Unknown drone type');
            end   
        end
        

        function u = compute_control(obj,x,index)
            %COMPUTE_CONTROL Compute the control input for the drone 
            %  Input:
            %   obj (DRONE):      Drone object
            %   x (2,iterations): Estimated position of target by the system of drones
            %  Output:
            %   u (3x1 double):   Control input
            
            % Check if index is within bounds
            if index <= 1
                error('Index must be greater than 1 to compute direction.');
            end

            %% Method 1) The drones move along the direction of two consecutive drone's state estimates
            direction = [x(1,index)-x(1,index-1), x(2,index)-x(2,index-1), 0-obj.q_real(3)];
            direction_norm = norm(direction);
            direction_rd = [x(1,index)-obj.q_real(1), x(2,index)-obj.q_real(2), 0-obj.q_real(3)];
            distance = sqrt((x(1,index)-obj.q_real(1))^2+(x(2,index)-obj.q_real(2))^2+(0-obj.q_real(3))^2);
            if (distance >= 20) %&& (direction_norm >= 0.5)
                u_lim = 2*obj.params.dt;
                u = direction./direction_norm;
                u = u_lim.*u;
                u = u';
            elseif (distance >= 10) %&& (direction_norm >= 0.5)
                u_lim = obj.params.dt;
                u = direction./direction_norm;
                u = u_lim.*u;
                u = u';
            elseif distance < 10
                u = [0;0;0];
            else
                u = [0;0;0];
            end

            %% Method 2)
            % direction = [x(1,index)-x(1,index-1), x(2,index)-x(2,index-1), 0];
            % direction_norm = norm(direction);
            % u_lim = 5*obj.params.dt;
            % u = direction./direction_norm;
            % u = u_lim.*u;
            % u = u';

        end

        
        function distance = Distance_RobotDrone(obj,x)
            %DISTANCE_ROBOTDRONE Relative distance between each pair
            %robot/drone
            % This function calculates the relative distance between the
            % moving target and each drone in the environment
            %  Input:
            %   obj (DRONE):           Drone object
            %   x (3x1 double):        Robot current state
            %  Output:
            %   distance (1x1 double): Euclidean distance between pair robot/drone

            distance = sqrt((x(1)-obj.q_real(1))^2+(x(2)-obj.q_real(2))^2+(x(3)-obj.q_real(3))^2);

        end


        function obj = ComputeMeasNoiseMatrix(obj)
            % COMPUTEMEASNOISEMATRIX Measurement noise matrix
            % This function calculates the noise on the TDOA measurement 
            % model of each drone based on its number of neighboring drones
            %  Input:
            %   obj (DRONE):                    Drone object

            % obj.R = (rand(obj.N_neighbors)-0.5)*obj.params.std_drones;
            % obj.R = obj.R*obj.R';
            obj.R = ones(obj.N_neighbors)*obj.params.std_drones^2;
            obj.R = obj.R - (obj.params.std_drones^2)/2 * (ones(obj.N_neighbors)-eye(obj.N_neighbors));

        end

        function measurement_noise = MeasurementNoise(obj)
            %MEASUREMENTNOISE Noise on the TDOA measurement model
            % This function defines the noise on the TDOA measurement model of each drone
            %  Input:
            %   obj (DRONE):                    Drone object
            %  Output:
            %   measurement_noise (Nx1 double): Noise on TDOA measurements
            
            if obj.Connection == "on"
                measurement_noise = mvnrnd(zeros(size(obj.R,1),1),obj.R);
            end

        end


        function drone = PlotDrone(obj,state)
            %PLOTDRONE Plot the drone
            %  This function plots the drone in the workspace
            %  Input:
            %   obj (DRONE): Drone object
            %  Output:
            %   state(3x1):  State of the drone
            
            if nargin == 1
                % State of the drone
                X = obj.q_real;
            else
                X = state;
            end
            X = reshape(X,[1,3]);
            % Drone radius
            r = obj.params.DRONE_RADIUS;

            % Drone Visualization
            th = linspace(0,2*pi,100); % Angle samples for the visualization of drone body

            % Drone body
            body = patch('XData', X(1,1) + r*cos(th),'YData', X(1,2) + r*sin(th), 'ZData', repmat(X(3),[1,100]), 'FaceColor',  'red');

            % Return the handle 
            drone = body;

        end

    end
end
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
        x_est               % [m;m;m]       (3xiterations double)            Estimated state of the robot by the drone from the consensus
        x_est_EKF           % [m;m;m]       (3xiterations double)            Estimated state of the robot by the drone from the EKF

        % Error between estimate and robot's real position
        Delta_x             % [m;m;m]       (3xiterations double)            Error in drone's estimate

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

                    % Initialize the initial estimate of the robot 
                    obj.x_est = zeros(3,params.max_iter+1);
                    obj.x_est_EKF = zeros(3,params.max_iter+1);
                    % Initialize the initial error of the robot's estimated
                    % position by the drone
                    obj.Delta_x = zeros(3,params.max_iter+1);

                    % Initialize the process noise
                    obj.Q = (randn(3,3)-0.5)*params.std_dyn_xyz;
                    obj.Q = obj.Q*obj.Q';

                    % Initialize number of neighbor drones
                    obj.N_neighbors = params.N_agents-1;

                    % Initialize neighboring drones
                    obj.neighbors = zeros(obj.N_neighbors,1);

                    % TDOA measurement model noise
                    obj.R = (rand(obj.N_neighbors)-0.5)*params.std_drones;
                    obj.R = obj.R*obj.R';

                    % Drone on/off
                    obj.Connection = cell(1,params.max_iter);

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
                    % x_new = x_old + vx*dt + wx
                    % y_new = y_old + vy*dt + wy
                    % z_new = z_old + wz

                    % True Dynamics with noise
                    q_old = obj.q_real;
                    x_old = q_old(1);
                    y_old = q_old(2);
                    z_old = q_old(3);
                    A = [1 0 0;0 1 0;0 0 1];
                    B = [obj.params.dt 0 0;0 obj.params.dt 0;0 0 obj.params.dt];
                    q_new = A*q_old + B*u + mvnrnd(zeros(3,1),obj.Q)';

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
            %   index (1x1 int):  Iteration number
            %  Output:
            %   u (3x1 double):   Control input
            
            % Check if index is within bounds
            if index <= 1
                error('Index must be greater than 1 to compute direction.');
            end

            %% The drones move along the direction of two consecutive robot's state estimates
            direction = [x(1,index)-x(1,index-1), x(2,index)-x(2,index-1), 0];
            direction_norm = norm(direction);

            % Limiting velocity
            u_lim = 2;
            % Move up with velocity u_z if the height becomes less than a threshold
            z_min = 1;
            u_z = 2;
            if strcmp(obj.Connection{index},'on') == 1 % if the drone has updated estimates use them to compute controls; check for its height: if it becomes lower than the threshold, give a positive velocity in z
                if obj.q_real(3) < z_min
                    u = direction./direction_norm;
                    u = [u_lim.*u(1:2),u_z];
                    u = u';
                else
                    u = direction./direction_norm;
                    u = u_lim.*u;
                    u = u';
                end
            else % if the drone has no updated estimates give zero velocity commands, unless its height becomes lower than the threshold
                if obj.q_real(3) < z_min
                    u = [0;0;u_z];
                else
                    u = [0;0;0];
                end
            end

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
            %   obj (DRONE):  Drone object

            obj.R = (rand(obj.N_neighbors)-0.5)*obj.params.std_drones;
            obj.R = obj.R*obj.R';

        end

        function measurement_noise = MeasurementNoise(obj,i)
            %MEASUREMENTNOISE Noise on the TDOA measurement model
            % This function defines the noise on the TDOA measurement model of each drone
            %  Input:
            %   obj (DRONE):                    Drone object
            %  Output:
            %   measurement_noise (Nx1 double): Noise on TDOA measurements
            
            if obj.Connection{i} == "on"
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

        function drone = PlotDrone2(obj,state)
            %PLOTDRONE Plot the drone
            %  This function plots the drone in the workspace
            %  Input:
            %   obj (DRONE): Drone object
            %  Output:
            %   state(3x1):  State of the drone
            
            % Plot the drone structure (a simple quadcopter design)
            % Initial position at x(1), y(1)
            if nargin == 1
                % State of the agent
                x=obj.q_real(1);
                y=obj.q_real(2);
            else
                x=state(1);
                y=state(2);
            end
            % Scaling factor (adjust this value to scale the size of the drone)
            scale = 2.0;
            
            
            % Body of the drone (center circle)
            body = rectangle('Position', [x(1)-0.3*scale, y(1)-0.3*scale, 0.6*scale, 0.6*scale], 'Curvature', [1, 1], ...
                             'FaceColor', 'b', 'EdgeColor', 'k');
            
            % Arms of the drone
            arm1 = plot([x(1)-0.5*scale, x(1)+0.5*scale], [y(1), y(1)], 'k', 'LineWidth', 2);
            arm2 = plot([x(1), x(1)], [y(1)-0.5*scale, y(1)+0.5*scale], 'k', 'LineWidth', 2);
            
            % Propellers of the drone (small circles at the ends of the arms)
            prop1 = rectangle('Position', [x(1)-0.55*scale, y(1)-0.55*scale, 0.2*scale, 0.2*scale], 'Curvature', [1, 1], ...
                              'FaceColor', 'r', 'EdgeColor', 'k');
            prop2 = rectangle('Position', [x(1)+0.35*scale, y(1)-0.55*scale, 0.2*scale, 0.2*scale], 'Curvature', [1, 1], ...
                              'FaceColor', 'r', 'EdgeColor', 'k');
            prop3 = rectangle('Position', [x(1)-0.55*scale, y(1)+0.35*scale, 0.2*scale, 0.2*scale], 'Curvature', [1, 1], ...
                              'FaceColor', 'r', 'EdgeColor', 'k');
            prop4 = rectangle('Position', [x(1)+0.35*scale, y(1)+0.35*scale, 0.2*scale, 0.2*scale], 'Curvature', [1, 1], ...
                              'FaceColor', 'r', 'EdgeColor', 'k');
            
            % Loop through all coordinates to animate the drone's movement
            for k = 1:length(x)
                % Update the drone's position
                set(body, 'Position', [x(k)-0.3*scale, y(k)-0.3*scale, 0.6*scale, 0.6*scale]);
                set(arm1, 'XData', [x(k)-0.5*scale, x(k)+0.5*scale], 'YData', [y(k), y(k)]);
                set(arm2, 'XData', [x(k), x(k)], 'YData', [y(k)-0.5*scale, y(k)+0.5*scale]);
                set(prop1, 'Position', [x(k)-0.55*scale, y(k)-0.55*scale, 0.2*scale, 0.2*scale]);
                set(prop2, 'Position', [x(k)+0.35*scale, y(k)-0.55*scale, 0.2*scale, 0.2*scale]);
                set(prop3, 'Position', [x(k)-0.55*scale, y(k)+0.35*scale, 0.2*scale, 0.2*scale]);
                set(prop4, 'Position', [x(k)+0.35*scale, y(k)+0.35*scale, 0.2*scale, 0.2*scale]);
            
                % Update the plot
                drawnow;
                
                % Pause to control the speed of animation
                pause(0.1);  % Adjust the pause time as needed
            end
            drone = [body,arm1,arm2,prop1,prop2,prop3,prop4];
        end

    end
end
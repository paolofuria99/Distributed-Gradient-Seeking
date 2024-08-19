%% Initialization
init

%% Get parameters
parameters

%% Simulation parameters and simulation initialization
dt = params.dt;                     % [s] - time step
iterations = params.max_iter;       % [#] - number of iteration
T = iterations/dt;                  % [s] - simulation duration
ANIMATE=0;
PLOT_ERROR_ELLIPSES=0;

% Environment/Field definition
environment;

% According to Matveev paper, make hypotesis regarding the field and check
% robot parameters
R_star=3;                         % [m] - Desired distance of arrival from the maximum point of the field
q_minus=1;                        % [-] - Lower bound of the intensity of the field
sigma2_minus = 100;               % [m^2] - Lower limit of the field variance
sigma2_plus = 500;                % [m^2] - Upper limit of the field variance
R_est = 9;                        % [m] - Estimated started distance from the field center
checkMatveevCond;                 % Checking Matveev parameters

fprintf('\nPress any key to continue or "q" to quit:');
keyPressed = input('', 's');  % Wait for key press

% Check if the user pressed 'q' or any other key
if strcmpi(keyPressed, 'q')
    disp('Execution stopped.');
    return;  % Stops the execution of the script
else
    disp('Continuing execution...');
end


% Create a robot agent with the initial position and orientation;
robot = AGENT(params.q0,1,'unicycle',params);

% Arrays to store outputs for plotting
time = zeros(1, iterations);
q_real_vals = zeros(3, iterations);     % [m, m, rad] - Real pose
q_est_vals = zeros(3, iterations);      % [m, m, rad] - Estimated pose
v_vals = zeros(1, iterations);          % [m/s] - Linear Velocity values
w_vals = zeros(1, iterations);          % [rad/s] - Angular Velocity values
D_vals = zeros(1, iterations);          % [-] - Field measured values
k_vals = zeros(1, iterations);          % [-] - Field Curvature values
P_vals = cell(1, iterations);           % [-] - Covariance values

%% Main Loop Simulation
for i = 1:iterations
    disp(['iteration[', num2str(i),']']);
    t=(i-1)*dt;
    time(i)=t;
    
    % Save parameters for plots
    q_est_vals(:,i) = robot.q_est;
    q_real_vals(:,i) = robot.q_real;
    D_vals(i) = robot.D_new;
    k_vals(i) = curvature(Z,[robot.q_real(1),robot.q_real(2)],x_range,y_range);
    P_vals{i} = robot.P;


    
    %% Start loop
    % High level control -  Calculate u that depend on the field value
    u = robot.compute_control();
    % Save control vals
    v_vals(i)=u(1); w_vals(i)=u(2);
    
    %TODO: Richiamare qua la funzione per il calcolo della posizione del robot tramite triangolazione.
    % si sostituirà quindi SOLO la funzione agent.gps_measurement() dentro a EKF, in modo tale che vada a sostituire la misura attuale del GPS

    % Low level control - Actuate the robot with the high level control and update the state based on the system state and GPS measurement
    EKF(robot, u);

    %% End Loop

    % Stop the simulation if the robot reaches the peak
    if norm([params.x_max_peak;params.y_max_peak]'-q_est_vals(1:2,i)')<R_star
        iter_break = i;
        break;
    end
end
iter_break = i;

%% Plotting results
plot_simulation;


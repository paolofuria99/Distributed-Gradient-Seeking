%% Initialization
init

%% Get parameters
parameters
params = sim_param;

%% Simulation parameters and simulation initialization
dt = params.dt;
iterations = params.max_iter;
T = iterations/dt;
ANIMATE = 0;

% Environment/Field definition
environment;

% Create a robot agent with the initial position and orientation;
robot = AGENT(params.q0,1,'unicycle',params);

% Arrays to store outputs for plotting
time = zeros(1, iterations);
q_real_vals = zeros(3, iterations);
q_est_vals = zeros(3, iterations);
v_vals = zeros(1, iterations);
w_vals = zeros(1, iterations);
D_vals = zeros(1, iterations);

%% Main Loop Simulation
for i = 1:iterations
    t=(i-1)*dt;
    time(i)=t;
    
    % Save parameters for plots
    q_est_vals(:,i) = robot.q_est;
    q_real_vals(:,i) = robot.q_real;
    D_vals(i) = robot.D_new;
    
    % High level control -  Calculate u that depend on the field value
    u = robot.compute_control();
    
    %TODO: Richiamare qua la funzione per il calcolo della posizione del robot tramite triangolazione.
    % si sostituirà quindi SOLO la funzione agent.gps_measurement() dentro a EKF, in modo tale che vada a sostituire la misura attuale del GPS

    % Low level control - Actuate the robot with the high level control and update the state based on the system state and GPS measurement
    EKF(robot, u);

    % Stop the simulation if the robot reaches the peak
    if norm([-20;-18]'-q_est_vals(1:2,i)')<1.5
        iter_break = i;
        break;
    end
end
iter_break = i;

plot_simulation;


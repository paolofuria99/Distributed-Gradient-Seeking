%% Initialization
init

%% Get parameters
parameters
params = sim_param;
ANIMATE = 1;

%% Environment definition

% Define the Gaussian field
x_range = linspace(-params.ENV_SIZE, params.ENV_SIZE, 200);  % X-axis range for the workspace
y_range = linspace(-params.ENV_SIZE, params.ENV_SIZE, 200);  % Y-axis range for the workspace
[X, Y] = meshgrid(x_range, y_range);  % Meshgrid for the workspace
% Gaussian distribution 
x_peak = -20;
y_peak = -12;
%D = @(x,y) 10 * exp(-((x - x_peak).^2 + (y - y_peak).^2) / (2*300));
D = @(x,y) 18 * exp(-((x - x_peak).^2 + (y - y_peak).^2) / 600) - 10*exp(-((x-10).^2+(y-8).^2)/200);
Z=D(X,Y);
[DX,DY]= gradient(Z);
k = curvature(Z);

% Define the unicycle's initial position and orientation
q0 = [0, 20, deg2rad(30)];

% Create a robot agent;
robot = AGENT(q0,1,'unicycle',params);

% Simulation parameters
dt = params.dt;
iterations = params.max_iter;
T = iterations/dt;

% Algorithm parameters
dv = 5*dt;                          
v_star= 0.05;
dw = @(dd) (3*dt*sign(dd -v_star));

% Arrays to store outputs for plotting
time = zeros(1, iterations);
q_vals = zeros(3, iterations);
v_vals = zeros(1, iterations);
w_vals = zeros(1, iterations);
D_vals = zeros(1, iterations);

%% Main Loop Simulation

for i = 1:iterations
    t=(i-1)*dt;
    time(i)=t;
    
    % Current state
    q_now = robot.q_real;
    q_vals(:,i)=q_now;
    % Current field value
    D_now = D(q_now(1), q_now(2));
    D_vals(i)=D_now;
    if i==1
        D_dot = D_now;
    else
        D_dot= D_vals(i)-D_vals(i-1);
    end

    % Actuate the robot
    robot.dynamics([dv;dw(D_dot)]);

    if norm([x_peak;y_peak]'-q_vals(1:2,i)')<1.5
        iter_break = i;
        break;
    end
end
iter_break = i;

% Animate the simulation
if ANIMATE
    figure;
    hold on;
    axis([-params.ENV_SIZE, params.ENV_SIZE, -params.ENV_SIZE,  params.ENV_SIZE]);
    xlabel('X');
    ylabel('Y');
    title('Unicycle Robot Gradient Descent to Find Gaussian Peak');
    grid on;
    %contourf(X, Y, D(X,Y), 50, 'LineStyle', 'none');
    contour(X, Y, Z, 30);
    colorbar;
    for i = 1:length(time(1:iter_break))
        agent=robot.PlotAgent(q_vals(:,i));
        plot(q_vals(1, 1:i), q_vals(2, 1:i), 'b-', 'LineWidth', 2);
        pause(dt);
        if ishandle(agent)
            delete(agent);
        end
    end
    
end

% Plot results
figure
ax1=subplot(3, 1, 1);
plot(time(1:iter_break), q_vals(1,1:iter_break), 'LineWidth', 1.2)
legend('$x$', 'Interpreter', 'latex', 'Location', 'NorthWest')
grid on
xlabel('Time [s]')
ylabel('X Coordinate')

ax2=subplot(3, 1, 2);
plot(time(1:iter_break), q_vals(2,1:iter_break), 'LineWidth', 1.2)
legend('$y$', 'Interpreter', 'latex', 'Location', 'NorthWest')
grid on
xlabel('Time [s]')
ylabel('Y Coordinate')

ax3=subplot(3, 1, 3);
plot(time(1:iter_break), q_vals(3,1:iter_break), 'LineWidth', 1.2)
legend('$\theta$', 'Interpreter', 'latex', 'Location', 'NorthWest')
grid on
xlabel('Time [s]')
ylabel('$\theta$ Coordinate')

set(gcf, 'Position', [100 100 500 350])
set(gcf, 'PaperPositionMode', 'auto')
linkaxes([ax1,ax2,ax3],'x')


% Additional visualization: plot trajectory on the scalar field
figure
hold on
contour(X, Y, Z, 30)
colorbar
%quiver(X,Y,DX,DY,0)
plot(q_vals(1,1:iter_break), q_vals(2,1:iter_break), 'r-', 'LineWidth', 1.5, 'DisplayName','Robot Trajectory')
xlabel('X Coordinate')
ylabel('Y Coordinate')
title('Unicycle Robot Trajectory with ESC')
grid on



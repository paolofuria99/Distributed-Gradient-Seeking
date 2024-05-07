%% Initialization
init

%% Get parameters
parameters
params = sim_param;

%% Environment definition

% Define the Gaussian field
x_range = linspace(-params.ENV_SIZE, params.ENV_SIZE, 200);  % X-axis range for the workspace
y_range = linspace(-params.ENV_SIZE, params.ENV_SIZE, 200);  % Y-axis range for the workspace
[X, Y] = meshgrid(x_range, y_range);  % Meshgrid for the workspace
% Gaussian distribution 
x_peak = -20;
y_peak = -12;
D = @(x,y) 10 * exp(-((x - x_peak).^2 + (y - y_peak).^2) / (2*300));
%D = @(x,y) 18 * exp(-((x - x_peak).^2 + (y - y_peak).^2) / 600) - 10*exp(-((x-10).^2+(y-8).^2)/200);
Z=D(X,Y);
[DX,DY]= gradient(Z);

% Define the unicycle's initial position and orientation
q0 = [0, 20, deg2rad(30)];

% Create an agent;
robot = AGENT(q0,1,'unicycle',params);

%% TODO: Questo parametro non servirebbe, da togliere in futuro
% Define simulation parameters
learning_rate = 0.3;  % Learning rate for movement

% Create a figure for animation
figure();
hold on
axis([-params.ENV_SIZE, params.ENV_SIZE, -params.ENV_SIZE,  params.ENV_SIZE]);
xlabel('X');
ylabel('Y');
title('Unicycle Robot Gradient Descent to Find Gaussian Peak');
grid on;

% Plot the field and the goal point
%contourf(X, Y, D(X,Y), 50, 'LineStyle', 'none');
contour(X, Y, D(X,Y), 30);
colorbar;
%quiver(X,Y,DX,DY);

% Initialize the robot's path
x_path = zeros(2, params.max_iter);
x_path(:, 1) = robot.q_est(1:2);

% Create a robot object for animation
agent = robot.PlotAgent();

%% Da qua in poi è SBAGLIATO usato solo per vedere se viene fuori qualcosa
% Tutto questo calcolo dovrebbe essere messo ad agent  e calcolare un
% azione in base alla misura del campo
dv = 0.5;
v_star= 0.01;
R=5;
Rstar=3;
dw = @(dd) (0.3*sign(dd -v_star));
d=[0];

% Simulate the robot's movement
for i = 2:params.max_iter
    % Current position of the robot
    current_position = robot.q_real;

    d_new = D(current_position(1),current_position(2));
    d=[d d_new];
    d_dot = d(end)-d(end-1);

    disp(['dd:', num2str(d(end))])
    disp(['dw:', num2str(dw(d_dot))])
    q_d= robot.dynamics([dv; dw(d_dot)]);
    q_d = robot.q_real;
    x_path(:,i)=q_d(1:2);

    % Plot the robot's new position
    agent =robot.PlotAgent();
    
    % Break the loop if the robot is close to the peak
    if norm([x_peak;y_peak]' - x_path(:, i)') < 2
        break;
    end
    
    % Pause for animation
    pause(params.dt);
    
    % Delete the previous robot position
    if ishandle(agent)
        delete(agent);
    end
end

% Display the final path
plot(x_path(1, 1:i), x_path(2, 1:i), 'b-', 'LineWidth', 2);

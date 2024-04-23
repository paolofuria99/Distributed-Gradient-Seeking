%% Initialization
init;

%% Get params
params = sim_param;

%% Environment definition

% Define the Gaussian field
x_range = linspace(-params.ENV_SIZE, params.ENV_SIZE, 100);  % X-axis range for the workspace
y_range = linspace(-params.ENV_SIZE, params.ENV_SIZE, 100);  % Y-axis range for the workspace
[X, Y] = meshgrid(x_range, y_range);  % Meshgrid for the workspace
% Gaussian distribution 
x_peak = -20;
y_peak = -12;
%D = @(x,y) 10 * exp(-((x - x_peak).^2 + (y - y_peak).^2) / 600);
D = @(x,y) 18 * exp(-((x - x_peak).^2 + (y - y_peak).^2) / 200) + 10*exp(-((x-10).^2+(y-8).^2)/600);

% Define the unicycle's initial position and orientation
q0 = [-40, 20, deg2rad(10)];

% Create an agent;
robot = AGENT(q0,1,'unicycle',params);

%% TODO: Questo parametro non servirebbe, da togliere in futuro
% Define simulation parameters
learning_rate = 0.1;  % Learning rate for movement

% Create a figure for animation
figure();
hold on
axis([-params.ENV_SIZE, params.ENV_SIZE, -params.ENV_SIZE,  params.ENV_SIZE]);
xlabel('X');
ylabel('Y');
title('Unicycle Robot Gradient Descent to Find Gaussian Peak');
grid on;

% Plot the field and the goal point
contourf(X, Y, D(X,Y), 50, 'LineStyle', 'none');
colorbar;

% Initialize the robot's path
x_path = zeros(2, params.max_iter);
x_path(:, 1) = q0(1:2)';

% Create a robot object for animation
agent=[];
agent = robot.PlotAgent();

%% Da qua in poi è SBAGLIATO usato solo per vedere se viene fuori qualcosa
% Tutto questo calcolo dovrebbe essere messo ad agent  e calcolare un
% azione in base alla misura del campo

% Simulate the robot's movement
for i = 2:params.max_iter
    % Current position of the robot
    current_position = x_path(:, i - 1);
    
    % Calculate the gradient by sampling nearby field values
    delta = 0.1;  % Small step to estimate gradient
    d_center = D(current_position(1), current_position(2));
    d_dx = D(current_position(1) + delta, current_position(2));
    d_dy = D(current_position(1), current_position(2) + delta);

    % Approximate the gradient
    gradient = [d_dx - d_center; d_dy - d_center] / delta;

    % Normalize the gradient
    if norm(gradient) > 0
        gradient = gradient / norm(gradient);
    end

    % Update the robot's orientation
    theta = atan2(gradient(2), gradient(1));
    
    % Move the robot in the gradient's direction
    x_path(:, i) = current_position + learning_rate * gradient;
    
    % Plot the robot's new position
    robot.x_real = [x_path(1,i);x_path(2,i)];
    robot.th_real = theta;
    agent =robot.PlotAgent();
    
    % Break the loop if the robot is close to the peak
    if norm([x_peak;y_peak]' - x_path(:, i)') < 0.8
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

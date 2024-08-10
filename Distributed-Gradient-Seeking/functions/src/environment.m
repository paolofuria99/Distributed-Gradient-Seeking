% Define the field range
x_range = linspace(-params.ENV_SIZE, params.ENV_SIZE, params.ENV_STEP);  % X-axis range for the workspace
y_range = linspace(-params.ENV_SIZE, params.ENV_SIZE, params.ENV_STEP);  % Y-axis range for the workspace
[X, Y] = meshgrid(x_range, y_range);  % Meshgrid for the workspace

% Field Gaussian distribution 
D = params.D;
Z = D(X,Y);

% Usefull parameters of the field
[DX,DY]= gradient(Z);
k = curvature(Z);
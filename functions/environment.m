% Define the field range
x_range = linspace(-params.ENV_SIZE, params.ENV_SIZE, params.ENV_STEP);  % X-axis range for the workspace
y_range = linspace(-params.ENV_SIZE, params.ENV_SIZE, params.ENV_STEP);  % Y-axis range for the workspace
[X, Y] = meshgrid(x_range, y_range);  % Meshgrid for the workspace

% Field Gaussian distribution 
D = params.D;
Z = D(X,Y);

z_range = D(x_range,y_range);

% Usefull parameters of the field
[DX,DY] = gradient(Z);

% Curvature of the field
k = curvature(Z);

% Find local maxima in both dimensions
TF = islocalmax(Z, 1) & islocalmax(Z, 2); 
[row, col] = find(TF); 
Dx_peaks = X(row, col);
Dy_peaks = Y(row, col);
Dz_peaks = Z(row, col);




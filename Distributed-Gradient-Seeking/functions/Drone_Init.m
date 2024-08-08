function q0 = Drone_Init(params)
%DRONE_INIT Initialize the position of the drones within the environment
%   Detailed explanation goes here

% Center of environment
xc = [0;0;0];

% Generate a random angle
angle = 2*pi*rand;
% Generate a random distance
distance = params.radius*sqrt(rand);
    
% Convert polar coordinates to Cartesian coordinates
x0 = xc(1) + distance * cos(angle);
y0 = xc(2) + distance * sin(angle);
z0_max = 3; % [m] Maximum height for each drone
z_offset = 2; % [m] Maximum deviation from z0_max
z0 = xc(3) + z0_max - rand*z_offset;

q0 = [x0,y0,z0];

end


function q0 = Drone_Init(params)
%DRONE_INIT Initialize the position of the drone
%   Detailed explanation goes here

% Generate a random angle
angle = 2*pi*rand;
% Generate a random distance
distance = params.radius*sqrt(rand);
    
% Convert polar coordinates to Cartesian coordinates
x0 = params.q0(1) + distance * cos(angle);
y0 = params.q0(2) + distance * sin(angle);
z0_max = 3; % [m] Maximum height for each drone
z_offset = 2; % [m] Maximum deviation from z0_max
z0 = z0_max - rand*z_offset;

q0 = [x0,y0,z0];

end


function drone_pose = Drone_Init(params)
%DRONE_INIT Initialize the position of the drones within the environment
% INPUTS:
% params -     Parameters of the simulation
% OUTPUTS:
% drone_pose - Matrix with initial position of the drones

% Center of environment
xc = [0;0;0];
% [m] Radius of circular region
r = params.radius;
% Number of drones within the environment
n = params.N_agents;
% Minimum distance between drones
min_distance = 20;

% Matrix to store drone positions
drone_pose = zeros(n,3);
for idx = 1:n
    initialization = false;
    while ~initialization
        angle = 2*pi*rand;       % Generate a random angle
        distance = r*sqrt(rand); % Generate a random distance
        % Convert polar coordinates to Cartesian coordinates
        x0 = xc(1) + distance * cos(angle);
        y0 = xc(2) + distance * sin(angle);
        z0 = xc(3) + rand*params.ENV_SIZE/3;

        if idx == 1
            initialization = true;
        else
            distance = sqrt((drone_pose(1:idx-1,1)-x0).^2 + (drone_pose(1:idx-1,2)-y0).^2 + ...
                            (drone_pose(1:idx-1,3)-z0).^2);
            if all(distance >= min_distance)
                initialization = true;
            end
        end
    end
    % Save xyz-coordinates inside drone_pose
    drone_pose(idx,:) = [x0,y0,z0];
end

disp('All drones are initialized within the environment!');

end


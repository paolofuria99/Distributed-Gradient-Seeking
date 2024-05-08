function estimated_position = multilateration(params, distance, position)
%MULTILATERATION Estimate the position of the ground vehicle

% Define variables
syms x y;
% Initialize counter    
count = 1; 
for i = 1:params.N_drones
    for j = i+1:params.N_drones
        eq(count) = distance(count) == sqrt((x - position(j, 1))^2 + (y - position(j, 2))^2) - ...
                                       sqrt((x - position(i, 1))^2 + (y - position(i, 2))^2);
        count = count + 1;
    end
end
sol = solve(eq(1:end-1), [x, y]);
estimated_position = [double(sol.x), double(sol.y)];
end
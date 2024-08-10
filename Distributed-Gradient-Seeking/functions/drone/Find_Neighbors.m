function neighbors = Find_Neighbors(params,drone)
%FIND_NEIGHBORS This function finds all neighbors for each drone based on
%their relative distance
%   Detailed explanation goes here

% Number of drones within the environment
n = params.N_agents;
% [m] Define radius to find neighbors based on the environment and number
% of drones within it
r = params.radius/2 + randi([params.N_min_agents params.N_agents]);
% Variable to store neighbors
neighbors = cell(n,1);

for idx = 1:n
    for j = 1:n
        if idx ~= j
            distance = sqrt((drone(idx).q_real(1)-drone(j).q_real(1))^2 + ...
                            (drone(idx).q_real(2)-drone(j).q_real(2))^2 + ...
                            (drone(idx).q_real(3)-drone(j).q_real(3))^2);
            if distance < r
                neighbors{idx} = [neighbors{idx},j];
            end
        end
    end
end

end


function neighbors = Find_Neighbors(params,drone)
%FIND_NEIGHBORS Summary of this function goes here
%   Detailed explanation goes here

% Number of drones within the environment
n = params.N_agents;
% [m] Define radius to find neighbors
r = params.radius/2 + randi([params.N_min_agents params.N_agents]);
assignin("base","r",r)
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


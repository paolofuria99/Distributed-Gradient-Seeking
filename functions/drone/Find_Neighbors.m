function neighbors = Find_Neighbors(params,drone)
%FIND_NEIGHBORS This function finds all neighbors for each drone based on
%their relative distance
% INPUTS:
% params -    Parameters of the simulation
% drone -     Drones classes
% OUTPUTS:
% neighbors - Cell with list of neighbors for each drone

% Number of drones within the environment
n = params.N_agents;
% [m] Define radius to find neighbors
r = 40;
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


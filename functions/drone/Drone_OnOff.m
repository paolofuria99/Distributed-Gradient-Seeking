function Drone_OnOff(idx,drone,i)
%DRONE_ONOFF Disable the functioning of all drones that do not communicate 
% with other drones
% INPUTS:
% idx -   Drone ID number
% drone - Drones classes
% i -     Iteration number

% Flag to enable print of disconnected drones: 1 to print; 0 to do nothing
PRINT_DISCONNECTIONS = 0;

if drone(idx).N_neighbors == 0
    drone(idx).Connection{i} = "off";
    if PRINT_DISCONNECTIONS
        fprintf("Drone %d lost connection!!\n",idx);
    end
else
    drone(idx).Connection{i} = "on";
end

end


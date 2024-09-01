function Drone_OnOff(idx,drone,i)
%DRONE_ONOFF Disable the functioning of all drones that do not communicate 
% with other drones
% INPUTS:
% idx -   Drone ID number
% drone - Drones classes
% i -     Iteration number

if drone(idx).N_neighbors == 0
    drone(idx).Connection{i} = "off";
    fprintf("Drone %d lost connection!!\n",idx);
else
    drone(idx).Connection{i} = "on";
end

end


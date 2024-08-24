function Drone_OnOff(idx,drone,i)
%DRONE_ONOFF This function disables the functioning of all drones that do
%not communicate with other drones
% INPUTS:
% idx -   Drone ID number
% drone - Drones classes

if drone(idx).N_neighbors == 0
    drone(idx).Connection{i} = "off";
    fprintf("Drone %d lost connection!!\n",idx);
else
    drone(idx).Connection{i} = "on";
end

end


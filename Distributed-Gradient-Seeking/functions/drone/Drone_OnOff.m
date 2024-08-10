function Drone_OnOff(idx,drone)
%DRONE_ONOFF Summary of this function goes here
%   Detailed explanation goes here

if drone(idx).N_neighbors == 0
    drone(idx).Connection = "off";
else
    drone(idx).Connection = "on";
end

end


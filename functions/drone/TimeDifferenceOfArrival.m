function h = TimeDifferenceOfArrival(idx,x,drone)
%TIMEDIFFERENCEOFARRIVAL Calculate the Time Difference Of Arrival between each pair of
%neighboring drones
% INPUTS:
% idx -   Drone ID number
% x -     Robot's state
% drone - Drones classes
% OUTPUTS:
% h -     TDOA measurements

for i = 1:drone(idx).N_neighbors
    h(1,i) = drone(idx).Distance_RobotDrone(x) - drone(drone(idx).neighbors(i)).Distance_RobotDrone(x);
end

end

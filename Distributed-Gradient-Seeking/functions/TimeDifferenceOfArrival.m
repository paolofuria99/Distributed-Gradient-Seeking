function h = TimeDifferenceOfArrival(idx,params,x,drone)
%TIMEDIFFERENCEOFARRIVAL Calculate the Time Difference Of Arrival between each pair of
%drone
%   Detailed explanation goes here

for i = 1:params.N_agents
    h(1,i) = drone(idx).Distance_RobotDrone(x) - drone(i).Distance_RobotDrone(x);
end

end

function h = TimeDifferenceOfArrival(idx,params,x,drone)
%TIMEDIFFERENCEOFARRIVAL Calculate the Time Difference Of Arrival between each pair of
%drone
%   Detailed explanation goes here

% for i = 1:params.N_agents
%     if i ~= idx
%         h(1,i) = drone(idx).Distance_RobotDrone(x) - drone(i).Distance_RobotDrone(x);
%     end
% end

j = 1;
for i = 1:params.N_agents
    if i ~= idx
        h(1,j) = drone(idx).Distance_RobotDrone(x) - drone(i).Distance_RobotDrone(x);
        j=j+1;
    end
end

end

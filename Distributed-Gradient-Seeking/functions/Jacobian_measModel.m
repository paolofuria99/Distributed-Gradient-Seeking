function H = Jacobian_measModel(idx,params,drone,x)
%JACOBIAN_MEASMODEL Summary of this function goes here
%   Detailed explanation goes here

for i = 1:params.N_agents
    for j = 1:3
        H(i,j) = (x(j)-drone(idx).q_real(j))/drone(idx).Distance_RobotDrone(x) - ...
                 (x(j)-drone(i).q_real(j))/drone(i).Distance_RobotDrone(x);
    end
end

end


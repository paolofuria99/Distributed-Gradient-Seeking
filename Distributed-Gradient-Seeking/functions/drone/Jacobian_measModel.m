function H = Jacobian_measModel(idx,drone,x)
%JACOBIAN_MEASMODEL Summary of this function goes here
%   Detailed explanation goes here

for i = 1:drone(idx).N_neighbors
    for j = 1:3
        H(i,j) = (x(j)-drone(idx).q_real(j))/drone(idx).Distance_RobotDrone(x) - ...
                 (x(j)-drone(drone(idx).neighbors(i)).q_real(j))/drone(drone(idx).neighbors(i)).Distance_RobotDrone(x);
    end
end

end


function H = Jacobian_measModel(idx,drone,x)
%JACOBIAN_MEASMODEL Calculate the Jacobian of drones' measurement model to
%linearize the non-linear model around the state of the robot
% INPUTS:
% idx -   Drone ID number
% drone - Drones classes    
% x -     Robot's state
% OUTPUTS:
% H -     Jacobian of measurement model

for i = 1:drone(idx).N_neighbors
    for j = 1:3
        H(i,j) = (x(j)-drone(idx).q_real(j))/drone(idx).Distance_RobotDrone(x) - ...
                 (x(j)-drone(drone(idx).neighbors(i)).q_real(j))/drone(drone(idx).neighbors(i)).Distance_RobotDrone(x);
    end
end

end


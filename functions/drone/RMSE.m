function diff = RMSE(x,y,diff)
%RMSE Calculate the square of errors between real pose and estimated pose
% INPUTS:
% x -    Real pose of the robot
% y -    Estimated pose of the robot by the drones
% diff - RMSE counter
% OUTPUTS:
% diff - RMSE counter

diff_i = (x(1)-y(1))^2 + (x(2)-y(2))^2 + (0-y(3))^2;
diff = diff + diff_i;

end


function diff = RMSE(x,y,diff)
%RMSE Summary of this function goes here
%   Detailed explanation goes here

diff_i = (x(1)-y(1))^2 + (x(2)-y(2))^2;
diff = diff + diff_i;

end


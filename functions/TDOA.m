function [TDOA_measurements, difference_in_distance] = TDOA(params, robot, position)
% TDOA Compute Time Difference of Arrival

distances = zeros(params.N_drones, 1);
reception_times = zeros(params.N_drones, 1);
for i = 1:params.N_drones
    distances(i) = sqrt((position(i, 1) - robot.x_est(1))^2 + (position(i, 2) - robot.x_est(2))^2 + ...
                        (position(i, 3) - 0)^2);
    reception_times(i) = distances(i)/params.c;
end
TDOA_measurements = zeros(nchoosek(params.N_drones, 2), 1); % Initialize array to store TDOA measurements
count = 1;                                                      % Initialize count for storing TDOA measurements
for i = 1:params.N_drones
    for j = i+1:params.N_drones
        % Compute TDOA measurement for pair (i, j)
        TDOA_measurements(count) = reception_times(j) - reception_times(i);
        count = count + 1;
    end
end

% Convert TDOA into a difference in distances
difference_in_distance = TDOA_measurements * params.c;
end
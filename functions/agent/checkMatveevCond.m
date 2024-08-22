%% init

tolerance = 1e-2;
x_max = params.x_max_peak; % x-coordinate of the maximum point
y_max = params.y_max_peak; % y-coordinate of the maximum point
% Chosen linear and angular velocities
v = params.MAX_LIN_VEL/params.dt; 
w = params.MAX_ANG_VEL/params.dt; 
% Robot maximum turning radius
R = v/w;                          

%% Calculate gamma* and R**

% Define points on the circle of radius R* around r0
theta = linspace(0, 2*pi, 100);
circle_points = [x_max;y_max]' + R_star * [cos(theta); sin(theta)]';

% Calculate gamma_star
gamma_star = max(arrayfun(@(i) D(circle_points(i, 1), circle_points(i, 2)), 1:length(theta)));

% Find R**
% find all the indexes that satisfies D(r)=gamma_star
matching_indices = find(abs(Z - gamma_star) < tolerance);

% Calculate distances from r0 to these points
distances = sqrt((X(matching_indices) - x_max).^2 + (Y(matching_indices) - y_max).^2);

% Find the minimum distance
R_star_star = min(distances);

% Output the results
%fprintf('gamma_star (γ*) = %.4f\n', gamma_star);
%fprintf('R_star_star (R**) = %.4f\n', R_star_star);
% fprintf('The minimal turning radius has to be R < R**/3 = %.4f\n', R_star_star/(3));
% if R<=R_star_star/3
%     fprintf('\t [V]  The choosen turning radius is OK:  R=v/w=%.2f/%.2f= %.4f < R**/3=%.4f \n',v,w,R,R_star_star/3)
% elseif R>R_star_star/3
%     fprintf('\t [X] The choosen turning radius is NOT OK:  R=v/w=%.2f/%.2f= %.4f !< R**/3=%.4f \n',v,w,R,R_star_star/3)
% end
% 
% fprintf('All the points ||r-r0|| >= R**-2*R = %.4f will be growth controllable. \n', R_star_star-2*R);
% fprintf('The estimate starting radius has to be R_{est}>R*+4R = %.4f. \n', R_star + 4*R );


%% Calculate v*
fprintf('\n\n MATVEEV CHECKS\n')
fprintf('R = %.3f \n',R)
if  R_star >= 3*R
    fprintf('[V] R*= %.3f >= 3*R = %.3f \n',R_star, 3*R)
elseif R_star < 3*R
    fprintf('[X] R*= %.3f !>= 3*R = %.3f \n',R_star, 3*R)
end

% if R_est >= R_star+4*R
%     fprintf('[V] R_est= %.3f >= R*+4R = %.3f \n',R_est, R_star+4*R)
% elseif R_est < R_star + 4*R
%     fprintf('[X] R_est= %.3f !>= R*+4R = %.3f \n',R_est, R_star+4*R)
% end


R_minus = R_star - 2*R;
%R_plus = R_est + 2*R;

eq = @(sigma) ((R_minus*exp(-((-R_minus^2)/(2*sigma^2)))/ sqrt(1+ R*R_minus/(R_minus-R) * (1/R_minus - R_minus / sigma^2)) ));
sigma_opt = fminbnd(eq, sqrt(sigma2_minus), sqrt(sigma2_plus));
v_star_max = (q_minus*v/sigma_opt^2)*eq(sigma_opt);

% Display the result
%disp(['The value of sigma that minimizes the equation is: ', num2str(sigma_opt)]);
%disp(['The v* value has to be less than ',num2str(v_star_max) ] )
%fprintf('v*max(%.1f)=%.6f; v*min(%.1f)=%.6f \n',sqrt(sigma2_minus), (q_minus*v/sigma2_minus^2)*eq(sqrt(sigma2_minus)),sqrt(sigma2_plus), (q_minus*v/sigma2_plus^2)*eq(sqrt(sigma2_plus)))
if params.V_STAR<= v_star_max
    fprintf('[V] v* choosen is OK v*=%.4f <= v*_{max} = %.4f. \n',params.V_STAR, v_star_max );
elseif params.V_STAR > v_star_max
    fprintf('[X] v* choosen is NOT OK v*=%.4f !<= v*_{max} = %.4f. \n',params.V_STAR, v_star_max );
end


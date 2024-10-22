%% Plot Simulation
% This script plots the simulation results of the distributed gradient seeking unicycle robot 
kfig=0;

%% Animate the simulation
GIF=0;
if ANIMATE
    kfig = kfig + 1;
    figure(kfig);
    %set(gcf, 'WindowState', 'maximized');
    set(gcf, 'Position', [100 100 1200 720]);
    hold on;
    axis([-params.ENV_SIZE, params.ENV_SIZE, -params.ENV_SIZE, params.ENV_SIZE]);
    xlabel('X');
    ylabel('Y');
    title('Unicycle Robot Gradient Descent to Find Gaussian Peak');
    contour(X, Y, Z, 30);  % Create the contour plot
    colorbar;
    grid on;
    
    % Preallocate plot handles for the trajectories
    h_real_traj = plot(NaN, NaN, 'b-', 'LineWidth', 2, 'DisplayName', 'Robot Real Traj.');
    h_est_traj = plot(NaN, NaN, 'r--', 'LineWidth', 2, 'DisplayName', 'Robot Est Traj.');
    
    % Specify the filename for the GIF
    filename = 'unicycle_robot_gradient_descent.gif';
    
    % Initialize the GIF creation process
    first_frame = true;
    
    for i = 1:length(time(1:iter_break))
        % Update the trajectories
        set(h_real_traj, 'XData', q_ROBOT_real_vals(1, 1:i), 'YData', q_ROBOT_real_vals(2, 1:i));
        set(h_est_traj, 'XData', q_ROBOT_est_vals(1, 1:i), 'YData', q_ROBOT_est_vals(2, 1:i));
        
        % Plot the agent and keep the handle
        agent = robot.PlotAgent(q_ROBOT_real_vals(:,i));
        
        % Redraw the plot only every few iterations or after significant changes
        if mod(i, 3) == 0 || i == length(time(1:iter_break))
            drawnow;
            if GIF
                % Capture the plot as a frame
                frame = getframe(kfig);
                im = frame2im(frame);
                [imind, cm] = rgb2ind(im, 256);
                
                % Write to the GIF file
                if first_frame
                    imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
                    first_frame = false;  % Update to indicate that the first frame is done
                else
                    imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
                end
            end
        end
        
        % Only delete the agent handle if necessary
        if ishandle(agent)
            delete(agent);
        end
    end
end

%% Plot x,y,theta coordinates of the robot, the real and the estimated one with the errors
% Support variables
% Extract variances
sigma2_xx = zeros(1, iter_break);
sigma2_yy = zeros(1, iter_break);
sigma2_tt = zeros(1, iter_break);
for i = 1:iter_break
    sigma2_xx(i) = P_ROBOT_vals{i}(1,1);
    sigma2_yy(i) = P_ROBOT_vals{i}(2,2);
    sigma2_tt(i) = P_ROBOT_vals{i}(3,3);
end
% Calculate the 95% confidence interval for x
x_ci_upper = q_ROBOT_est_vals(1,1:iter_break) + 1.96 * sqrt(sigma2_xx(1:iter_break));
x_ci_lower = q_ROBOT_est_vals(1,1:iter_break) - 1.96 * sqrt(sigma2_xx(1:iter_break));
% Calculate the 95% confidence interval for y
y_ci_upper = q_ROBOT_est_vals(2,1:iter_break) + 1.96 * sqrt(sigma2_yy(1:iter_break));
y_ci_lower = q_ROBOT_est_vals(2,1:iter_break) - 1.96 * sqrt(sigma2_yy(1:iter_break));
% Calculate the 95% confidence interval for theta
tt_ci_upper = q_ROBOT_est_vals(3,1:iter_break) + 1.96 * sqrt(sigma2_tt(1:iter_break));
tt_ci_lower = q_ROBOT_est_vals(3,1:iter_break) - 1.96 * sqrt(sigma2_tt(1:iter_break));

% Main plot
kfig=kfig+1;
figure(kfig);
ax1 = subplot(3, 2, 1);
hold on;
plot(time(1:iter_break), q_ROBOT_real_vals(1,1:iter_break), 'LineWidth', 1.4, 'DisplayName', '$x_{real}$');
plot(time(1:iter_break), q_ROBOT_est_vals(1,1:iter_break), 'r--', 'LineWidth', 1,  'DisplayName', '$x_{est}$');
fill([time(1:iter_break) fliplr(time(1:iter_break))], [x_ci_upper fliplr(x_ci_lower)], 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName','$95\%$ C.I.');
legend('Interpreter', 'latex', 'Location', 'northeast');
grid on;
xlabel('Time [s]');
ylabel('$x$ Coordinate [m]');
hold off;

ax2 = subplot(3,2,2);
x_error = sqrt(sum((q_ROBOT_est_vals(1,1:iter_break) - q_ROBOT_real_vals(1,1:iter_break)).^2, 1));
plot(time(1:iter_break), x_error, 'LineWidth', 1.2, 'DisplayName', '$x$ Error');
mean_error_x = mean(x_error);
yline(mean_error_x,'--', sprintf('%.3f',mean_error_x), 'Color', "red",'LineWidth',2,'LabelHorizontalAlignment','right', 'LabelVerticalAlignment','top', 'DisplayName','Mean', 'FontSize',12);
xlabel('Time [s]');
ylabel('$x$ - Error [m]');
legend('Interpreter', 'latex', 'Location', 'northeast');
grid on;

ax3 = subplot(3, 2, 3);
hold on;
plot(time(1:iter_break), q_ROBOT_real_vals(2,1:iter_break), 'LineWidth', 1.4, 'DisplayName', '$y_{real}$');
plot(time(1:iter_break), q_ROBOT_est_vals(2,1:iter_break), 'r--', 'LineWidth', 1,  'DisplayName', '$y_{est}$');
fill([time(1:iter_break) fliplr(time(1:iter_break))], [y_ci_upper fliplr(y_ci_lower)], 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName','$95\%$ C.I.');
legend('Interpreter', 'latex', 'Location', 'northeast');
grid on;
xlabel('Time [s]');
ylabel('$y$ Coordinate [m]');
hold off;

ax4 = subplot(3,2,4);
y_error = sqrt(sum((q_ROBOT_est_vals(2,1:iter_break) - q_ROBOT_real_vals(2,1:iter_break)).^2, 1));
plot(time(1:iter_break), y_error, 'LineWidth', 1.2, 'DisplayName', '$y$ Error');
mean_error_y = mean(y_error);
yline(mean_error_y,'--', sprintf('%.3f',mean_error_y), 'Color', "red",'LineWidth',2,'LabelHorizontalAlignment','right', 'LabelVerticalAlignment','top','DisplayName','Mean', 'FontSize',12);
xlabel('Time [s]');
ylabel('$y$ - Error [m]');
legend('Interpreter', 'latex', 'Location', 'northeast');
grid on;

ax5 = subplot(3, 2, 5);
hold on;
plot(time(1:iter_break), q_ROBOT_real_vals(3,1:iter_break), 'LineWidth', 1.4, 'DisplayName', '$\theta _{real}$');
plot(time(1:iter_break), q_ROBOT_est_vals(3,1:iter_break), 'r--', 'LineWidth', 1.0,  'DisplayName', '$\theta _{est}$');
fill([time(1:iter_break) fliplr(time(1:iter_break))], [tt_ci_upper fliplr(tt_ci_lower)], "b", 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName','$95\%$ C.I.');
legend('Interpreter', 'latex', 'Location', 'northeast');
grid on;
xlabel('Time [s]');
ylabel('$\theta$ Coordinate [rad]');
hold off;

ax6 = subplot(3,2,6);
theta_error = sqrt(sum((q_ROBOT_est_vals(3,1:iter_break) - q_ROBOT_real_vals(3,1:iter_break)).^2, 1));
plot(time(1:iter_break), theta_error, 'LineWidth', 1.2, 'DisplayName', '$\theta$ Error');
mean_error_theta = mean(theta_error);
yline(mean_error_theta,'--', sprintf('%.3f',mean_error_theta), 'Color', "red",'LineWidth',2,'LabelHorizontalAlignment','right', 'LabelVerticalAlignment','top','DisplayName','Mean', 'FontSize',12);
xlabel('Time [s]');
ylabel('$\theta$ - Error [rad]');
legend('Interpreter', 'latex', 'Location', 'northeast');
grid on;

% set(gcf, 'Position', [100 100 500 350]);
set(gcf, 'WindowState', 'maximized');
set(gcf, 'PaperPositionMode', 'auto');
linkaxes([ax1,ax2,ax3,ax4,ax5,ax6],'x');

%% Plot trajectory on the scalar field
% kfig=kfig+1;
% figure(kfig);
% hold on;
% contour(X, Y, Z, 30, 'DisplayName','Scalar Field');
% colorbar;
% plot(q_ROBOT_est_vals(1,1:iter_break), q_ROBOT_est_vals(2,1:iter_break), 'r--', 'LineWidth', 1.5, 'DisplayName','Robot Est Trajectory');
% plot(q_ROBOT_real_vals(1,1:iter_break), q_ROBOT_real_vals(2,1:iter_break), 'b-', 'LineWidth', 1.5, 'DisplayName','Robot Real Trajectory');
% xlabel('X Coordinate');
% ylabel('Y Coordinate');
% title('Unicycle Robot Trajectory');
% legend('Location','best');
% grid on;

%% Plot the error ellipses
if PLOT_ERROR_ELLIPSES
    kfig=kfig+1;
    figure(kfig);
    hold on;
    contour(X, Y, Z, 30, 'DisplayName','Scalar Field');
    colorbar;
    
    for i = [1, 5:5:iter_break]  % Combining first ellipse and every 5th ellipse
        plot_ellipse(q_ROBOT_est_vals(1,i), q_ROBOT_est_vals(2,i), P_ROBOT_vals{i}(1:2,1:2), 0.95, [0.9290 0.6940 0.1250]);
    end
    
    plot(q_ROBOT_est_vals(1,1:iter_break), q_ROBOT_est_vals(2,1:iter_break), 'r--', 'LineWidth', 1.5, 'DisplayName','Robot Est Trajectory');
    plot(q_ROBOT_real_vals(1,1:iter_break), q_ROBOT_real_vals(2,1:iter_break), 'b-', 'LineWidth', 1.5, 'DisplayName','Robot Real Trajectory');
    hold off;
    xlabel('X Coordinate [m]');
    ylabel('Y Coordinate [m]');
    title('Error Ellipses of the Robot Estimated Position');
    grid on;
end

%% Plot covariance matrix - diagonal elements
kfig=kfig+1;
figure(kfig);
hold on;
plot(time(1:iter_break), sigma2_xx, 'Color', 'r', 'DisplayName', '$\sigma_{xx}^2$');
plot(time(1:iter_break), sigma2_yy, 'Color', 'b', 'DisplayName', '$\sigma_{yy}^2$');
ylabel("$\sigma_{xx}^2$,$\sigma_{yy}^2$ [$m^2$]",'Interpreter','latex')
yyaxis right
plot(time(1:iter_break), sigma2_tt, 'Color', "#EDB120", 'DisplayName', '$\sigma_{\theta\theta}^2$');
ylabel('$\sigma_{\theta\theta}^2$ [$rad^2$]','Interpreter','latex')
xlabel('Time [s]');
title('Robot P Covariance Matrix Diagonal Elements');
legend('Interpreter', 'latex', 'Location', 'best');
grid on;
hold off;

%% Plot in a 3D-view the field distribution
% kfig=kfig+1;
% figure(kfig);
% surf(X,Y,Z);
% hold on;
% plot3(Dx_peaks, Dy_peaks, Dz_peaks, 'r*', 'MarkerSize', 10);
% hold off;

%% Plot in a 3D-view the field curvature
% kfig=kfig+1;
% figure(kfig);
% hold on;
% surf(X,Y,k);
% title('Field curvature');
% hold off;

%% Plot field curvature
% kfig=kfig+1;
% figure(kfig);
% hold on
% plot(time(1:iter_break), k_ROBOT_vals(1,1:iter_break), 'LineWidth', 1.4, 'DisplayName', '$\kappa_{field}$');
% plot(time(1:iter_break), w_ROBOT_vals(1,1:iter_break)./v_ROBOT_vals(1,1:iter_break), 'LineWidth', 1.4,'Color','red', 'DisplayName', '$\kappa_{robot}$');
% legend('Interpreter', 'latex', 'Location', 'NorthWest');
% grid on;
% xlabel('Time [s]');
% ylabel('$\kappa$ Curvature [$m^{-1}$]');
% title('Curvature $\kappa$ [$m^{-1}$]');
% legend('Interpreter', 'latex', 'Location', 'NorthWest');
% grid on;
% hold off

%% Plot field value
kfig=kfig+1;
figure(kfig);
hold on
plot(time(1:iter_break), D_ROBOT_vals(1,1:iter_break), 'LineWidth', 1.4);%, 'DisplayName', '$D{field}$');
grid on;
xlabel('Time [s]');
ylabel('D [-]');
title('Field Values');
%legend('Interpreter', 'latex', 'Location', 'NorthWest');
grid on;
hold off

%% Plot v_star values if control algorithm selected is Matveev-v3
if params.control_alg=='Matveev-v3'
    % fit the values
    p = polyfit(time(1:iter_break),v_star_ROBOT_vals(1:iter_break),2);
    y_fit = polyval(p,time(1:iter_break));

    kfig=kfig+1;
    figure(kfig);
    plot(time(1:iter_break), v_star_ROBOT_vals(1:iter_break),'DisplayName','Real values')
    hold on;
    plot(time(1:iter_break), y_fit, 'r-', 'DisplayName','Fitted values'); % Fitted curve in red
    grid on;
    xlabel('Time [s]');
    ylabel('$v^*$ [-]');
    title('$v^*$ Values');
    legend('Interpreter', 'latex', 'Location', 'Best');
    grid on;
    hold off

end

%% =============FUNCTIONS===============
%% Useful functions for plotting purposes
function plot_ellipse(x, y, P, conf, color)
    %PLOT_ELLIPSE Plot a covariance ellipse
    % Inputs:
    %   x, y: Center of the ellipse
    %   P: Covariance matrix
    %   conf: Confidence interval (0.95 for 95% confidence)
    %   color: Color of the ellipse
    
    [eigVec, eigVal] = eig(P);

    if eigVal(1,1) > eigVal(2,2)  % Get the highest eigenvalue index
        a = sqrt(eigVal(1,1));  % Half-major axis length
        b = sqrt(eigVal(2,2));  % Half-minor axis length
        theta = atan2(eigVec(2,1), eigVec(1,1)); % Ellipse angle (radians)
    else
        a = sqrt(eigVal(2,2));  % Half-major axis length
        b = sqrt(eigVal(1,1));  % Half-minor axis length
        theta = atan2(eigVec(2,2), eigVec(1,2)); % Ellipse angle (radians)
    end

    k = sqrt(-2 * log(1 - conf)); % 95% confidence

    a95 = k * a; % 95% confidence
    b95 = k * b; % 95% confidence

    % Plot the ellipse
    t = linspace(0, 2*pi, 100);
    ellipse = [cos(t) * a; sin(t) * b];
    ellipse95 = [cos(t) * a95; sin(t) * b95];
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    ellipse = R * ellipse;
    ellipse95 = R * ellipse95;

    fill(x + ellipse(1,:), y + ellipse(2,:), color, 'FaceAlpha', 0.2);
    plot(x + ellipse95(1,:), y + ellipse95(2,:),'--', 'Color', color, 'LineWidth', 1.5);

    % Create dummy plots for legend entries
    h1 = fill(NaN, NaN, color, 'FaceAlpha', 0.2, 'DisplayName', 'Covariance ellipse'); 
    h2 = plot(NaN, NaN, '--', 'Color', color, 'DisplayName', '95\% Confidence ellipse '); 

    % Create custom legend
    legend([h1, h2], 'Interpreter', 'latex', 'Location', 'best');
end


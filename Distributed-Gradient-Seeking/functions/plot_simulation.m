%%  Plot Simulation
% This script plots the simulation results of the distributed gradient seeking unicycle robot 

%% Animate the simulation
if ANIMATE
    figure;
    hold on;
    axis([-params.ENV_SIZE, params.ENV_SIZE, -params.ENV_SIZE,  params.ENV_SIZE]);
    xlabel('X');
    ylabel('Y');
    title('Unicycle Robot Gradient Descent to Find Gaussian Peak');
    grid on;
    %contourf(X, Y, D(X,Y), 50, 'LineStyle', 'none');
    contour(X, Y, Z, 30);
    colorbar;
    for i = 1:length(time(1:iter_break))
        agent=robot.PlotAgent(q_real_vals(:,i));
        plot(q_real_vals(1, 1:i), q_real_vals(2, 1:i), 'b-', 'LineWidth', 2);
        pause(dt);
        if ishandle(agent)
            delete(agent);
        end
    end
    
end

%% Plot results
if PLOTALL
    figure
        ax1=subplot(3, 2, 1);
        hold on
        plot(time(1:iter_break), q_real_vals(1,1:iter_break), 'LineWidth', 1.4, 'DisplayName', '$x_{real}$')
        plot(time(1:iter_break), q_est_vals(1,1:iter_break), 'r--', 'LineWidth', 1,  'DisplayName', '$x_{est}$')
        legend('Interpreter', 'latex', 'Location', 'NorthWest')
        grid on
        xlabel('Time [s]')
        ylabel('$x$ Coordinate [m]')
        hold off
    
        ax2= subplot(3,2,2);
        plot(time(1:iter_break), sqrt(sum((q_est_vals(1,1:iter_break) - q_real_vals(1,1:iter_break)).^2, 1)), 'LineWidth', 1.2, 'DisplayName', '$x$ Error')
        mean_error_x = mean(sqrt(sum((q_est_vals(1,1:iter_break) - q_real_vals(1,1:iter_break)).^2, 1)));
        yline(mean_error_x,'--', sprintf('%.3f',mean_error_x), 'Color', "red",'LineWidth',2,'LabelHorizontalAlignment','right', 'LabelVerticalAlignment','top', 'DisplayName','Mean', 'FontSize',12);
        xlabel('Time [s]')
        ylabel('$x$ - Error [m]')
        legend('Interpreter', 'latex', 'Location', 'NorthWest')
        grid on
    
        ax3=subplot(3, 2, 3);
        hold on
        plot(time(1:iter_break), q_real_vals(2,1:iter_break), 'LineWidth', 1.4, 'DisplayName', '$y_{real}$')
        plot(time(1:iter_break), q_est_vals(2,1:iter_break), 'r--', 'LineWidth', 1,  'DisplayName', '$y_{est}$')
        legend('Interpreter', 'latex', 'Location', 'NorthWest')
        grid on
        xlabel('Time [s]')
        ylabel('$y$ Coordinate [m]')
        hold off
    
        ax4= subplot(3,2,4);
        plot(time(1:iter_break), sqrt(sum((q_est_vals(2,1:iter_break) - q_real_vals(2,1:iter_break)).^2, 1)), 'LineWidth', 1.2, 'DisplayName', '$y$ Error')
        mean_error_y = mean(sqrt(sum((q_est_vals(2,1:iter_break) - q_real_vals(2,1:iter_break)).^2, 1)));
        yline(mean_error_y,'--', sprintf('%.3f',mean_error_y), 'Color', "red",'LineWidth',2,'LabelHorizontalAlignment','right', 'LabelVerticalAlignment','top','DisplayName','Mean', 'FontSize',12);
        xlabel('Time [s]')
        ylabel('$y$ - Error [m]')
        legend('Interpreter', 'latex', 'Location', 'NorthWest')
        grid on
    
        ax5=subplot(3, 2, 5);
        hold on
        plot(time(1:iter_break), q_real_vals(3,1:iter_break), 'LineWidth', 1.4, 'DisplayName', '$\theta _{real}$')
        plot(time(1:iter_break), q_est_vals(3,1:iter_break), 'r--', 'LineWidth', 1.0,  'DisplayName', '$\theta _{est}$')
        legend('Interpreter', 'latex', 'Location', 'NorthWest')
        grid on
        xlabel('Time [s]')
        ylabel('$\theta$ Coordinate [rad]')
        hold off
    
        ax6= subplot(3,2,6);
        plot(time(1:iter_break), sqrt(sum((q_est_vals(3,1:iter_break) - q_real_vals(3,1:iter_break)).^2, 1)), 'LineWidth', 1.2, 'DisplayName', '$\theta$ Error')
        mean_error_theta = mean(sqrt(sum((q_est_vals(3,1:iter_break) - q_real_vals(3,1:iter_break)).^2, 1)));
        yline(mean_error_theta,'--', sprintf('%.3f',mean_error_theta), 'Color', "red",'LineWidth',2,'LabelHorizontalAlignment','right', 'LabelVerticalAlignment','top','DisplayName','Mean', 'FontSize',12);
        xlabel('Time [s]')
        ylabel('$\theta$ - Error [rad]')
        legend('Interpreter', 'latex', 'Location', 'NorthWest')
        grid on
    
        set(gcf, 'Position', [100 100 500 350])
        set(gcf, 'PaperPositionMode', 'auto')
        linkaxes([ax1,ax2,ax3,ax4,ax5,ax6],'x')
    
    
    % Plot trajectory on the scalar field
    figure
        hold on
        contour(X, Y, Z, 30, 'DisplayName','Scalar Field')
        colorbar
        %quiver(X,Y,DX,DY,0)
        plot(q_est_vals(1,1:iter_break), q_est_vals(2,1:iter_break), 'r--', 'LineWidth', 1.5, 'DisplayName','Robot Est Trajectory')
        plot(q_real_vals(1,1:iter_break), q_real_vals(2,1:iter_break), 'b-', 'LineWidth', 1.5, 'DisplayName','Robot Real Trajectory')
        xlabel('X Coordinate')
        ylabel('Y Coordinate')
        title('Unicycle Robot Trajectory with ESC')
        legend('Location','best')
        grid on
    
    % Plot the error ellipses
    figure
        hold on
        contour(X, Y, Z, 30, 'DisplayName','Scalar Field')
        colorbar
        for i=1:iter_break
            % Plot the error ellipses, plot the first and every 5th ellipse
            if mod(i,5)==0
                plot_ellipse(q_est_vals(1,i), q_est_vals(2,i), P_vals{i}(1:2,1:2), 0.95, [0.9290 0.6940 0.1250]);
            elseif i==1
                plot_ellipse(q_est_vals(1,i), q_est_vals(2,i), P_vals{i}(1:2,1:2), 0.95, [0.9290 0.6940 0.1250]);
            end
        end
        plot(q_est_vals(1,1:iter_break), q_est_vals(2,1:iter_break), 'r--', 'LineWidth', 1.5, 'DisplayName','Robot Est Trajectory')
        plot(q_real_vals(1,1:iter_break), q_real_vals(2,1:iter_break), 'b-', 'LineWidth', 1.5, 'DisplayName','Robot Real Trajectory')
        hold off
        xlabel('X Coordinate [m]')
        ylabel('Y Coordinate [m]')
        title('Error Ellipses of the Robot Estimated Position')
        grid on
        
    % Plot covariance matrix - diagonal elements
    figure 
        hold on
        for i = 1:iter_break
            sigma_xx(i) = P_vals{i}(1,1);
            sigma_yy(i) = P_vals{i}(2,2);
            sigma_tt(i) = P_vals{i}(3,3);
        end
    
        plot(time(1:iter_break), sigma_xx, 'Color', 'r', 'DisplayName', '$\sigma_{xx}^2$');
        plot(time(1:iter_break), sigma_yy, 'Color', 'b', 'DisplayName', '$\sigma_{yy}^2$');
        plot(time(1:iter_break), sigma_tt, 'Color', 'g', 'DisplayName', '$\sigma_{\theta\theta}^2$');
        
        xlabel('Time [s]')
        ylabel('Covariance Matrix Diagonal Elements')
        legend('Interpreter', 'latex', 'Location', 'best')
        grid on
        hold off
        
else
    % Plot in a 3D-view the field distribution
    figure
    surf(X,Y,Z)
end

%% Useful functions for plotting purposes
function plot_ellipse(x, y, P, conf, color)
    %PLOT_ELLIPSE Plot a covariance ellipse
    % Inputs:
    %   x, y: Center of the ellipse
    %   P: Covariance matrix
    %   conf: Confidence interval (0.95 for 95% confidence)
    %   color: Color of the ellipse
    
    [eigVec, eigVal] = eig(P);

    if eigVal(1,1)>eigVal(2,2)  % get the highest eigenvalue index
        a = sqrt(eigVal(1,1));  % half-major axis length
        b = sqrt(eigVal(2,2));  % half-minor axis length
        theta = atan2(eigVec(2,1), eigVec(1,1)); % ellipse angle (radians)
    else
        a = sqrt(eigVal(2,2));  % half-major axis length
        b = sqrt(eigVal(1,1));  % half-minor axis length
        theta = atan2(eigVec(2,2), eigVec(1,2)); % ellipse angle (radians)
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
    xdummy=NaN;
    ydummy=NaN;
    h1 = fill(xdummy, ydummy, color, 'FaceAlpha', 0.2, 'DisplayName', 'Covariance ellipse'); % Example for contour
    h2 = plot(xdummy, ydummy, '--', 'Color', color, 'DisplayName', '95\% Confidence ellipse '); % Example for error ellipse
    % Add more dummy plots as needed for other legend entries

    % Create custom legend
    legend([h1, h2], 'Interpreter', 'latex','Location', 'best');

end



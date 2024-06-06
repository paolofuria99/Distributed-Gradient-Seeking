%%  Plot Simulation
% This script plots the simulation results of the unicycle robot 

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


% Additional visualization: plot trajectory on the scalar field
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



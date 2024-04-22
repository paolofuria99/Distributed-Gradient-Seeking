function PlotUnicycle(x,y,theta, time, params, x_d, y_d,theta_d,x_d2,y_d2,theta_d2)

% Vehicle description
Vehicle.r = params.AGENT_RADIUS;
Vehicle.dir = params.AGENT_RADIUS;

% MaxXY = params.WIDTH;
% MinXY = params.HEIGHT;
MaxXY = max(max(x), max(y)) + 4*Vehicle.r;
MinXY = min(min(x), min(y)) - 4*Vehicle.r;
MaxTOT= max(MinXY,MaxXY);

% Initial position
figure(1), clf, hold on;
hr = rectangle('Position', [x(1) - Vehicle.r, y(1) - Vehicle.r, 2*Vehicle.r, 2*Vehicle.r], 'Curvature', [1,1], ...
    'EdgeColor', [1 0 0]);
hl = plot([x(1), x(1)+Vehicle.dir*cos(theta(1))], [y(1), y(1)+Vehicle.dir*sin(theta(1))], 'r');
if nargin > 5
    hr_d = rectangle('Position', [x_d(1) - Vehicle.r, y_d(1) - Vehicle.r, 2*Vehicle.r, 2*Vehicle.r], 'Curvature', [1,1], ...
    'EdgeColor', [0 1 0]);
    hl_d = plot([x_d(1), x_d(1)+Vehicle.dir*cos(theta_d(1))], [y_d(1), y_d(1)+Vehicle.dir*sin(theta_d(1))], 'g');
    if nargin > 8
        hr_d2 = rectangle('Position', [x_d2(1) - Vehicle.r, y_d2(1) - Vehicle.r, 2*Vehicle.r, 2*Vehicle.r], 'Curvature', [1,1], ...
            'EdgeColor', [0 0 1]);
        hl_d2 = plot([x_d2(1), x_d2(1)+Vehicle.dir*cos(theta_d2(1))], [y_d2(1), y_d2(1)+Vehicle.dir*sin(theta_d2(1))], 'b');
    end
end
axis([-MaxTOT, MaxTOT, -MaxTOT, MaxTOT]);

ht = [];
ht_d = [];
ht_d2 = [];

for i=2:length(x)

    %% Actual trajectory

    delete(hr);
    delete(hl);
    if not(isempty(ht))
        delete(ht);
    end

    % Vehicle plot
    hr = rectangle('Position', [x(i) - Vehicle.r, y(i) - Vehicle.r, 2*Vehicle.r, 2*Vehicle.r], 'Curvature', [1,1], ...
        'EdgeColor', [1 0 0]);
    hl = plot([x(i), x(i)+Vehicle.dir*cos(theta(i))], [y(i), y(i)+Vehicle.dir*sin(theta(i))], 'r');
    axis([MinXY, MaxXY, MinXY, MaxXY]);

    % Vehicle trajectory
    ht = plot(x(1:i), y(1:i), 'r');


    %% Discrete approximations

    if nargin > 5
        delete(hr_d);
        delete(hl_d);
        if not(isempty(ht_d))
            delete(ht_d);
        end

        % Vehicle plot
        hr_d = rectangle('Position', [x_d(i) - Vehicle.r, y_d(i) - Vehicle.r, 2*Vehicle.r, 2*Vehicle.r], 'Curvature', [1,1], ...
            'EdgeColor', [0 1 0]);
        hl_d = plot([x_d(i), x_d(i)+Vehicle.dir*cos(theta_d(i))], [y_d(i), y_d(i)+Vehicle.dir*sin(theta_d(i))], 'g');

        % Vehicle trajectory
        ht_d = plot(x_d(1:i), y_d(1:i), 'g--');

        if nargin > 8
            delete(hr_d2);
            delete(hl_d2);
            if not(isempty(ht_d2))
                delete(ht_d2);
            end


            hr_d2 = rectangle('Position', [x_d2(i) - Vehicle.r, y_d2(i) - Vehicle.r, 2*Vehicle.r, 2*Vehicle.r], 'Curvature', [1,1], ...
                'EdgeColor', [0 0 1]);
            hl_d2 = plot([x_d2(i), x_d2(i)+Vehicle.dir*cos(theta_d2(i))], [y_d2(i), y_d2(i)+Vehicle.dir*sin(theta_d2(i))], 'b');
            axis([MinXY, MaxXY, MinXY, MaxXY]);

            % Vehicle trajectory
            ht_d2 = plot(x_d2(1:i), y_d2(1:i), 'b--');
        end
    end

    title(['Time - ', num2str(1e3*time(i)), ' [ms]']);

    drawnow;

    %pause(1e-2*(time(i) - time(i-1)));
end
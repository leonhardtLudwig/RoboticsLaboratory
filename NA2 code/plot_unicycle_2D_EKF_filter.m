function plot_unicycle_2D_EKF_filter(q, q_filt, P_filt, downsampling)
% q [3,N] x, y, theta trajectory
% q_filt [3,N] x, y, theta EKF filtered trajectory
% downsampling: useful to reduce the unicycle plot

% plot the trajectory
plot(q(1,:),q(2,:), 'k--');
hold on
plot(q_filt(1,:),q_filt(2,:), 'g--');
xlabel('x');
ylabel('y');

% get x and y ranges
max_pos = max(q(1:2,:),[], 2) + 0.5;
min_pos = min(q(1:2,:),[], 2) - 0.5;
diff_pos = max_pos - min_pos;
center = (max_pos + min_pos)/2;
delta_visualization = max(diff_pos);
xlim([center(1)-delta_visualization/2, center(1)+delta_visualization/2 ])
ylim([center(2)-delta_visualization/2, center(2)+delta_visualization/2])

% set axis
xline(0, 'k:')
yline(0, 'k:')
grid('on')

% get the rotation matrix
f_R = @(theta) [[cos(theta);sin(theta)],[-sin(theta);cos(theta)]];

% set unrotated polygon points
polygon_points =  [[0 0 0.05];[-0.01 0.01 0]];
if downsampling<0
    % initial
    rotated_polygon_points = f_R(q(3,1))*polygon_points;
    pgon = polyshape(q(1,1)+rotated_polygon_points(1,:),q(2,1)+rotated_polygon_points(2,:));
    plot(pgon, 'FaceColor','black', 'FaceAlpha',0.1);
    rotated_polygon_points_filt = f_R(q_filt(3,1))*polygon_points;
    pgon_filt = polyshape(q_filt(1,1)+rotated_polygon_points_filt(1,:),q_filt(2,1)+rotated_polygon_points_filt(2,:));
    plot(pgon_filt, 'FaceColor','green', 'FaceAlpha',0.1);
    % final
    rotated_polygon_points = f_R(q(3,end))*polygon_points;
    pgon = polyshape(q(1,end)+rotated_polygon_points(1,:),q(2,end)+rotated_polygon_points(2,:));
    plot(pgon, 'FaceColor','black', 'FaceAlpha',0.1);
    rotated_polygon_points_filt = f_R(q_filt(3,end))*polygon_points;
    pgon_filt = polyshape(q_filt(1,end)+rotated_polygon_points_filt(1,:),q_filt(2,end)+rotated_polygon_points_filt(2,:));
    plot(pgon_filt, 'FaceColor','green', 'FaceAlpha',0.1);
else
    for i = 1:downsampling:size(q,2)
        % plot unicycle
        rotated_polygon_points = f_R(q(3,i))*polygon_points;
        pgon = polyshape(q(1,i)+rotated_polygon_points(1,:),q(2,i)+rotated_polygon_points(2,:));
        plot(pgon, 'FaceColor','black');
        rotated_polygon_points_filt = f_R(q_filt(3,i))*polygon_points;
        pgon_filt = polyshape(q_filt(1,i)+rotated_polygon_points_filt(1,:),q_filt(2,i)+rotated_polygon_points_filt(2,:));
        plot(pgon_filt, 'FaceColor','green', 'FaceAlpha',0.2);
        % plot variance
        ellipse_points = get_elipse(P_filt(1:2,1:2,i), 0.2);
        ellipse = polyshape(q_filt(1,i)+ellipse_points(1,:),q_filt(2,i)+ellipse_points(2,:));
        plot(ellipse, 'FaceColor','green', 'FaceAlpha',0.2,'LineStyle','none');
    end
end
axis square
end
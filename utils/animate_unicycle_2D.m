function animate_unicycle_2D(q, T_sampling, figure_number)
% q [3,N] x, y, theta trajectory
% T_sampling sampling time, needed for video
% figure number number of the figure used to plot the animation

% get the x and y range to plot
max_pos = max(q(1:2,:),[], 2) + 0.5;
min_pos = min(q(1:2,:),[], 2) - 0.5;
diff_pos = max_pos - min_pos;
center = (max_pos + min_pos)/2;
delta_visualization = max(diff_pos);

% get the 2D rotation matrix
f_R = @(theta) [[cos(theta);sin(theta)],[-sin(theta);cos(theta)]];

% set un-rotated polygon points
polygon_points = [[0 0 0.25];[-0.05 0.05 0]];

% iterate over trajectory
num_steps = size(q,2);
for t = 1:num_steps
   
    figure(figure_number) 
    % plot ther trajectory
    plot(q(1,:),q(2,:), 'k--');
    hold on
    xlabel('x');
    ylabel('y');
    xline(0, 'k:')
    yline(0, 'k:')
    xlim([center(1)-delta_visualization/2, center(1)+delta_visualization/2 ])
    ylim([center(2)-delta_visualization/2, center(2)+delta_visualization/2])
    grid('on')

    % plot the polygon
    rotated_polygon_points = f_R(q(3,t))*polygon_points;
    pgon = polyshape(q(1,t)+rotated_polygon_points(1,:),q(2,t)+rotated_polygon_points(2,:));
    plot(pgon, 'FaceColor','green');
    axis square
    drawnow
    hold off
    F(t) = getframe(gcf);
end
writerObj = VideoWriter('myVideo.avi');
writerObj.FrameRate = round(1/T_sampling);
open(writerObj);
% write the frames to the video
for i=1:length(F)
    % convert the image to a frame
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);
end
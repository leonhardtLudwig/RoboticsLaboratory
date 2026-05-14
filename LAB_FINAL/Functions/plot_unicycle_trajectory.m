function plot_unicycle_trajectory(trajectory_ideal, trajectory_actual, label)
    % PLOT_UNICYCLE_TRAJECTORY Plots ideal vs actual trajectories with unicycle symbols.
    %
    % Input:
    %   trajectory_ideal  - 3xN matrix (x, y, theta) of the desired trajectory
    %   trajectory_actual - 3xM matrix (x, y, theta) of the executed trajectory

    % Check dimensions
    if size(trajectory_ideal, 1) ~= 3 || size(trajectory_actual, 1) ~= 3
        error('Error: Both input matrices must have 3 rows (x, y, theta).');
    end

    % Extract coordinates and orientations
    x_ideal = trajectory_ideal(1, :);
    y_ideal = trajectory_ideal(2, :);
    theta_ideal = trajectory_ideal(3, :);

    x_actual = trajectory_actual(1, :);
    y_actual = trajectory_actual(2, :);
    theta_actual = trajectory_actual(3, :);

    % Setup the figure
    figure('Name', 'Unicycle Trajectories Comparison', 'Color', 'w');
    hold on;
    
    % Plot the lines first (so symbols go on top)
    plot(x_ideal, y_ideal, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Ideal');
    plot(x_actual, y_actual, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual');
    
    % --- DYNAMIC SIZING FOR UNICYCLE SYMBOLS ---
    % Calculate the max range of the plot to scale the triangles appropriately
    max_x = max([x_ideal, x_actual]); min_x = min([x_ideal, x_actual]);
    max_y = max([y_ideal, y_actual]); min_y = min([y_ideal, y_actual]);
    span = max(max_x - min_x, max_y - min_y);
    
    if span == 0
        span = 1; % Safety fallback if data is perfectly stationary
    end
    
    % Triangle size is dynamically set to 4% of the total plot span
    base_L = span * 0.04; 
    
    % Base vertices in local frame (centered)
    base_vertices = [base_L, 0; -0.3*base_L, 0.4*base_L; -0.3*base_L, -0.4*base_L];

    % Configuration for unicycle symbols
    num_symbols_target = 12; % Target number of symbols per trajectory
    symbol_opacity = 0.5;    % Semi-transparent (FaceAlpha)
    symbol_border_width = 0.5; % Light border (LineWidth)

    % Function to plot symbols along a trajectory (name parameter removed)
    plot_symbols = @(x, y, theta, color) ...
        plot_trajectory_symbols(x, y, theta, base_vertices, num_symbols_target, ...
                               color, symbol_opacity, symbol_border_width);

    % Plot symbols for Ideal (Red)
    plot_symbols(x_ideal, y_ideal, theta_ideal, 'r');

    % Plot symbols for Actual (Blue)
    plot_symbols(x_actual, y_actual, theta_actual, 'b');

    % Graphic settings
    axis equal; 
    grid on;
    xlabel('X');
    ylabel('Y');
    title(['Ideal vs Actual Trajectory with Unicycle Symbols ', label]);
    legend('Location', 'best');
    
    hold off;
end

% --- Helper Function to Plot Symbols ---
function plot_trajectory_symbols(x, y, theta, base_v, num_target, color, alpha, border_w)
    N = length(x);
    if N < 2, return; end % Nothing to plot
    
    % Calculate sampling interval to get roughly 'num_target' symbols
    step = max(1, round(N / num_target));
    indices = 1:step:N;
    
    for i = indices
        % Get current state
        xi = x(i); yi = y(i); th_i = theta(i);
        
        % Create rotation matrix
        R = [cos(th_i), -sin(th_i); sin(th_i), cos(th_i)];
        
        % Rotate and translate base vertices
        rotated_v = (R * base_v')';
        translated_v = rotated_v + [xi, yi];
        
        % Plot the triangle patch
        % 'HandleVisibility', 'off' keeps all triangles out of the legend
        patch('Vertices', translated_v, 'Faces', [1 2 3], ...
              'FaceColor', color, 'FaceAlpha', alpha, ...
              'EdgeColor', color, 'LineWidth', border_w, ...
              'HandleVisibility', 'off'); 
    end
end
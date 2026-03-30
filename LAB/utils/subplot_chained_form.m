function subplot_chained_form(Q, U, Z, V, t)

    % Q [3 x 1 x t] -> q [3 x t]
    q = reshape(Q, [], size(Q, 3));
    
    % U [2 x 1] -> u [2 x t]
    u = U .* ones(2, length(t));
    
    % Z [3 x 1 x t] -> z [3 x t]
    z = reshape(Z, [], size(Z, 3));
    
    % V [2 x 1 x t] -> v [2 x t]
    v = reshape(V, [], size(V, 3));
    

    % UNICYCLE MODEL - STATE
    subplot(2,2,1);
    plot(t, q(1,:), 'b-', 'LineWidth', 1.2, 'DisplayName', 'x'); 
    hold on; grid on;
    plot(t, q(2,:), 'r-', 'LineWidth', 1.2, 'DisplayName', 'y');
    plot(t, q(3,:), 'k-', 'LineWidth', 1.2, 'DisplayName', '\theta');
    title('Unicycle State (q)');
    legend('Location', 'best');
    
    % UNICYCLE MODEL - INPUTS
    subplot(2,2,3);
    plot(t, u(1,:), 'm-', 'LineWidth', 1.2, 'DisplayName', 'v'); 
    hold on; grid on;
    plot(t, u(2,:), 'c-', 'LineWidth', 1.2, 'DisplayName', '\omega');
    title('Unicycle Inputs (u)');
    legend('Location', 'best');
    
    % CHIANED FORM - STATE
    subplot(2,2,2);
    plot(t, z(1,:), 'b-', 'LineWidth', 1.2, 'DisplayName', 'z_1'); 
    hold on; grid on;
    plot(t, z(2,:), 'r-', 'LineWidth', 1.2, 'DisplayName', 'z_2');
    plot(t, z(3,:), 'k-', 'LineWidth', 1.2, 'DisplayName', 'z_3');
    title('Chained Form State (z)');
    legend('Location', 'best');
    
    % CHAINED FORM - INPUTS
    subplot(2,2,4);
    plot(t, v(1,:), 'm-', 'LineWidth', 1.2, 'DisplayName', 'v_1'); 
    hold on; grid on;
    plot(t, v(2,:), 'c-', 'LineWidth', 1.2, 'DisplayName', 'v_2');
    title('Chained Form Inputs (v)');
    legend('Location', 'best');
end
function subplot_chained_formV2(Q, U, Z, V, t)

    % Q [3 x 1 x t] -> q [3 x t]
    q = reshape(Q, [], size(Q, 3));
    
    % U [2 x 1] -> u [2 x t]
    u = U .* ones(2, length(t));
    
    % Z [3 x 1 x t] -> z [3 x t]
    z = reshape(Z, [], size(Z, 3));
    
    % V [2 x 1 x t] -> v [2 x t]
    v = reshape(V, [], size(V, 3));


    % UNICYCLE MODEL
    subplot(5,2,1);
    plot(t, q(1,:), 'b-', 'LineWidth', 1.2);
    ylabel('x'); grid on; title('Unicycle (q, u)');
    
    subplot(5,2,3);
    plot(t, q(2,:), 'r-', 'LineWidth', 1.2);
    ylabel('y'); grid on;
    
    subplot(5,2,5);
    plot(t, q(3,:), 'k-', 'LineWidth', 1.2);
    ylabel('\theta'); grid on;
    
    subplot(5,2,7);
    plot(t, u(1,:), 'm-', 'LineWidth', 1.2);
    ylabel('v'); grid on;
    
    subplot(5,2,9);
    plot(t, u(2,:), 'c-', 'LineWidth', 1.2);
    ylabel('\omega'); xlabel('time'); grid on;
    
    % CHAINED FORM
    subplot(5,2,2);
    plot(t, z(1,:), 'b-', 'LineWidth', 1.2);
    ylabel('z_1'); grid on; title('Chained Form (z, v)');
    
    subplot(5,2,4);
    plot(t, z(2,:), 'r-', 'LineWidth', 1.2);
    ylabel('z_2'); grid on;
    
    subplot(5,2,6);
    plot(t, z(3,:), 'k-', 'LineWidth', 1.2);
    ylabel('z_3'); grid on;
    
    subplot(5,2,8);
    plot(t, v(1,:), 'm-', 'LineWidth', 1.2);
    ylabel('v_1'); grid on;
    
    subplot(5,2,10);
    plot(t, v(2,:), 'c-', 'LineWidth', 1.2);
    ylabel('v_2'); xlabel('time'); grid on;
end
function [q, u] = flat_to_kinematic_chained(y1, y2, y1_dot, y2_dot, y1_ddot, y2_ddot)
    
    % state
    z1 = y1;
    z3 = y2;
    
    v1 = y1_dot;
    
    z2 = y2_dot ./ y1_dot;
    
    % inputs
    v2 = (y2_ddot .* y1_dot - y2_dot .* y1_ddot) ./ (y1_dot.^2);
    
    % obtain unicycle state and inputs q, v, w

    theta = z1;
    
    x = z2 .* cos(theta) + z3 .* sin(theta);
    y = z2 .* sin(theta) - z3 .* cos(theta);
    
    q = [x; y; theta];
    
    v = v2 + z3 .* v1;
    omega = v1;
    
    u = [v; omega];
end
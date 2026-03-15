function [q, u] = flat_to_kinematic_cartesian(x, y, x_dot, y_dot, x_ddot, y_ddot)
    % FLAT_TO_KINEMATIC_CARTESIAN 

    theta = atan2(y_dot, x_dot);
    
    q = [x; y; theta];
    
    v = sqrt(x_dot^2 + y_dot^2);
    
    omega = (y_ddot * x_dot - x_ddot * y_dot) / (x_dot^2 + y_dot^2);
    
    u = [v; omega];
end
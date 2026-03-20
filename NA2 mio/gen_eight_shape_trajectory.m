function [x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot] = gen_eight_shape_trajectory(R, w, s)
    
    x_s = R*sin(2*w*s);
    y_s = R*sin(w*s);
    
    % computes derivatives analytically

    x_s_dot = 2*R*w*cos(2*w*s);
    y_s_dot = R*w*cos(w*s);
    
    x_s_ddot = -4*R*w^2*sin(2*w*s);
    y_s_ddot = -R*w^2*sin(w*s);

end
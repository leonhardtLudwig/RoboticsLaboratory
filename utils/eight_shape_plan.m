function [v, w, wL, wR, q_des] = eight_shape_plan(s, s_dot, R, freq, r, d)
    % Calcolo traiettoria
    x_s = R * sin(2*freq*s);
    y_s = R * sin(freq*s);
    
    x_s_dot = 2*R*freq * cos(2*freq*s);
    y_s_dot = R*freq * cos(freq*s);
    
    x_s_ddot = -4*R*freq^2 * sin(2*freq*s);
    y_s_ddot = -R*freq^2 * sin(freq*s);
    
    theta = atan2(y_s_dot, x_s_dot);
    q_des = [x_s; y_s; theta];
   
    v_tilde = sqrt(x_s_dot.^2 + y_s_dot.^2);
    omega_tilde = (x_s_dot .* y_s_ddot - y_s_dot .* x_s_ddot) ./ (v_tilde.^2);
    
    % Ingressi di controllo (v, w)
    v = v_tilde .* s_dot;
    w = omega_tilde .* s_dot; % rinominato da omega a w
    
    % Velocità ruote
    wL = v/r - (d*w)/(2*r);
    wR = v/r + (d*w)/(2*r);
end
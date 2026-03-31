function [v, w] = plan_cartesian_polynomials(qi, qf, ki, kf, s, s_dot)
   
    % CUBIC POLYNOMIALS
    if ki * kf < 0
        disp('error: ki and kf must have the same sign');
        v = []; w = [];
        return;
    end
    
    % Extract initial and final states
    xi = qi(1); yi = qi(2); thetai = qi(3);
    xf = qf(1); yf = qf(2); thetaf = qf(3);
    
    % Get a,b to satisfy boundary conditions
    a = [xf, xi, kf*cos(thetaf)-3*xf, ki*cos(thetai)+3*xi ];
    b = [yf, yi, kf*sin(thetaf)-3*yf, ki*sin(thetai)+3*yi ];
    
    % Obtain the geometric path
    x_s = a(1)*s.^3 - a(2)*(s.^3 - 3*s.^2 + 3*s - 1) + a(3)*(s.^3 - s.^2) + a(4)*(s.^3 - 2*s.^2 + s);
    y_s = b(1)*s.^3 - b(2)*(s.^3 - 3*s.^2 + 3*s - 1) + b(3)*(s.^3 - s.^2) + b(4)*(s.^3 - 2*s.^2 + s);
    
    x_s_dot = 3*a(1)*s.^2 - a(2)*(3*s.^2 - 6*s + 3) + a(3)*(3*s.^2 - 2*s) + a(4)*(3*s.^2 - 4*s + 1);
    y_s_dot = 3*b(1)*s.^2 - b(2)*(3*s.^2 - 6*s + 3) + b(3)*(3*s.^2 - 2*s) + b(4)*(3*s.^2 - 4*s + 1);
    
    x_s_ddot = 6*a(1)*s - a(2)*(6*s - 6) + a(3)*(6*s - 2) + a(4)*(6*s - 4);
    y_s_ddot = 6*b(1)*s - b(2)*(6*s - 6) + b(3)*(6*s - 2) + b(4)*(6*s - 4);
    
    % Geometric Inputs
    v_tilde = sqrt(x_s_dot.^2 + y_s_dot.^2);
    omega_tilde = (x_s_dot .* y_s_ddot - y_s_dot .* x_s_ddot) ./ (x_s_dot.^2 + y_s_dot.^2);
    
    % Convert into real Inputs
    v = v_tilde .* s_dot;
    w = omega_tilde .* s_dot;
end
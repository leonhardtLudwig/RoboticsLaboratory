function [v, w] = trajectory_plan_chained_translated(qi, qf, s, s_dot)
    
    % POLYNOMIALS
    
    % Extract initial and final physical states
    xi = qi(1); yi = qi(2); thetai = qi(3);
    xf = qf(1); yf = qf(2); thetaf = qf(3);
    
    % Convert to chained form (translated version)
    z1i = thetai;
    z2i = 0;
    z3i = 0; 
    
    z1f = thetaf;
    z2f = (xf - xi) * cos(thetaf) + (yf - yi) * sin(thetaf);
    z3f = (xf - xi) * sin(thetaf) - (yf - yi) * cos(thetaf);
    
    if any(z1f - z1i == 0)
        disp('error: no straightforward trajectory');
        v = []; w = [];
        % WE CAN ADD 2 SOLUTIONS (see notes)
        return;
    end

    % Get a, b to satisfy boundary conditions
    a = [z1f, z1i];
    b = [z3f, z3i, z2f.*(z1f - z1i) - 3.*z3f, z2i.*(z1f - z1i) + 3.*z3i];
    
    % Obtain the geometric path 
    z1_s = a(1).*s - (s - 1).*a(2);
    z3_s = b(1).*s.^3 - b(2).*(s - 1).^3 + b(3).*s.^2.*(s - 1) + b(4).*s.*(s - 1).^2;
    
    z1_s_dot = a(1) - a(2);
    z3_s_dot = 3.*b(1).*s.^2 - 3.*b(2).*(s - 1).^2 + b(3).*(2.*s.*(s - 1) + s.^2) + b(4).*((s - 1).^2 + 2.*s.*(s - 1));
    
    z1_s_ddot = 0;
    z3_s_ddot = 6.*b(1).*s - 6.*b(2).*(s - 1) + b(3).*(6.*s - 2) + b(4).*(6.*s - 4);

    % Geometri Inputs
    v1_tilde = z1_s_dot;
    v2_tilde = (z3_s_ddot .* z1_s_dot - z3_s_dot .* z1_s_ddot ) ./ ( z1_s_dot.^2 );
    
    % Inverse trascformation
    w_tilde = v1_tilde;
    v_tilde = v2_tilde + z3_s .* w_tilde;
    
    % Convert to original coordinates
    v = v_tilde .* s_dot;
    w = w_tilde .* s_dot;

end


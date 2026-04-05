function [q_dot, omega_wheels] = DDR_Unicycle(q, u, d_actual, r_actual)

    if size(q, 2) > size(u, 2)
        q = q(:, 1:end-1);
    end
    
    % extract variables
    v = u(1, :);
    omega = u(2, :);
    theta = q(3, :);

    q_dot = zeros(size(q));
    
    q_dot(1, :) = v .* cos(theta);
    q_dot(2, :) = v .* sin(theta);
    q_dot(3, :) = omega;

    wL = (v ./ r_actual) - (d_actual / (2 * r_actual)) .* omega;
    wR = (v ./ r_actual) + (d_actual / (2 * r_actual)) .* omega;

    omega_wheels = [wL; wR];

end
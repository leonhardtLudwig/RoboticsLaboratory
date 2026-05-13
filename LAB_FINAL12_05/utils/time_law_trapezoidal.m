function [s, s_dot] = time_law_trapezoidal(t, Ta, Tc)
            
    vc = 1/(Ta+Tc);
    a = vc/Ta;
   
    % Preallocate vectors with the same size as t
    s = zeros(size(t));
    s_dot = zeros(size(t));
    
    % Phase 1: Acceleration
    idx1 = (t < Ta);
    s_dot(idx1) = a * t(idx1);
    s(idx1) = 0.5 * a * t(idx1).^2;
    
    % Phase 2: Constant velocity
    idx2 = (t >= Ta) & (t < (Ta + Tc));
    s_dot(idx2) = vc;
    s(idx2) = 0.5 * a * Ta^2 + vc * (t(idx2) - Ta);
    
    % Phase 3: Deceleration
    idx3 = (t >= (Ta + Tc)) & (t < (2*Ta + Tc));
    dt3 = t(idx3) - Ta - Tc; % Temporary variable for cleaner code
    s_dot(idx3) = vc - a * dt3;
    s(idx3) = 0.5 * a * Ta^2 + vc * Tc + vc * dt3 - 0.5 * a * dt3.^2;
    
    % Phase 4: End (t >= 2*Ta + Tc)
    idx4 = (t >= (2*Ta + Tc));
    s_dot(idx4) = 0;
    s(idx4) = 1;
    
end
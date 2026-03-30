function [s, s_dot] = timing_law_trapezoidal(t)
    
    % considering T_SIM = 10s (otherwise 10% of T_SIM maybe)
    Ta = 1;
    Tc = 8; 

    %Ta = T * 0.1;
    %Tc = T - 2 * Ta;
        
    vc = 1/(Ta+Tc);
    a = vc/Ta;
   
    if t < Ta
        s_dot = a*t;
        s = 0.5*a*t^2;
    
    elseif t < (Ta+Tc)
        s_dot = vc;
        s = 0.5*a*Ta^2 + vc*(t - Ta);
    
    elseif t < (2*Ta+Tc)
        s_dot = vc - a*(t - Ta - Tc);
        s = 0.5*a*Ta^2 + vc*Tc + vc*(t - Ta - Tc) ...
            - 0.5*a*(t - Ta - Tc)^2;
    else
        s_dot = 0;
        s = 1;
    end
    
end
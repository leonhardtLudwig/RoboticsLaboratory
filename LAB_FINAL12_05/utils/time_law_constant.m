function [s, s_dot] = time_law_constant(t, T_tot)
   
    s = t / T_tot;
    
    s(s > 1) = 1; 
    
    s_dot = ones(size(t)) / T_tot;
    
    % final velocity null
    s_dot(t > T_tot) = 0;

end
function [s, s_dot] = time_law_constant(t,T)

    if t == 0
        s = 0;
    else
        s = 1;
    end

    s_dot = (1 / T);

end
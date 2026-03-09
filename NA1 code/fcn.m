function [s, s_dot] = fcn(t)

Ta = 1;
Tc = 2;

T1 = Ta;
T2 = Ta + Tc;
T3 = 2*Ta + Tc;

vc = 1/(Ta+Tc);
a = vc/Ta;


% velocità
if t < T1
    s_dot = a*t;

elseif t < T2
    s_dot = vc;

elseif t <= T3
    s_dot = vc - a*(t - Ta - Tc);

else
    s_dot = 0;
end


% posizione
if t < T1
    s = 0.5*a*t^2;

elseif t < T2
    s = 0.5*a*Ta^2 + vc*(t - Ta);

elseif t < T3
    s = 0.5*a*Ta^2 + vc*Tc + vc*(t - Ta - Tc) ...
        - 0.5*a*(t - Ta - Tc)^2;

else
    s = 1;
end

end


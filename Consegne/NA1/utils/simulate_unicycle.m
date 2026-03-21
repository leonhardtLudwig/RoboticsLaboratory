function q = simulate_unicycle(q0, v, omega, Ts)
% q0: stato iniziale [3x1]
% v:  velocità lineare [1xN] o [Nx1]
% omega: velocità angolare [1xN] o [Nx1]
% Ts: tempo di campionamento

N = length(v);

q = zeros(3, N);
q(:,1) = q0;

for k = 1:N-1
    theta = q(3,k);

    q_dot = [v(k)*cos(theta);
             v(k)*sin(theta);
             omega(k)];

    q(:,k+1) = q(:,k) + Ts * q_dot;
end

end
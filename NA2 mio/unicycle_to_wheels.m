function omega_wheels = unicycle_to_wheels(u, r, d)

v = u(1,:);
omega = u(2,:);

omega_L = (v - d/2 * omega) / r;
omega_R = (v + d/2 * omega) / r;

omega_wheels = [omega_L; omega_R];

end
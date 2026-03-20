function [PHI, Y_out] = get_phi_reg(q, omega_wheels, T_s)
%{
q = [N,3]
omega_wheels = [N,2]
T_S = time sampling
Returns the regression matrix PHI and the output vector Y_out s.t.:
- Y_out = [3*(N-1),1] containing [delta_X, delta_Y, delta_theta]
- PHI = [3*(N-1), 2] s.t. Y_out = PHI*w with w=[2,1] = [r; r/d]
%}

% Get the targets
delta_X = q(2:end,1) - q(1:end-1,1);
delta_Y = q(2:end,2) - q(1:end-1,2);
delta_theta = q(2:end,3) - q(1:end-1,3);
Y_out = [delta_X;delta_Y;delta_theta];

% Get the regression matrix
delta_v = T_s/2*(omega_wheels(1:end-1,1) + omega_wheels(1:end-1,2));
delta_omega = T_s*(omega_wheels(1:end-1,2) - omega_wheels(1:end-1,1));
theta = q(1:end-1, 3);
PHI_X = delta_v.*cos(theta);
PHI_Y = delta_v.*sin(theta);
PHI_THETA = delta_omega;
PHI = [PHI_X, zeros(size(PHI_X));
       PHI_Y, zeros(size(PHI_Y));
       zeros(size(PHI_THETA)),PHI_THETA];
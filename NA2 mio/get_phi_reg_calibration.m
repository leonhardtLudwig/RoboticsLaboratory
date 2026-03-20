function [PHI_cal, Y_out] = get_phi_reg_calibration(q, omega_wheels, T_s, theta_offset)
%{
q = [N,3]
omega_wheels = [N,2]
T_S = time sampling
theta_offset = angle calibration offset
Returns the regression matrix PHI and the output vector Y_out s.t.:
- Y_out = [3*(N-1),1] containing [delta_X, delta_Y, delta_theta]
- PHI = [3*(N-1), 4] s.t. Y_out = PHI*w with w=[4,1] = [r; r/d; x_off*r/d, y_off*r/d]
%}

% Get the targets
delta_X = q(2:end,1) - q(1:end-1,1);
delta_Y = q(2:end,2) - q(1:end-1,2);
delta_theta = q(2:end,3) - q(1:end-1,3);
Y_out = [delta_X;delta_Y;delta_theta];

% Get the regression matrix
delta_v = T_s/2*(omega_wheels(1:end-1,1) + omega_wheels(1:end-1,2));
delta_omega = T_s*(omega_wheels(1:end-1,2) - omega_wheels(1:end-1,1));
theta_off = q(1:end-1, 3)-theta_offset;
N = size(q, 1)-1;
PHI_X = delta_v.*cos(theta_off);
PHI_Y = delta_v.*sin(theta_off);
PHI_THETA = delta_omega;
PHI = [PHI_X, zeros(size(PHI_X));
       PHI_Y, zeros(size(PHI_Y));
       zeros(size(PHI_THETA)),PHI_THETA];

% Get the regression matrix with translations
% TO DO COMPUTE PHI_trasl_cal [3*N,2] 
% (regression matrix of the features r/d x_off, r/d y_off)
% PHI_trasl_cal= ??
PHI_cal = [PHI, PHI_trasl_cal];
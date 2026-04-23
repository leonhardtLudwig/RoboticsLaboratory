function SE = get_SE_id_and_calibration(q4id, omega_wheels_cmd, T_s, theta_offset)
% Compute the regression matrix
[PHI, Y] = get_phi_reg_calibration(q4id, omega_wheels_cmd, T_s, theta_offset);
% Estimate parameters
w_hat = lsqlin(PHI,Y,[],[],[],[],[0,0,-inf, -inf]);
% Compute estimates
Y_hat = PHI*w_hat;
% Compute MSE
SE = sum((Y-Y_hat).^2);
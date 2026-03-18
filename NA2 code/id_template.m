clear all;
close all;
% addpath(genpath('utils'));

T_s = 0.04;
r_nominal = 0.03;
d_nominal = 0.165;
w_nominal = [r_nominal;r_nominal/d_nominal];

%% Load and plot signals
% load from a simulation:
% q4id: [N, 3] matrix with positions and theta
% omega_wheels: [N,2] matrix with omega_L and omega_R
% t: time
N_samples = size(q4id, 1) - 1;


%% Test identification without calibration
% get the regression matrix and output
[PHI, Y] = get_phi_reg(q4id, omega_wheels, T_s);
delta_X = Y(1:N_samples);
delta_Y = Y(N_samples+1:2*N_samples);
delta_theta = Y(2*N_samples+1:3*N_samples);
% compute unconstrained solution
w_unconstrained_hat = (PHI'*PHI)\PHI'*Y;
r_unconstrained_hat = w_unconstrained_hat(1);
d_unconstrained_hat = w_unconstrained_hat(1)/w_unconstrained_hat(2);
% compute constrained solution
w_constr_hat = lsqlin(PHI,Y,[],[],[],[],[0,0]);
r_constr_hat = w_unconstrained_hat(1);
d_constr_hat = w_unconstrained_hat(1)/w_unconstrained_hat(2);
% compute estimate
Y_unconstrained_hat = PHI*w_unconstrained_hat;
delta_X_unconstrained_hat = Y_unconstrained_hat(1:N_samples);
delta_Y_unconstrained_hat = Y_unconstrained_hat(N_samples+1:2*N_samples);
delta_theta_unconstrained_hat = Y_unconstrained_hat(2*N_samples+1:3*N_samples);


%% Test identification with calibration
% set initial value
offset_0 = 0;
% setup objective function (function of orientation offset)
f_SE = @(w) get_SE_id_and_calibration(q4id, omega_wheels, T_s, w);
% optimize the offset with nonlinear opt
offset_hat = fminsearch(f_SE,offset_0);
% compute [r, r/d, x_off*r/d, y_off*r/d] estimates
[PHI_cal, Y] = get_phi_reg_calibration(q4id, omega_wheels, T_s, offset_hat);
w_cal_hat = lsqlin(PHI_cal,Y,[],[],[],[],[0,0, -inf, -inf]);
r_cal_hat = w_cal_hat(1);
d_cal_hat = w_cal_hat(1)/w_cal_hat(2);
x_off_cal_hat = w_cal_hat(3)/w_cal_hat(2);
y_off_cal_hat = w_cal_hat(4)/w_cal_hat(2);
% compute estimates
Y_cal_hat = PHI_cal*w_cal_hat;
delta_X_cal_hat = Y_cal_hat(1:N_samples);
delta_Y_cal_hat = Y_cal_hat(N_samples+1:2*N_samples);
delta_theta_cal_hat = Y_cal_hat(2*N_samples+1:3*N_samples);

%% Plot estimates

% plot X estimates
figure()
subplot(3,1,1)
plot(t(2:end), delta_X, 'LineWidth', 3, 'Color','k')
xlim([0,t(end)])
hold on
grid on
xlabel('Steps')
ylabel('\delta X [m]')
plot(t(2:end), delta_X_unconstrained_hat(1:N_samples), 'LineWidth', 1.5, 'Color', 'r')
plot(t(2:end), delta_X_cal_hat(1:N_samples), 'LineWidth', 1.5, 'Color', 'b')
legend('\delta X', '\delta X ID', '\delta X ID + CAL')
% compute error statistics
E_X = delta_X-delta_X_cal_hat;
mean_E_X = mean(E_X);
std_E_X = std(E_X);
std_X = std(delta_X);

% plot Y estimates
subplot(3,1,2)
plot(t(2:end), delta_Y, 'LineWidth', 3, 'Color','k')
xlim([0,t(end)])
hold on
grid on
plot(t(2:end), delta_Y_unconstrained_hat, 'LineWidth', 1.5, 'Color', 'r')
plot(t(2:end), delta_Y_cal_hat, 'LineWidth', 1.5, 'Color', 'b')
legend('\delta Y', '\delta Y ID', '\delta Y ID + CAL')
xlabel('Steps')
ylabel('\delta Y [m]')
% compute error statistics
E_Y = delta_Y-delta_Y_cal_hat;
mean_E_Y = mean(E_Y);
std_E_Y = std(E_Y);
std_Y = std(delta_Y);

% plot theta estimates
subplot(3,1,3)
plot(t(2:end), delta_theta, 'LineWidth', 3, 'Color','k')
xlim([0,t(end)])
hold on
grid on
plot(t(2:end), delta_theta_unconstrained_hat, 'LineWidth', 1.5, 'Color', 'r')
plot(t(2:end), delta_theta_cal_hat, 'LineWidth', 1.5, 'Color', 'b')
legend('\delta \theta', '\delta \theta ID', '\delta \theta ID + CAL')
xlabel('Steps')
ylabel('\delta \theta [rad]')
% compute error statistics
E_theta = delta_theta-delta_theta_cal_hat;
mean_E_theta = mean(E_theta);
std_E_theta = std(E_theta);
std_theta = std(delta_theta);
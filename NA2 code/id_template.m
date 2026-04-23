%% Numerical Activity 2 (NA2): Localization and Identification
%% PART 2: IDENTIFICATION AND CALIBRATION

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));

%%
T_s = 0.04;
r_nominal = 0.03;
d_nominal = 0.165;
w_nominal = [r_nominal;r_nominal/d_nominal];

%% Load and plot signals
% load from a simulation:
% q4id: [N, 3] matrix with positions and theta
% omega_wheels: [N,2] matrix with omega_L and omega_R
% t: time
load("data_for_identification.mat")

%% get data_for_id from simulation

T_SIMULATION = 10;
r_actual = r_nominal;
d_actual = d_nominal;

% N_samples =T_SIMULATION* 1/T_s; % arbitrary
% s = linspace(0,1, N_samples); 
% R = 0.4;
% omega_trj = 2*pi;
% [x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot] = gen_eight_shape_trajectory(R, omega_trj, s);
% [q, u] = cartisian_flatness(x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot);
% Q_INIT = q(:,1);
% PHI_INIT = [0;0];

Q_INIT = [0;0;atan(0.5)];
PHI_INIT = [0;0];

% open_system('sim_identification.slx');
load_system('sim_identification.slx');
out = sim('sim_identification.slx',T_SIMULATION);

%%
q_sim = out.q_sim.signals.values;
q_des = out.q_des.signals.values;
omega_wheels_sim = out.omega_wheels_sim.signals.values;

N = size(q_sim, 3);

q4id = squeeze(q_des)'; % -> (N x 3)
omega_wheels = squeeze(omega_wheels_sim)'; % -> (N x 2)
t = out.tout;

%% Save workspace
saveFilename = 'data_for_identification.mat';
save(saveFilename);
fprintf('Workspace saved to %s\n', saveFilename);

%%
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
r_constr_hat = w_constr_hat(1);
d_constr_hat = w_constr_hat(1)/w_constr_hat(2);
% compute estimate
Y_unconstrained_hat = PHI*w_unconstrained_hat;
delta_X_unconstrained_hat = Y_unconstrained_hat(1:N_samples);
delta_Y_unconstrained_hat = Y_unconstrained_hat(N_samples+1:2*N_samples);
delta_theta_unconstrained_hat = Y_unconstrained_hat(2*N_samples+1:3*N_samples);


%% Test identification with calibration
% set initial value
offset_0 = 0.1;
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

open_system('sim_identification_2_6.slx');
load_system('sim_identification_2_6.slx');
out = sim('sim_identification_2_6.slx',T_SIMULATION);


%%

omega_wheels_sim = out.omega_wheels_sim.signals.values;
omega_wheels_unc = out.omega_wheels_unc.signals.values;
omega_wheels_con = out.omega_wheels_con.signals.values;
omega_wheels_cal = out.omega_wheels_cal.signals.values;

%% 
omega_sim = squeeze(omega_wheels_sim)';   % N x 2
omega_unc = squeeze(omega_wheels_unc)';   % N x 2
omega_con = squeeze(omega_wheels_con)';   % N x 2
omega_cal = squeeze(omega_wheels_cal)';   % N x 2

wL_sim = omega_sim(:,1);
wR_sim = omega_sim(:,2);

wL_unc = omega_unc(:,1);
wR_unc = omega_unc(:,2);

wL_con = omega_con(:,1);
wR_con = omega_con(:,2);

wL_cal = omega_cal(:,1);
wR_cal = omega_cal(:,2);

%% 

figure()
subplot(2,1,1)
plot(t, wL_sim, 'k', 'LineWidth', 0.5)
hold on
plot(t, wL_unc, 'r')
plot(t, wL_con, 'g')
plot(t, wL_cal, 'b')
grid on
legend('sim','ID','ID constr','ID + CAL')
title('Left wheel speed')
ylabel('\omega_L')


subplot(2,1,2)
plot(t, wR_sim, 'k', 'LineWidth', 0.5)
hold on
plot(t, wR_unc, 'r')
plot(t, wR_con, 'g')
plot(t, wR_cal, 'b')
grid on
legend('sim','ID','ID constr','ID + CAL')
title('Right wheel speed')
xlabel('time')
ylabel('\omega_R')

%% 
E_unc = omega_sim - omega_unc;
E_con = omega_sim - omega_con;
E_cal = omega_sim - omega_cal;

rmse_unc = sqrt(mean(E_unc.^2));
rmse_con = sqrt(mean(E_con.^2));
rmse_cal = sqrt(mean(E_cal.^2));





%% Experimental Activity 1 (EA1): Planning, Localization, and Identification
%% PART 2

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));
addpath(genpath(fullfile(pwd,'..','..','utils')));

%% Set simulation parameters
T_s = 0.04; 
r = 0.03;
d = 0.165;
omega_max = 10;

%% Load Data from Part1 and extract configuration 1 (Tc = 30s)

load('Parte1/DATI_LABORATORIO_PARTE_1.mat');

results = results_part1(1); % extract T=30 datas

disp(['Results with Ta, Tc: ', num2str(results.Ta), ', ', num2str(results.Tc)]);

T_SIM = 2*results.Ta + results.Tc;

% extract data
q_des = results.q_des;
q_motion_capture = results.q_motion_capture;
w_gyro = results.w_gyro;
acce = results.acce;
ws_des = results.ws_des;
ws_meas = results.ws_meas;


%% 2.1 - IDENTIFICATION / CALIBRATION

% as in NA2
q4id = q_motion_capture;  
q4id(:,3) = unwrap(q_motion_capture(:,3));  % correct ±2π jump

omega_wheels = ws_meas;  
N_samples = size(q4id, 1) - 1;

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
offset_0 =0;
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



% new motion capture state calibrated
theta_cal = q4id(:, 3) - offset_hat;
x_cal = q4id(:, 1) - (x_off_cal_hat .* cos(theta_cal) - y_off_cal_hat .* sin(theta_cal));
y_cal = q4id(:, 2) - (x_off_cal_hat .* sin(theta_cal) + y_off_cal_hat .* cos(theta_cal));


q_motion_capture_cal = [x_cal, y_cal, theta_cal];


%% 2.2 - TEST LOCALIZATION STRATEGIES
r_actual = r_cal_hat;
d_actual = d_cal_hat;

% New Identified params: r_actual = 0.03293 ; d_actual = 0.16040

%% Prepare Data for the Simulink (Part2)

N_samples = size(ws_meas, 1);
t_array = (0:N_samples-1)' * T_s;

wheels_speed_meas_ts = timeseries(ws_meas, t_array);
wheels_speed_meas_ts.DataInfo.Interpolation = tsdata.interpolation('zoh');

wheels_speed_des = ws_des(1:N_samples, :); 
wheels_speed_des_ts = timeseries(wheels_speed_des, t_array);
wheels_speed_des_ts.DataInfo.Interpolation = tsdata.interpolation('zoh');

omega_gyro = w_gyro(1:N_samples, :); 
omega_gyro_ts = timeseries(omega_gyro, t_array);
omega_gyro_ts.DataInfo.Interpolation = tsdata.interpolation('zoh');

% from calibration
q_motion_capture_cal = q_motion_capture_cal(1:N_samples, :);
q_motion_capture_ts = timeseries(q_motion_capture_cal, t_array);
q_motion_capture_ts.DataInfo.Interpolation = tsdata.interpolation('zoh');

%% Initial State for localization

Q_INIT = q_motion_capture_cal(1,:)';

Q_INIT_LOC = Q_INIT;

Z_INIT_EKF = [Q_INIT; 0; 0; 0; 0]; 
PHI_INIT = [0;0];


%% Observation matrix H for EKF

% Encorder
H_enc = [0, 0, 0, 1, 0, 0, 0;
         0, 0, 0, 0, 1, 0, 0];

% IMU (wz_gyro)
H_gyro = [0, 0, 0, 0, 0, -r_actual/d_actual, r_actual/d_actual];
     
% Motion capture (State)
H_motion_cap = [1, 0, 0, 0, 0, 0, 0;
                0, 1, 0, 0, 0, 0, 0;
                0, 0, 1, 0, 0, 0, 0]; 

%% Covariance matrix

ENCODER_QUANTIZATION = 2 * pi / 4096;

sigma_motion_capture = 8e-3;        
sigma_enc = ENCODER_QUANTIZATION/sqrt(12);  
sigma_imu = 1e-2;                 
 

% first attempt
%sigma_motion_capture = 1e-3;


% EKF initil covariance
P_INIT_EKF = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);

% EKF process covariance
% prof: D = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);

% [x,y,theta,deltaphiL,deltaphi_R,deltaphi_dotL,deltaphi_dotR]
% [position, .., heading (drift), enc_states, .. , .. , .. , ..]

%D_first_attempt = diag([1.5e-3, 1.5e-3, 8e-3, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
D = diag([0.8e-3, 0.8e-3, 5e-3, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);

% Encoder + IMU + motion capture
R_3 = diag(([sigma_motion_capture, sigma_motion_capture, sigma_motion_capture, ...
             sigma_enc, sigma_enc, sigma_imu]).^2);




%% VERSION WITH H FULL

% set simulation params

% test case 1: Only encoder 
% test case 2: Encoder + IMU 
% test case 3.3: Encoder + IMU + motion capture (𝑝𝑙𝑜𝑠𝑠 = 0.9) 
% test case 3.2: Encoder + IMU + motion capture (𝑝𝑙𝑜𝑠𝑠 = 0.99) 

test_case = 3;
p_loss_values = [1.0, 0.99, 0.9, 0.0];
p_loss = p_loss_values(2);     % useful just for case 3


%% Run simulation
simulink_model_name = 'Part2'; 
out = sim(simulink_model_name);

% Collect Data (optional)

q_loc_euler = out.q_loc_euler.signals.values;
q_loc_rk2 = out.q_loc_rk2.signals.values;
q_loc_exact = out.q_loc_exact.signals.values;

% plot_localization_results(q_motion_capture_cal', q_loc_euler, q_loc_rk2, q_loc_exact);

z_estimate = out.z_EKF.signals.values;
P_filt_EKF = out.P_filt_EKF.signals.values;

q_loc_EKF = z_estimate(1:3,:,:);

%% Plot e Analisi

% plot_ekf_uncertainty(t_array, q_loc_EKF, P_filt_EKF, q_motion_capture_cal)
plot_covariance_analysis(t_array, P_filt_EKF)

plot_EKF_results(q_motion_capture', q_loc_exact, q_loc_EKF);
plot_ekf_error_analysis(t_array, q_loc_EKF, P_filt_EKF, q_motion_capture_cal)


% --- Aggiunta Ellissi di Covarianza al Plot 2D ---
% --- Plot Pulito della Traiettoria con Ellissi (Versione Report) ---

figure('Name', 'EKF Trajectory with Covariance Ellipses', 'Color', 'w');
hold on; grid on; 
axis equal; % CRITICO: impedisce la distorsione ottica delle ellissi

% 1. Plotta prima le linee continue (Ground Truth e Stima)
% Assicurati che q_motion_capture sia [N x 3]
if size(q_motion_capture, 1) == 3
    q_motion_capture = q_motion_capture'; 
end
plot(q_motion_capture(:,1), q_motion_capture(:,2), 'k', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');

% Estrai tutta la traiettoria EKF
est_x_all = squeeze(z_estimate(1, 1, :));
est_y_all = squeeze(z_estimate(2, 1, :));
plot(est_x_all, est_y_all, 'r--', 'LineWidth', 1.5, 'DisplayName', 'EKF Estimate');

% 2. Plotta solo un numero fisso di ellissi (es. 6 in totale)
num_ellipses = 25;
step_plot = floor(length(t_array) / num_ellipses); 

% Facciamo partire il loop da step_plot per evitare l'ellisse a t=0 (che di solito è un punto minuscolo)
for k = step_plot : step_plot : length(t_array)
    x_est = z_estimate(1, 1, k);
    y_est = z_estimate(2, 1, k);
    
    % Estrai la sottomatrice 2x2
    P_spaziale = P_filt_EKF(1:2, 1:2, k);
    
    % Disegna l'ellisse
    plot_covariance_ellipse(x_est, y_est, P_spaziale, [1 0.2 0.2 0.5]);
    
    % Disegna il marker centrale
    plot(x_est, y_est, 'r.', 'MarkerSize', 12, 'HandleVisibility', 'off');
end

% Aggiungi un'ellisse fittizia invisibile solo per farla comparire nella legenda in modo pulito
plot(NaN, NaN, 'Color', [1 0.2 0.2 0.5], 'LineWidth', 1.5, 'DisplayName', 'Uncertainty (3\sigma)');

xlabel('X [m]'); ylabel('Y [m]');
title('EKF Spatial Uncertainty Evaluation');


%% Automatic Figure Export

base_name = 'final_test4';

output_folder = fullfile(pwd, 'grafici_report_1');

% Prende tutte le figure aperte
figs = findall(0, 'Type', 'figure');

counter = 1;

for k = 1:length(figs)

    fig = figs(k);

    % Verifica validità handle
    if ~isvalid(fig)
        continue;
    end

    filename = sprintf('%s_%02d.png', base_name, counter);

    fullpath = fullfile(output_folder, filename);

    try
        exportgraphics(fig, fullpath, 'Resolution', 300);
        counter = counter + 1;

    catch ME
        fprintf('Error exporting figure %d:\n%s\n', k, ME.message);
    end

end

disp('All valid figures exported successfully.');


%% Save Data

% test case 1: Only encoder 
% test case 2: Encoder + IMU 
% test case 3: Encoder + IMU + motion capture (𝑝𝑙𝑜𝑠𝑠 = 0.9) 
% test case 4: Encoder + IMU + motion capture (𝑝𝑙𝑜𝑠𝑠 = 0.99) 
i = 4;

out_backup = out; 
q_loc_euler = out.q_loc_euler.signals.values;
q_loc_rk2 = out.q_loc_rk2.signals.values;
q_loc_exact = out.q_loc_exact.signals.values;
z_estimate = out.z_EKF.signals.values;
P_filt_EKF = out.P_filt_EKF.signals.values;
      

results_part2(i).T_s = T_s;
results_part2(i).Ta = results.Ta; 
results_part2(i).Tc = results.Tc; 
results_part2(i).p_loss = p_loss;
results_part2(i).D = D;
results_part2(i).R = R_3;

results_part2(i).q_des = q_des;
results_part2(i).q_loc_euler = q_loc_euler;
results_part2(i).q_loc_rk2 = q_loc_rk2;
results_part2(i).q_loc_exact = q_loc_exact;
results_part2(i).q_motion_capture = q_motion_capture;
results_part2(i).q_motion_capture_cal = q_motion_capture_cal;
results_part2(i).w_gyro = w_gyro;
results_part2(i).ws_des = ws_des;
results_part2(i).ws_meas = ws_meas;
results_part2(i).P_filt_EKF = P_filt_EKF; 
results_part2(i).z_EKF = z_estimate;      

results_part2(i).out_backup = out;

%%
save('results_part2.mat', 'results_part2')



%%
function plot_covariance_ellipse(x, y, P_2x2, color_string)
    % PLOT_COVARIANCE_ELLIPSE Disegna l'ellisse di incertezza a 3-sigma
    % nel piano cartesiano data la posizione e la sottomatrice di covarianza.
    %
    % INPUT:
    %   x, y: coordinate stimate dal filtro (centro dell'ellisse)
    %   P_2x2: sottomatrice 2x2 estratta da P_filt_EKF(1:2, 1:2, k)
    %   color_string: colore della linea (es. 'r', 'b', o un array RGB)

    % Calcolo autovalori (D) e autovettori (V)
    [V, D] = eig(P_2x2);
    
    % Generazione dei punti di un cerchio unitario
    theta_grid = linspace(0, 2*pi, 100);
    circle = [cos(theta_grid); sin(theta_grid)];
    
    % Trasformazione: scala per 3*sigma e ruota secondo gli autovettori
    % D contiene le varianze, sqrt(D) sono le deviazioni standard
    ellipse_points = V * (3 * sqrt(D)) * circle;
    
    % Traslazione nel punto stimato
    X_ell = ellipse_points(1, :) + x;
    Y_ell = ellipse_points(2, :) + y;
    
    % Plot dell'ellisse
    plot(X_ell, Y_ell, 'Color', color_string, 'LineWidth', 1.2);
end





%%
% --- Aggiunta Ellissi di Covarianza al Plot 2D ---
% --- Plot Pulito della Traiettoria con Ellissi (Versione Report) ---
figure('Name', 'EKF Trajectory with Covariance Ellipses', 'Color', 'w');
hold on; grid on; 
axis equal; % CRITICO: impedisce la distorsione ottica delle ellissi

% 1. Plotta prima le linee continue (Ground Truth e Stima)
% Assicurati che q_motion_capture sia [N x 3]
if size(q_motion_capture, 1) == 3
    q_motion_capture = q_motion_capture'; 
end
plot(q_motion_capture(:,1), q_motion_capture(:,2), 'k', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');

% Estrai tutta la traiettoria EKF
est_x_all = squeeze(z_estimate(1, 1, :));
est_y_all = squeeze(z_estimate(2, 1, :));
plot(est_x_all, est_y_all, 'r--', 'LineWidth', 1.5, 'DisplayName', 'EKF Estimate');

% 2. Plotta solo un numero fisso di ellissi (es. 6 in totale)
num_ellipses = 5;
step_plot = floor(length(t_array) / num_ellipses); 

% Facciamo partire il loop da step_plot per evitare l'ellisse a t=0 (che di solito è un punto minuscolo)
for k = step_plot : step_plot : length(t_array)
    x_est = z_estimate(1, 1, k);
    y_est = z_estimate(2, 1, k);
    
    % Estrai la sottomatrice 2x2
    P_spaziale = P_filt_EKF(1:2, 1:2, k);
    
    % Disegna l'ellisse
    plot_covariance_ellipse(x_est, y_est, P_spaziale, [1 0.2 0.2 0.5]);
    
    % Disegna il marker centrale
    plot(x_est, y_est, 'r.', 'MarkerSize', 12, 'HandleVisibility', 'off');
end

% Aggiungi un'ellisse fittizia invisibile solo per farla comparire nella legenda in modo pulito
plot(NaN, NaN, 'Color', [1 0.2 0.2 0.5], 'LineWidth', 1.5, 'DisplayName', 'Uncertainty (3\sigma)');

xlabel('X [m]'); ylabel('Y [m]');
title('EKF Spatial Uncertainty Evaluation');

%%
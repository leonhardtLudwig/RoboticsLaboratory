%% Experimental Activity 1 (EA1): Planning, Localization, and Identification
%% PART 3

clear all;
close all;
addpath(genpath(fullfile(pwd,'..','utils')));

%% Set simulation parameters
T_s = 0.04; 
r = 0.03;
d = 0.165;
omega_max = 10;

r_id = 0.03316;
d_id = 0.18428;

r = r_id;
d = d_id;


%% Load Data from Part1 and extract configuration 1

load('EA1_Part3_Data.mat');

results = results_part3(1);
disp(['Results with Ta, Tc: ', num2str(results.Ta), ', ', num2str(results.Tc)]);

T_SIM = 2*results.Ta + results.Tc;


%% 
% tiro fuori i dati per comodità 
q_des = results.q_desired;
q_motion_capture = results.q_motion_capture.signals.values;
gyro = results.gyro.signals.values;
acce = results.acce.signals.values;
ws_des = results.wheels_speed_desired.signals.values;
ws_meas = results.wheels_speed_measured.signals.values;
q_loc_exact = results.q_loc_exact.signals.values;
q_loc_EKF = results.q_loc_kalman.signals.values;

%% HO PROVATO A RUOTARE I DATI, MA E' INUTILE PERCHE' COMUNQUE POSIZIONE INIZIALE SBALLATA
alpha = pi/2; 

X_old = q_motion_capture(:, 1);
Y_old = q_motion_capture(:, 2);
theta_old = q_motion_capture(:, 3);

% 1. Applica la Matrice di Rotazione per correggere X e Y
X_new = X_old * cos(alpha) - Y_old * sin(alpha);
Y_new = X_old * sin(alpha) + Y_old * cos(alpha);

% 2. Somma l'offset a theta
theta_new = theta_old + alpha;

% 3. Wrap to pi (Riporta l'angolo nel range [-pi, pi] per pulizia)
theta_new = atan2(sin(theta_new), cos(theta_new));

% Sovrascrivi i dati originali con quelli raddrizzati
q_motion_capture_new(:, 1) = X_new;
q_motion_capture_new(:, 2) = Y_new;
q_motion_capture_new(:, 3) = theta_new;



%%
%plot_unicycle_2D(q_des,50);

plot_unicycle_2D(q_motion_capture_new',50);

%plot_EKF_results(q_motion_capture_new', q_loc_exact, q_loc_EKF);

%%
plot_wheels_speed(ws_des',T_s);
plot_wheels_speed(ws_meas',T_s);

%%
w_gyro = -gyro(:,3);

t = results.gyro.time;

figure()
plot(t, w_gyro);


%% ANALISI CONFRONTO TRA VELOCITA' RUOTE DESIDERATE e MISURATE dall'ENC

% ws_des = ws_des;
ws_meas_inv(:,1) = ws_meas(:,2);  % bc enc gives [wR;wL]
ws_meas_inv(:,2) = ws_meas(:,1);

ws_meas = ws_meas_inv;

%%
plot_wheels_speed(ws_des',T_s);
plot_wheels_speed(ws_meas',T_s);

%%
N = size(ws_des, 1);
t = (0:N-1)' * T_s;

% 1. CALCOLO DELL'ERRORE E DEL BIAS
error_ws = ws_des - ws_meas;

%plot(t,error_ws);

% La media dell'errore ci rivela se c'è un offset costante
bias_L = mean(error_ws(:,1));
bias_R = mean(error_ws(:,2));

fprintf('\n--- RISULTATI ANALISI VELOCITÀ RUOTE ---\n');
fprintf('Bias Ruota Sinistra : %+.4f rad/s\n', bias_L);
fprintf('Bias Ruota Destra   : %+.4f rad/s\n', bias_R);
fprintf('Se questi valori sono vicini a 0 (es. < 0.05), non c''è offset rilevante.\n');

% 2. FILTRAGGIO (Media Mobile)
% Applichiamo un filtro per eliminare il rumore visivo. 
% Più alzi la window_size, più la curva sarà liscia (ma in ritardo).
window_size = 15; 
ws_meas_filt = movmean(ws_meas, window_size);

% 3. PLOTTING DIAGNOSTICO
figure('Name', 'Diagnostica: Rumore e Bias delle Ruote', 'Position', [100, 100, 1000, 600]);

% -- Riga 1: Confronto Segnali (con Misure Filtrate) --
% Ruota Sinistra
subplot(2,2,1);
plot(t, ws_des(:,1), 'k--', 'LineWidth', 1.5); hold on; grid on;
plot(t, ws_meas_filt(:,1), 'b', 'LineWidth', 1.5);
title('Ruota Sinistra (\omega_L)');
ylabel('[rad/s]');
legend('Desiderata', 'Misurata (Filtrata)', 'Location', 'best');

% Ruota Destra
subplot(2,2,2);
plot(t, ws_des(:,2), 'k--', 'LineWidth', 1.5); hold on; grid on;
plot(t, ws_meas_filt(:,2), 'r', 'LineWidth', 1.5);
title('Ruota Destra (\omega_R)');
ylabel('[rad/s]');
legend('Desiderata', 'Misurata (Filtrata)', 'Location', 'best');

% -- Riga 2: Plot dell'Errore (Il Rumore Raw) --
% Errore Ruota Sinistra
subplot(2,2,3);
plot(t, error_ws(:,1), 'Color', [0.5 0.5 1 0.5]); hold on; grid on;
yline(bias_L, 'k', 'LineWidth', 2); % Mostra la linea dell'offset
title('Errore Ruota Sinistra (Des - Meas)');
ylabel('Errore [rad/s]'); xlabel('Tempo [s]');
legend('Rumore Istantaneo', sprintf('Bias: %.3f', bias_L));

% Errore Ruota Destra
subplot(2,2,4);
plot(t, error_ws(:,2), 'Color', [1 0.5 0.5 0.5]); hold on; grid on;
yline(bias_R, 'k', 'LineWidth', 2); % Mostra la linea dell'offset
title('Errore Ruota Destra (Des - Meas)');
ylabel('Errore [rad/s]'); xlabel('Tempo [s]');
legend('Rumore Istantaneo', sprintf('Bias: %.3f', bias_R));



%% ANALISI confronto gyro - derivata di theta desired - derivata di theta mot cap

% Estraiamo i theta dal Mocap e dalla traiettoria desiderata
theta_mocap = q_motion_capture(:, 3); 
theta_des   = q_des(3, 1:301)';


% Applichiamo unwrap prima di derivare, fondamentale per evitare picchi infiniti!
% theta_mocap_unw = unwrap(theta_mocap);
% theta_des_unw   = unwrap(theta_des);
theta_mocap_unw = theta_mocap;
theta_des_unw   = theta_des;

%% CALCOLO DELLE DERIVATE (Velocità Angolari \omega)
% La funzione diff() restituisce un vettore più corto di 1 elemento.
% Aggiungiamo uno zero alla fine (o duplichiamo l'ultimo valore) per pareggiare le lunghezze.

w_mocap = diff(theta_mocap_unw) / T_s;
w_mocap = [w_mocap; w_mocap(end)]; % Pareggio la lunghezza con 't'

w_des = diff(theta_des_unw) / T_s;
w_des = [w_des; w_des(end)];       % Pareggio la lunghezza con 't'

w_gyro = -gyro(:,3) * (pi / 180) * 0.1; % Conversione in rad/s

w_odom = (r / d) * (ws_meas(:, 2) - ws_meas(:, 1));


%% OPZIONALE: Filtraggio della derivata del Mocap (scommenta se fa troppo rumore)
% Dato che la derivazione amplifica il rumore, un semplice filtro a media mobile aiuta.
% window_size = 5; 
% w_mocap = movmean(w_mocap, window_size);

figure('Name', 'Analisi Comparativa a 4 Vie');

% 1. Desiderata (Il traguardo ideale)
subplot(4,1,1);
plot(t, w_des, 'k', 'LineWidth', 1.5);
title('\omega Desiderata'); ylabel('[rad/s]'); grid on;

% 2. Mocap (La realtà esterna derivata)
subplot(4,1,2);
plot(t, w_mocap, 'b', 'LineWidth', 1.5);
title('\omega Mocap (\dot{\theta})'); ylabel('[rad/s]'); grid on;

% 3. Odometria (La realtà interna cinematica)
subplot(4,1,3);
plot(t, w_odom, 'g', 'LineWidth', 1.5);
title('\omega Odometria (Encoder)'); ylabel('[rad/s]'); grid on;

% 4. Giroscopio (Il sensore raw convertito)
subplot(4,1,4);
plot(t, w_gyro, 'r', 'LineWidth', 1.5);
title('\omega Giroscopio'); ylabel('[rad/s]'); grid on;


%%
N = size(w_odom, 1);
t = (0:N-1)' * T_s;

% 1. CALCOLO DELL'ERRORE E DEL BIAS
error_w = w_des - w_gyro;
max_err = max(error_w);

plot(t,error_w);

% La media dell'errore ci rivela se c'è un offset costante
bias = mean(error_w);

fprintf('\n--- RISULTATI ANALISI VELOCITÀ RUOTE ---\n');
fprintf('Bias vel ang : %+.4f rad/s\n', bias);
fprintf('errore max  %+.4f\n', max_err)
fprintf('Se questi valori sono vicini a 0 (es. < 0.05), non c''è offset rilevante.\n');


%%
%% ---------------------------------------------------------
%% CALIBRAZIONE GIROSCOPIO (Post-Conversione Nominale)
%% ---------------------------------------------------------

% 1. Applichiamo prima la conversione "da manuale" (decigradi -> rad/s)
w_gyro_nominale = -gyro(:,3) * 0.1 * (pi / 180); 

% 2. Scegliamo l'Odometria come "Ground Truth"
w_truth = w_des; 

% 3. Usiamo i Minimi Quadrati per trovare l'errore residuo
% Cerca la retta: w_truth = p(1) * w_gyro_nominale + p(2)
p = polyfit(w_gyro_nominale, w_truth, 1);

% Ora il fattore di scala è un moltiplicatore di correzione (ideale = 1.0)
correction_factor = p(1); 
% L'offset ora è calcolato direttamente sui dati già in rad/s
bias_gyro_rad     = p(2);

fprintf('\n--- CALIBRAZIONE GIROSCOPIO (SUI DATI IN RAD/S) ---\n');
fprintf('Fattore di correzione (k)   : %.4f (se = 1.0, la conversione nominale era perfetta)\n', correction_factor);
fprintf('Offset esatto in rad/s (b)  : %+.6f rad/s\n', bias_gyro_rad);

% 4. Applichiamo la calibrazione finale
w_gyro_calibrated = (w_gyro_nominale * correction_factor) + bias_gyro_rad;

% 5. Ricalcoliamo l'errore con i dati calibrati
error_w_cal = w_truth - w_gyro_calibrated;

fprintf('\nDopo la calibrazione finale:\n');
fprintf('Nuovo Bias dell''errore      : %+.6f rad/s\n', mean(error_w_cal));
fprintf('Nuovo Errore Massimo        : %+.4f rad/s\n', max(abs(error_w_cal)));

% 6. Plot di verifica visiva
figure('Name', 'Verifica Calibrazione Giroscopio (Post-Conversione)');

subplot(2,1,1);
plot(t, w_truth, 'g', 'LineWidth', 2); hold on; grid on;
plot(t, w_gyro_calibrated, 'r--', 'LineWidth', 1.5);
title('Confronto: Odometria vs Giroscopio Calibrato');
legend('\omega Odometria', '\omega Giroscopio Calibrato');
ylabel('[rad/s]');

subplot(2,1,2);
plot(t, error_w_cal, 'k'); grid on;
title('Errore residuo (Il rumore dinamico)');
xlabel('Tempo [s]'); ylabel('Errore [rad/s]');
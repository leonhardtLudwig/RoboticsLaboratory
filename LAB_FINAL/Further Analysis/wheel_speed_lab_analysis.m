%% Analisi delle traiettorie sotto le wheel speed desiderate (volendo anche quelle misurate) nell'esperienza pratica in lab

N = size(ws_des_lab, 2);
t_vec = (0:N-1)' * 0.04;

input_wL = [t_vec, ws_des_lab(1, :)'];
input_wR = [t_vec, ws_des_lab(2, :)'];

% N = size(ws_meas_lab, 1);
% t_vec = (0:N-1)' * 0.04;
% 
% input_wL = [t_vec, ws_meas_lab(:, 1)];
% input_wR = [t_vec, ws_meas_lab(:, 2)];

%%

%plot_unicycle_trajectory(q_model_lab,squeeze(out.q_ws_test.signals.values),'comp')

labels = {'Q-MODEL-LAB', 'Q-WS-TEST-MEAS', 'Q-EKF', 'Q-WF1'};

plot_4_unicycle_trajectories(q_model_lab,squeeze(out.q_ws_test.signals.values),q_ekf_lab,q_WF1,labels,'comp',10);
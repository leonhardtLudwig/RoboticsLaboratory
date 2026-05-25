function exp_struct = create_regulation_exp_struct(sim_out, T_seconds)
    % CREATE_REGULATION_EXP_STRUCT Estrae e formatta i dati dell'esperimento.
    %
    % Input:
    %   sim_out   - La struttura dati grezza (es. out1.out)
    %   T_seconds - Durata dell'esperimento da estrarre in secondi (es. 15.04)
    %
    % Output:
    %   exp_struct - Struct formattata per le funzioni di plotting
    
    % --- PARAMETRI FISSI ---
    Ts = 0.04; % Tempo di campionamento [s]
    q_target = [-1.6; -1.1; 0]; % Target fisso per tutti gli esperimenti
    
    % Calcolo del numero di campioni (round per evitare problemi di precisione float)
    N_samples = round(T_seconds / Ts);
    
    % Inizializza la struct vuota
    exp_struct = struct();
    
    % q_EKF (3 x N)
    temp_q_EKF = squeeze(sim_out.q_EKF.signals.values);
    exp_struct.q_EKF = temp_q_EKF(:, 1:N_samples);
    
    % ws_meas (N x 2)
    exp_struct.ws_meas = sim_out.ws_meas.signals.values(1:N_samples, :);
    
    % q_model (3 x N)
    temp_q_model = squeeze(sim_out.q_model.signals.values);
    exp_struct.q_model = temp_q_model(:, 1:N_samples);
    
    % ws_des (N x 2)
    temp_ws_des = squeeze(sim_out.ws_des.signals.values)';
    exp_struct.ws_des = temp_ws_des(1:N_samples, :);
    
    % gt (3 x N)
    exp_struct.gt = sim_out.vicon_gt.signals.values(1:N_samples, :)';
    
    % err_gt_des (3 x N)
    temp_err_gt = sim_out.err_actual.signals.values';
    exp_struct.err_gt_des = temp_err_gt(:, 1:N_samples);
    
    % err_model_des (3 x N)
    temp_err_model = squeeze(sim_out.err_model.signals.values);
    exp_struct.err_model_des = temp_err_model(:, 1:N_samples);
    
    % --- SEGNALI DI CONTROLLO (u = [v, omega]) ---
    temp_u = squeeze(sim_out.control_signal_u.signals.values);
    if size(temp_u, 1) == 2
        temp_u = temp_u'; % Traspone per avere N x 2
    end
    exp_struct.u = temp_u(1:N_samples, :);
    
    % --- NUOVI CAMPI: STATI INTERNI EKF (z e P) ---
    temp_z = squeeze(sim_out.z_EKF.signals.values);
    exp_struct.z_EKF = temp_z(:, 1:N_samples);
    
    temp_P = sim_out.P_filt_EKF.signals.values;
    exp_struct.P_filt_EKF = temp_P(:, :, 1:N_samples);
    
    % --- CALCOLI DERIVATI: ERRORI CARTESIANI ---
    exp_struct.err_ekf_des = q_target - exp_struct.q_EKF;
    exp_struct.err_ekf_des(3, :) = atan2(sin(exp_struct.err_ekf_des(3, :)), cos(exp_struct.err_ekf_des(3, :)));
    
    exp_struct.sim_real_gap = (exp_struct.q_model - exp_struct.q_EKF)';
    exp_struct.gt_real_gap  = (exp_struct.gt - exp_struct.q_EKF)';
    
    % --- NUOVO CAMPO: COORDINATE POLARI (rho, gamma, delta, theta') ---
    % 1. Trasformazione nel frame del target (x', y', theta') come da slide
    theta_d = q_target(3);
    dx = exp_struct.q_EKF(1, :) - q_target(1);
    dy = exp_struct.q_EKF(2, :) - q_target(2);
    
    xp = dx * cos(theta_d) + dy * sin(theta_d);
    yp = -dx * sin(theta_d) + dy * cos(theta_d);
    theta_prime = exp_struct.q_EKF(3, :) - theta_d;
    theta_prime = atan2(sin(theta_prime), cos(theta_prime)); % Wrap
    
    % 2. Calcolo variabili polari
    rho = sqrt(xp.^2 + yp.^2);
    
    gamma = atan2(yp, xp) + pi - theta_prime;
    gamma = atan2(sin(gamma), cos(gamma)); % Wrap per i grafici
    
    delta = theta_prime + gamma;
    delta = atan2(sin(delta), cos(delta)); % Wrap per i grafici
    
    % Salvataggio nella struct come matrice 4xN
    exp_struct.polar_states = [rho; gamma; delta; theta_prime];
end
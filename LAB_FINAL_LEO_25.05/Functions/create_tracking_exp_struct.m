function exp_struct = create_tracking_exp_struct(sim_out, T_seconds)
    % CREATE_TRACKING_EXP_STRUCT Estrae e formatta i dati dell'esperimento di tracking.
    
    % --- PARAMETRI FISSI ---
    Ts = 0.04; % Tempo di campionamento [s]
    
    % Calcolo del numero di campioni
    N_samples = round(T_seconds / Ts);
    
    % Inizializza la struct vuota
    exp_struct = struct();
    
    % --- TRAIETTORIA DESIDERATA ---
    temp_q_des = squeeze(sim_out.q_des.signals.values);
    exp_struct.q_des = temp_q_des(:, 1:N_samples);
    
    % q_EKF (3 x N)
    temp_q_EKF = squeeze(sim_out.q_EKF.signals.values);
    exp_struct.q_EKF = temp_q_EKF(:, 1:N_samples);
    
    % ws_meas (N x 2)
    exp_struct.ws_meas = sim_out.ws_meas.signals.values(1:N_samples, :);
    
    % --- CAMPO: MODELLI (Rumoroso e Ideale) ---
    % q_model_noise (3 x N)
    temp_q_model_noise = squeeze(sim_out.q_model_noise.signals.values);
    exp_struct.q_model_noise = temp_q_model_noise(:, 1:N_samples);
    
    % q_model (3 x N) - Legge da q_model_noiseless ma lo salva come q_model
    temp_q_model = squeeze(sim_out.q_model_noiseless.signals.values);
    exp_struct.q_model = temp_q_model(:, 1:N_samples);
    
    % ws_des (N x 2)
    temp_ws_des = squeeze(sim_out.ws_des.signals.values)';
    exp_struct.ws_des = temp_ws_des(1:N_samples, :);
    
    % gt (3 x N) - Estratto da timeseries e trasposto
    %exp_struct.gt = sim_out.y_vicon_ground.Data(1:N_samples, :)';
    % --- ESTRAZIONE ROBUSTA GROUND TRUTH ---
    % 1. Verifica il formato del dato in uscita da Simulink
    if isstruct(sim_out.y_vicon_ground)
        % Formato "Structure" o "Structure with Time"
        temp_gt = sim_out.y_vicon_ground.signals.values;
    elseif isa(sim_out.y_vicon_ground, 'timeseries')
        % Formato "Timeseries"
        temp_gt = sim_out.y_vicon_ground.Data;
    else
        % Fallback generico nel caso in cui sia stato salvato come "Array"
        temp_gt = sim_out.y_vicon_ground; 
    end
    
    % 2. Rimuove eventuali dimensioni unitarie tridimensionali (es. 1x3xN -> 3xN)
    temp_gt = squeeze(temp_gt);
    
    % 3. Assicura l'orientamento corretto: vogliamo N righe e 3 colonne per fare lo slicing
    if size(temp_gt, 1) == 3 && size(temp_gt, 2) ~= 3
        temp_gt = temp_gt';
    end
    
    % 4. Estrae i campioni desiderati e traspone per avere il formato finale 3 x N
    exp_struct.gt = temp_gt(1:N_samples, :)';
    
    % err_gt_des (3 x N)
    temp_err_gt = squeeze(sim_out.err_actual.signals.values);
    exp_struct.err_gt_des = temp_err_gt(:, 1:N_samples);
    exp_struct.err_gt_des(3, :) = atan2(sin(exp_struct.err_gt_des(3, :)), cos(exp_struct.err_gt_des(3, :)));

    
    % err_model_des (3 x N)
    temp_err_model = squeeze(sim_out.err_model.signals.values);
    exp_struct.err_model_des = temp_err_model(:, 1:N_samples);
    
    % --- SEGNALI DI CONTROLLO (u = [v, omega]) ---
    temp_u = squeeze(sim_out.control_signal_u.signals.values);
    if size(temp_u, 1) == 2
        temp_u = temp_u'; % Traspone per avere N x 2
    end
    exp_struct.u = temp_u(1:N_samples, :);
    
    % --- CALCOLI DERIVATI ---
    % Errore EKF vs Desired
    exp_struct.err_ekf_des = exp_struct.q_des - exp_struct.q_EKF;

    
    % FIX ANGLE WRAPPING PER I GRAFICI
    exp_struct.err_ekf_des(3, :) = atan2(sin(exp_struct.err_ekf_des(3, :)), cos(exp_struct.err_ekf_des(3, :)));
    
    
    % Reality Gaps
    exp_struct.sim_real_gap_noise = (exp_struct.q_model_noise - exp_struct.q_EKF)';
    exp_struct.sim_real_gap_noise(3, :) = atan2(sin(exp_struct.sim_real_gap_noise(3, :)), cos(exp_struct.sim_real_gap_noise(3, :)));

    exp_struct.sim_real_gap = (exp_struct.q_model - exp_struct.q_EKF)';
    exp_struct.sim_real_gap(3, :)= atan2(sin(exp_struct.sim_real_gap(3, :)), cos(exp_struct.sim_real_gap(3, :)));
    
    exp_struct.gt_real_gap = (exp_struct.gt - exp_struct.q_EKF)';
    exp_struct.gt_real_gap(3, :)  =atan2(sin(exp_struct.gt_real_gap(3, :)), cos(exp_struct.gt_real_gap(3, :)));
end
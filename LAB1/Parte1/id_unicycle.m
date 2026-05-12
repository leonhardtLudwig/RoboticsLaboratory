function id_out = id_unicycle(results)
    % IDENTIFY_UNICYCLE_KINEMATICS_LAB Esegue l'identificazione basata sulla struct di laboratorio
    %
    % Input:
    %   results - Struct con i campi: Ta, Tc, q_des(3xN), w_gyro(Nx1), 
    %             acce(Nx3), q_motion_capture(3xN), ws_des(Nx2), ws_meas(Nx2)
    %
    % Output:
    %   id_out  - Struttura con i parametri stimati e la traiettoria calibrata
    %% 1. Parametri e Preprocessing dei dati
    T_s = 0.04; % Tempo di campionamento fisso come richiesto
    % Estrazione e TRASPOSIZIONE per avere i campioni sulle righe (Nx3)
    q_motion_capture = results.q_motion_capture'; % Da 3xN diventa Nx3
    ws_meas = results.ws_meas;                    % È già Nx2
    % Preparazione array di identificazione
    q4id = q_motion_capture;  
    q4id(:,3) = unwrap(q_motion_capture(:,3));  % Corregge i salti di ±2π
    omega_wheels = ws_meas;
    
    N_samples = size(q4id, 1) - 1; 
    %% 2. Identificazione senza calibrazione
    [PHI, Y] = get_phi_reg(q4id, omega_wheels, T_s);
    
    delta_X = Y(1:N_samples);
    delta_Y = Y(N_samples+1:2*N_samples);
    delta_theta = Y(2*N_samples+1:3*N_samples);
    
    % Soluzione non vincolata
    w_unconstrained_hat = (PHI'*PHI)\PHI'*Y;
    r_unconstrained_hat = w_unconstrained_hat(1);
    d_unconstrained_hat = w_unconstrained_hat(1)/w_unconstrained_hat(2);
    
    % Soluzione vincolata
    w_constr_hat = lsqlin(PHI, Y, [], [], [], [], [0, 0]);
    r_constr_hat = w_constr_hat(1);
    d_constr_hat = w_constr_hat(1)/w_constr_hat(2);
    
    % Stime
    Y_unconstrained_hat = PHI*w_unconstrained_hat;
    delta_X_unconstrained_hat = Y_unconstrained_hat(1:N_samples);
    delta_Y_unconstrained_hat = Y_unconstrained_hat(N_samples+1:2*N_samples);
    delta_theta_unconstrained_hat = Y_unconstrained_hat(2*N_samples+1:3*N_samples);
    %% 3. Identificazione con calibrazione
    offset_0 = 0;
    
    % Setup funzione obiettivo e ottimizzazione
    f_SE = @(w) get_SE_id_and_calibration(q4id, omega_wheels, T_s, w);
    offset_hat = fminsearch(f_SE, offset_0);
    
    % Stime [r, r/d, x_off*r/d, y_off*r/d]
    [PHI_cal, Y_cal] = get_phi_reg_calibration(q4id, omega_wheels, T_s, offset_hat);
    
    w_cal_hat = lsqlin(PHI_cal, Y_cal, [], [], [], [], [0, 0, -inf, -inf]);
    r_cal_hat = w_cal_hat(1);
    d_cal_hat = w_cal_hat(1)/w_cal_hat(2);
    x_off_cal_hat = w_cal_hat(3)/w_cal_hat(2);
    y_off_cal_hat = w_cal_hat(4)/w_cal_hat(2);
    
    % Calcolo stime deltas
    Y_cal_hat = PHI_cal*w_cal_hat;
    delta_X_cal_hat = Y_cal_hat(1:N_samples);
    delta_Y_cal_hat = Y_cal_hat(N_samples+1:2*N_samples);
    delta_theta_cal_hat = Y_cal_hat(2*N_samples+1:3*N_samples);
    %% 4. Ricalcolo dello stato della motion capture calibrato
    theta_cal = q4id(:, 3) - offset_hat;
    x_cal = q4id(:, 1) - (x_off_cal_hat .* cos(theta_cal) - y_off_cal_hat .* sin(theta_cal));
    y_cal = q4id(:, 2) - (x_off_cal_hat .* sin(theta_cal) + y_off_cal_hat .* cos(theta_cal));
    
    q_motion_capture_cal = [x_cal, y_cal, theta_cal];
    %% 5. Plot Stime (Deltas)
    t = (0:N_samples-1) * T_s;  % Vettore tempi
    
    % Estrazione Ta e Tc per il titolo
    Ta = results.Ta;
    Tc = results.Tc;
    
    % Creazione stringa titolo dinamicamente
    titolo_fig = sprintf('Confronto Stime Cinematiche (Ta = %g, Tc = %g)', Ta, Tc);
    
    figure('Name', titolo_fig, 'Color', 'w')
    sgtitle(titolo_fig, 'FontWeight', 'bold', 'FontSize', 12) % Aggiunge il titolo sopra ai subplot
    
    subplot(3,1,1)
    plot(t, delta_X, 'k', 'LineWidth', 2); hold on; grid on
    plot(t, delta_X_unconstrained_hat(1:N_samples), 'r', 'LineWidth', 1.5)
    plot(t, delta_X_cal_hat(1:N_samples), 'b', 'LineWidth', 1.5)
    legend('\delta X', '\delta X ID', '\delta X ID + CAL')
    ylabel('\delta X [m]')
    subplot(3,1,2)
    plot(t, delta_Y, 'k', 'LineWidth', 2); hold on; grid on
    plot(t, delta_Y_unconstrained_hat, 'r', 'LineWidth', 1.5)
    plot(t, delta_Y_cal_hat, 'b', 'LineWidth', 1.5)
    legend('\delta Y', '\delta Y ID', '\delta Y ID + CAL')
    ylabel('\delta Y [m]')
    subplot(3,1,3)
    plot(t, delta_theta, 'k', 'LineWidth', 2); hold on; grid on
    plot(t, delta_theta_unconstrained_hat, 'r', 'LineWidth', 1.5)
    plot(t, delta_theta_cal_hat, 'b', 'LineWidth', 1.5)
    legend('\delta\theta', '\delta\theta ID', '\delta\theta ID + CAL')
    ylabel('\delta\theta [rad]'); xlabel('time [s]')
    %% 6. Stampa dei Risultati a Console
    fprintf('--- IDENTIFICATION ---\n')
    fprintf('r_unconstrained = %.5f m\n', r_unconstrained_hat)
    fprintf('d_unconstrained = %.5f m\n', d_unconstrained_hat)
    fprintf('r_constrained   = %.5f m\n', r_constr_hat)
    fprintf('d_constrained   = %.5f m\n\n', d_constr_hat)
    fprintf('--- CALIBRATION ---\n')
    fprintf('r_cal     = %.5f m\n', r_cal_hat)
    fprintf('d_cal     = %.5f m\n', d_cal_hat)
    fprintf('x_off_cal = %.5f m\n', x_off_cal_hat)
    fprintf('y_off_cal = %.5f m\n', y_off_cal_hat)
    fprintf('theta_off = %.5f rad\n\n', offset_hat)
    % Calcolo Error statistics
    E_X     = delta_X     - delta_X_cal_hat;
    E_Y     = delta_Y     - delta_Y_cal_hat;
    E_theta = delta_theta - delta_theta_cal_hat;
    fprintf('--- ERROR STATS (ID + CAL) ---\n')
    fprintf('X:     mean=%.2e  std=%.2e\n', mean(E_X), std(E_X))
    fprintf('Y:     mean=%.2e  std=%.2e\n', mean(E_Y), std(E_Y))
    fprintf('theta: mean=%.2e  std=%.2e\n', mean(E_theta), std(E_theta))
    %% 7. Salvataggio Output
    id_out.unc.r = r_unconstrained_hat;
    id_out.unc.d = d_unconstrained_hat;
    
    id_out.con.r = r_constr_hat;
    id_out.con.d = d_constr_hat;
    
    id_out.cal.r = r_cal_hat;
    id_out.cal.d = d_cal_hat;
    id_out.cal.x_off = x_off_cal_hat;
    id_out.cal.y_off = y_off_cal_hat;
    id_out.cal.offset_theta = offset_hat;
    
    % Salvo la traiettoria ricalibrata (ritrasposta in 3xN per coerenza con l'input)
    id_out.q_motion_capture_cal = q_motion_capture_cal';
end
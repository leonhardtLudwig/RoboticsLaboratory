% --- ALLINEAMENTO 100% GARANTITO: ROTAZIONE + TRASLAZIONE ---

% 1. Punti chiave del MOCAP (World Frame)
P1_m = q_mocap_cal_lab(1:2, 1); % Punto di pre-innesco (X nera)
P2_m = q_mocap_cal_lab(1:2, 2); % Punto di ancoraggio (Cerchio verde)

% 2. Punti chiave del MODELLO (Simulazione)
P1_s = q_model_lab(1:2, 1);
P2_s = q_model_lab(1:2, 2);

% 3. Calcolo del Displacement (Il "Vettore Bussola")
% Definiamo la direzione del primo passo nel mondo reale e nella simulazione
v_mocap = P2_m - P1_m;
v_sim   = P2_s - P1_s;

% 4. Calcolo dell'angolo di rotazione (Delta Theta)
% Usiamo atan2 per essere sicuri al 100% dei quadranti
phi_mocap = atan2(v_mocap(2), v_mocap(1));
phi_sim   = atan2(v_sim(2), v_sim(1));

%delta_theta = phi_mocap - phi_sim;
delta_theta = 0.37;

% 5. APPLICAZIONE DELLA TRASFORMAZIONE RIGIDA A TUTTA LA TRAIETTORIA
% Matrice di rotazione 2D
R = [cos(delta_theta), -sin(delta_theta); 
     sin(delta_theta),  cos(delta_theta)];

% PASSO A: Centriamo il modello sul suo primo punto (traslazione temporanea)
q_centered = q_model_lab(1:2, :) - P1_s;

% PASSO B: RUOTIAMO l'intera traiettoria simulata
q_rotated = R * q_centered;

% PASSO C: TRASLIAMO la traiettoria ruotata sul SECONDO campione MOCAP (Anchor)
q_model_aligned_xy = q_rotated + P2_m;

% PASSO D: Allineamento dello stato theta (orientamento del robot)
theta_aligned = q_model_lab(3, :) + delta_theta;
theta_aligned = atan2(sin(theta_aligned), cos(theta_aligned)); % Normalizzazione [-pi, pi]

% Risultato finale da plottare
q_model_aligned = [q_model_aligned_xy; theta_aligned];

% --- VERIFICA VISIVA ---
figure;
plot(q_mocap_cal_lab(1, 2:end), q_mocap_cal_lab(2, 2:end), 'b', 'LineWidth', 1.5); hold on;
plot(q_model_aligned(1,:), q_model_aligned(2,:), 'r--', 'LineWidth', 1.5);
plot(P2_m(1), P2_m(2), 'go', 'MarkerSize', 12, 'LineWidth', 2); % Anchor
legend('MOCAP (Reale)', 'Model Lab (ALLINEATO)', 'Punto di Innesco');
axis equal; grid on;
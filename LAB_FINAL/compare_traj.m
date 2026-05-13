% --- PREPARAZIONE DATI ---
% Assicurati che siano matrici 2xN (solo X e Y)
ref  = q_model_lab(1:2, :); 
meas = q_ekf_lab(1:2, :); % o q_ekf_lab

% --- ESECUZIONE ALLINEAMENTO ---
[R_opt, t_opt, ref_aligned, rmse_post] = align_trajectories(ref, meas);

% --- ANALISI DEI RISULTATI ---
rmse_pre = sqrt(mean(sum((ref - meas).^2, 1)));
angolo_deg = rad2deg(atan2(R_opt(2,1), R_opt(1,1)));

fprintf('--- ANALISI CALIBRAZIONE ---\n');
fprintf('Rotazione rilevata: %.2f gradi\n', angolo_deg);
fprintf('Traslazione rilevata: X=%.3f m, Y=%.3f m\n', t_opt(1), t_opt(2));
fprintf('RMSE Originale: %.4f m\n', rmse_pre);
fprintf('RMSE dopo Allineamento: %.4f m\n', rmse_post);

% --- VERIFICA FINALE ---
if rmse_post < (rmse_pre / 5)
    fprintf('CONCLUSIONE: Errore sistematico di calibrazione confermato.\n');
else
    fprintf('CONCLUSIONE: L''errore non è solo rototraslazione. C''è un problema dinamico.\n');
end

% --- PLOT DI CONFRONTO ---
figure;
plot(meas(1,:), meas(2,:), 'b', 'LineWidth', 1.5); hold on;
plot(ref(1,:), ref(2,:), 'r--', 'LineWidth', 1);
plot(ref_aligned(1,:), ref_aligned(2,:), 'g', 'LineWidth', 2);
legend('Lab (Reale)', 'Ideale (Originale)', 'Ideale (Allineato)');
title('Verifica Ipotesi Calibrazione');
axis equal; grid on;
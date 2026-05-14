function [R, t, q_aligned, rmse] = align_trajectories(q_ref, q_meas)
    % q_ref: Traiettoria ideale [x; y] (2xN)
    % q_meas: Traiettoria da lab [x; y] (2xN)
    
    % 1. Calcolo i centroidi
    c_ref = mean(q_ref, 2);
    c_meas = mean(q_meas, 2);
    
    % 2. Centratura dei dati (rimuovo la traslazione temporaneamente)
    A = q_ref - c_ref;
    B = q_meas - c_meas;
    
    % 3. Calcolo della matrice di covarianza
    H = A * B';
    
    % 4. Singular Value Decomposition per trovare la rotazione ottimale
    [U, ~, V] = svd(H);
    R = V * U';
    
    % Gestione del caso di riflessione (determinante negativo)
    if det(R) < 0
        V(:, end) = V(:, end) * -1;
        R = V * U';
    end
    
    % 5. Calcolo della traslazione ottimale
    t = c_meas - R * c_ref;
    
    % 6. Applico la trasformazione alla traiettoria di riferimento
    % q_aligned = R * q_ref + t
    q_aligned = R * q_ref + t;
    
    % 7. Calcolo l'errore quadratico medio (RMSE)
    rmse = sqrt(mean(sum((q_aligned - q_meas).^2, 1)));
end
clc; 

%% Save Data

% test case 1: Only encoder 
% test case 2: Encoder + IMU 
% test case 3: Encoder + IMU + motion capture (𝑝𝑙𝑜𝑠𝑠 = 0.9) 
% test case 4: Encoder + IMU + motion capture (𝑝𝑙𝑜𝑠𝑠 = 0.99) 


%%
i = 3;

% Base configuration parameters
results(i).T_s = T_s;
%results(i).Ta = results.Ta; 
%results(i).Tc = results.Tc; 
results(i).p_loss = p_loss;
results(i).D = D;
results(i).R = R_3;
results(i).out_backup = out;

% Define the expected simulation signals
% Using standard naming to ensure 1:1 mapping with the 'out' struct
expected_signals = {
    'q_des', ...
    'q_loc_euler', ...
    'q_loc_rk2', ...
    'q_loc_exact', ...
    'q_motion_capture', ...
    'q_motion_capture_cal', ...
    'w_gyro', ...
    'acce', ... 
    'ws_model', ...
    'ws_des', ...
    'ws_meas', ...
    'P_filt_EKF', ...
    'z_EKF', ...
    'q_EKF', ...
    'control_signal_u',
};


% Check if 'out' is a Simulink.SimulationOutput object or a standard struct
% Extract the available variable names accordingly to ensure robust checks
if isa(out, 'Simulink.SimulationOutput')
    available_signals = out.who;
elseif isstruct(out)
    available_signals = fieldnames(out);
else
    error('The simulation output "out" is neither a struct nor a Simulink.SimulationOutput object.');
end

% Reference time vector length to handle N vs N+1 integration mismatches
if ismember('tout', available_signals)
    N_time = length(out.tout);
else
    warning('Time vector tout not found in simulation output. Using 0 as reference.');
    N_time = 0;
end

% Iterate over expected signals dynamically
for k = 1:length(expected_signals)
    sig_name = expected_signals{k};
    
    % Robust existence check bypassing isfield limitations on objects
    if ismember(sig_name, available_signals)
        % Extract raw values expecting 'Structure with Time' format
        raw_val = out.(sig_name).signals.values;
        
        % Silently correct dimensional mismatches (N+1 states vs N time steps)
        dim_val = size(raw_val);
        
        if N_time > 0
            % Handle 1D or 2D signals (e.g., states [N x M])
            if dim_val(1) > N_time
                raw_val = raw_val(1:N_time, :);
            % Handle 3D signals (e.g., covariance matrices [M x P x N])
            elseif length(dim_val) == 3 && dim_val(3) > N_time
                raw_val = raw_val(:, :, 1:N_time);
            end
        end
        
        % Assign valid and dimensionally correct data
        results(i).(sig_name) = raw_val;
    else
        % Assign an empty array to maintain structural consistency
        results(i).(sig_name) = [];
    end
end

disp("data save comleted")

%% Assign the name to save the results
res = results(3);



%%
plot_unicycle_2D(res.q_des, 50)

%% 
plot_unicycle_2D(res.q_motion_capture', 50)

%%
plot_unicycle_2D(res.q_EKF, 50)


















%%
% qui mettiamo il nome del simulink
sim_output = sim(REAL_TIME_TRACKING);

%% estrazione tag 

%funzione ternaria per tirare fuori i tag dalla simulazione
ternary_str = @(arr, s_true, s_false) subsref({s_true, s_false}, struct('type', '{}', 'subs', {{2 - (arr(1) == 0)}}));

T_s = (sim_output.tout(end)-sim_output.tout(1))/(length(sim_output.tout)-1);
env = ternary_str(sim_output.env.signals.values, 'sim', 'ros');
sim_noise_enabled = ternary_str(sim_output.sim_noise_enabled.signals.values, 'no', 'yes');
p_loss = sim_output.p_loss.signals.values(1);

%% 
empty_struct = struct('label', [],...
                       'env', [], ...
                      'sim_noise_enabled', [], ...
                      'T_s', [], ...
                      'Ta', [], ...
                      'Tc', [], ...
                      'p_loss', [], ...
                      'q_desired', [], ...
                      'q_loc_euler', [], ...
                      'q_loc_rk2', [],...
                      'q_loc_exact', [], ...
                      'motion_capture', [], ...
                      'q_motion_capture_cal', [], ...
                      'gyro', [], ...
                      'w_gyro', [], ...
                      'acce', [], ...
                      'wheels_speed_des', [], ...
                      'wheels_speed_meas', [], ...
                      'P_filt_EKF', [], ...
                      'z_EKF', [], ...
                      'state_error', [], ...
                      'control_signal_u', [], ...
                      'tout', [], ...
                      'out_backup', []);


%% salvataggio dati in struct 
label = 'LAB_DATI_circFL';

results = struct('label', label,...
                      'T_s', T_s, ...
                      'Ta', Ta, ...
                      'Tc', Tc, ...
                      'p_loss', p_loss, ...
                      'q_desired', q, ...
                      'q_loc_euler', sim_output.q_loc_euler, ...
                      'q_loc_rk2', sim_output.q_loc_rk2,...
                      'q_loc_exact', sim_output.q_loc_exact, ...
                      'motion_capture', sim_output.q_motion_capture, ...
                      'q_motion_capture_cal', sim_output.q_motion_capture_cal, ...
                      'gyro', sim_output.gyro, ...
                      'w_gyro', sim_output.w_gyro, ...
                      'acce', sim_output.acce, ...
                      'wheels_speed_des', sim_output.wheels_speed_des, ...
                      'wheels_speed_meas', sim_output.wheels_speed_meas, ...
                      'P_filt_EKF', sim_output.P_filt_EKF, ...
                      'z_EKF', sim_output.z_EKF, ...
                      'state_error', sim_output.state_error, ...
                      'control_signal_u', sim_output.control_signal_u, ...
                      'tout', sim_output.tout, ...
                      'out_backup', sim_output);

%% salvataggio struct nel workspace con nome unico

nome_cartella = 'DATI_LAB';

% 2. Controlla se la cartella esiste, altrimenti creala
if ~exist(nome_cartella, 'dir')
    mkdir(nome_cartella);
end

% 3. Genera il nome dinamico del file
timestamp = datestr(now, 'yyyy-mm-dd_HHMMSS');
nome_file = sprintf('Result_%s_%s.mat', results.label, timestamp);

% 4. Crea il percorso completo (Cartella/NomeFile)
% fullfile è la funzione migliore perché gestisce bene gli slash (/ o \) 
percorso_completo = fullfile(nome_cartella, nome_file);

% 5. Salva
save(percorso_completo, 'results');

fprintf('Dati salvati in: %s\n', percorso_completo);

%%
plot_unicycle_2D(results.q_motion_capture_cal.signals.values, 50)
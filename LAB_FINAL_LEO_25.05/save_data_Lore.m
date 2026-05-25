clc; 

%% Save Data

% test case 1: Only encoder 
% test case 2: Encoder + IMU 
% test case 3: Encoder + IMU + motion capture (𝑝𝑙𝑜𝑠𝑠 = 0.9) 
% test case 4: Encoder + IMU + motion capture (𝑝𝑙𝑜𝑠𝑠 = 0.99) 


%%
i = 1;

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
results(i).q_motion_capture = squeeze(results(i).q_motion_capture);


res = results(1);

%% SAVE
name = 'test_ctrl1.mat';
name_workspace = 'testctrl1_workspace.mat';

save(fullfile('TASK3_DATI', name), 'res');
save(fullfile('TASK3_DATI', name_workspace));

%%
plot_wheels_speed(res.ws_des', T_s);
%plot_wheels_speed(res.ws_meas, T_s);


%%
plot_unicycle_2D(res.q_des, 50)

%% 
plot_unicycle_2D(res.q_motion_capture, 50)

%%
plot_unicycle_2D(res.q_EKF, 50)
%%
plot_wheels_speed(res.ws_des, T_s);
plot_wheels_speed(res.ws_meas, T_s);





clear all;
close all;
addpath(genpath('utils'));

%% Set simulation parameters
T_s = 0.04; 
r = 0.03;
d = 0.165;
% r_actual = 0.031;
% d_actual = 0.164;
% wheels angles init
PHI_INIT = [0;0];

r_actual = r;
d_actual = d;


%% Eight-shape trajectory parameters
R = 0.4;
omega_trj = 2*pi;

%% Get an eight-shaped geometric path

% Sample the space variable s uniformly in [0,1]
N_samples = 1000; % arbitrary
s = linspace(0,1, N_samples); 

[x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot] = gen_eight_shape_trajectory(R, omega_trj, s);

%% Sample s and get q trajectory with differential flatness

% (u contains geometric input)
[q, u] = cartisian_flatness(x_s, y_s, x_s_dot, y_s_dot, x_s_ddot, y_s_ddot);

Q_INIT = q(:,1);


%% Plot the trajectory

plot_unicycle_2D(q, 50);

%% Get timing law

% all inside the simulink

%% 1.3 Sim_tracking

T_SIM = 10;

% good trajectory tracking. just an error when linear motion (flat w
% derivation)

%% 1.4 Sim_tracking

T_s = 0.1;  % 0.001 ; 0.04 ; 0.1

%%
simulink_model_name = 'Sim_tracking_1_3_4'; 
out = sim(simulink_model_name);

q_actual = out.q.signals.values;

plot_unicycle_2D(q_actual, 50);

% tracking works perfectly with all 3 T_s. Error start growing a bit with
% 0.1

%% EKF parameters

ENCODER_QUANTIZATION = 2 * pi / 4096;

% EKF initil covariance
P_INIT_EKF = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
% EKF process covariance
D = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
% EKF measurement noise (delta wheels angles)
R_2 = diag([ENCODER_QUANTIZATION/6,ENCODER_QUANTIZATION/6].^2);
% EKF measurement noise (GPS + delta wheels angles)
R_4 = diag(([0.001, 0.001, ENCODER_QUANTIZATION/6,ENCODER_QUANTIZATION/6]).^2);

Z_INIT_EKF = [Q_INIT; 0; 0; 0; 0]; 


%% 1.5 Sim_localization (manual)

Ts_values = [0.1, 0.04, 0.001];
T_s = Ts_values(2);

Q_INIT_LOC = Q_INIT;
% Q_INIT_LOC = Q_INIT + [0.2; 0.2; deg2rad(15)]; 
% (eventually with initial error)

%% plot localization state q vs the true state q

simulink_model_name = 'Sim_localization_1_5_6_7'; 
out = sim(simulink_model_name);

% t = out.q.time;
q_actual = out.q.signals.values;
q_loc_euler = out.q_loc_euler.signals.values;
q_loc_rk2 = out.q_loc_rk2.signals.values;
q_loc_exact = out.q_loc_exact.signals.values;

plot_localization_results(q_actual, q_loc_euler, q_loc_rk2, q_loc_exact);


% ANALYSIS: all 3 methods accumulates drift error
% with T_s = 0.001 localization is perfect. then the error grows with T_s
% Euler is less precise but with T_s small it converges to the others 

%% 1.5 Sim_localization (Automatic)
Ts_values = [0.1, 0.04, 0.001];
error_cases = [0, 1]; % 0 = without error, 1 = with initial error
simulink_model_name = 'Sim_localization_1_5_6_7'; 

% Preallocate results matrix
results_loc(length(Ts_values),length(error_cases)) = [];

for i = 1:length(Ts_values)
    T_s = Ts_values(i);
    
    for j = 1:length(error_cases)
        if error_cases(j) == 0
            fprintf('Execution: T_s = %.3f | WITHOUT initial error\n', T_s);
            Q_INIT_LOC = Q_INIT; 
        else
            fprintf('Execution: T_s = %.3f | WITH initial error\n', T_s);
            Q_INIT_LOC = Q_INIT + [0.2; 0.2; deg2rad(15)]; 
        end
        
        % 1. Run the simulation automatically
        out = sim(simulink_model_name);
        
        % 2. Data extraction and saving into a structured array
        results_loc(i, j).Ts = T_s;
        results_loc(i, j).initial_error = error_cases(j);
        results_loc(i, j).q_actual = out.q.signals.values;
        results_loc(i, j).q_loc_euler = out.q_loc_euler.signals.values;
        results_loc(i, j).q_loc_rk2 = out.q_loc_rk2.signals.values;
        results_loc(i, j).q_loc_exact = out.q_loc_exact.signals.values;
        
        % 3. Plot results
        plot_localization_results(results_loc(i, j).q_actual, ...
                                  results_loc(i, j).q_loc_euler, ...
                                  results_loc(i, j).q_loc_rk2, ...
                                  results_loc(i, j).q_loc_exact);
        
        name = sprintf('T_s = %.3f | Error = %d', T_s, error_cases(j));
        
        fig_path = findobj('Type', 'Figure', 'Name', 'Localization Paths (X-Y)');
        set(fig_path, 'Name', ['Path 2D: ', name]);    

        % Pause to analyze the plots before the next iteration
        %disp('Press any key in the Command Window to continue...');
        %pause; 
        
        % close all; % Close figures to avoid cluttering the screen
    end
end

%% 1.6 Extended Kalman Filter

%% initialization 

Ts_values = [0.1, 0.04];
T_s = Ts_values(2);

% ( robot start with vel = 0 )
Q_INIT_LOC = Q_INIT;
% Q_INIT_LOC = Q_INIT + [0.2; 0.2; deg2rad(15)]; 

Z_INIT_EKF = [Q_INIT_LOC; 0; 0; 0; 0]; 


%% 1.6 Sim_localization (manual)

simulink_model_name = 'Sim_localization_1_5_6_7'; 
out = sim(simulink_model_name);

t = out.q.time;

q_actual = out.q.signals.values;
q_loc_exact = out.q_loc_exact.signals.values;
z_estimate = out.z_EKF.signals.values;

plot_EKF_results(q_actual, q_loc_exact, z_estimate);

% ANALYSIS
% Kalman estimate is identical to the exact localization method
% both are using the wheels velocities as only information


%% 1.6 Sim_EKF (Automatic)
Ts_values = [0.1, 0.04]; % Test with two sampling times
error_cases = [0, 1]; % 0 = without error, 1 = with initial error

simulink_model_name = 'Sim_localization_1_5_6_7'; 

for i = 1:length(Ts_values)
    T_s = Ts_values(i);
    
    for j = 1:length(error_cases)
        if error_cases(j) == 0
            fprintf('Execution: T_s = %.3f | WITHOUT initial error\n', T_s);
            Q_INIT_LOC = Q_INIT; 
            Z_INIT_EKF = [Q_INIT; 0; 0; 0; 0];
        else
            fprintf('Execution: T_s = %.3f | WITH initial error\n', T_s);
            % Arbitrary initial error setup (e.g., 20 cm offset, 15 deg)
            Q_INIT_LOC = Q_INIT + [0.2; 0.2; deg2rad(15)]; 
            Z_INIT_EKF = [Q_INIT_LOC; 0; 0; 0; 0];
        end
        
        % 1. Run the simulation automatically
        out = sim(simulink_model_name);
        
        % 2. Data extraction and saving into structured array
        results_ekf(i, j).Ts = T_s;
        results_ekf(i, j).initial_error = error_cases(j);
        
        % Actual path and pure odometry (Exact) as baseline
        results_ekf(i, j).q_actual = out.q.signals.values;
        results_ekf(i, j).q_loc_exact = out.q_loc_exact.signals.values;
        results_ekf(i, j).z_estimate = out.z_EKF.signals.values;
        
        % 3. Plot results
        plot_EKF_results(results_ekf(i, j).q_actual, ...
                         results_ekf(i, j).q_loc_exact, ...
                         results_ekf(i, j).z_estimate);
        
        % Dynamic window renaming
        name = sprintf('T_s = %.3f | Error = %d', T_s, error_cases(j));
        
        fig_path = findobj('Type', 'Figure', 'Name', 'EKF Paths (X-Y)');
        set(fig_path, 'Name', ['Path 2D EKF: ', name]);
        
        fig_states = findobj('Type', 'Figure', 'Name', 'EKF States vs Time');
        set(fig_states, 'Name', ['States EKF: ', name]);

       
        % disp('Press any key in the Command Window to continue...');
        % pause; 
    end
end

%% Introduce error in the parameters for DDR model and Localization algorithms

r = 0.03;
d = 0.165;
r_actual = 0.031;
d_actual = 0.164;


%% 1.7 Parameters displacement

% run again 1.5 and 1.6 with T_s = 0.04

% Analysis
% (even with precise initial cond.) the localization works well at the
% begin but then, when the robot turns, it accumulates error and diverge
% from the real trajectory


%% 1.8 Kalman Localization with GPS system

% in the simulink: 
% H = [H3 ; H1] 
% R = R4
% noise measurements of x,y 
% check if GPS Data are valid

% params displacement 
r = 0.03;
d = 0.165;
r_actual = 0.031;
d_actual = 0.164;

noise_var = 0.01^2;

%% Initialization 

T_s = 0.04;

Q_INIT_LOC = Q_INIT;
% Q_INIT_LOC = Q_INIT + [0.2; 0.2; deg2rad(15)]; 

Z_INIT_EKF = [Q_INIT_LOC; 0; 0; 0; 0]; 

%% EKF parameters (proportional to T_s)

ENCODER_QUANTIZATION = 2 * pi / 4096;

% EKF initil covariance
P_INIT_EKF = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
% EKF process covariance
D = diag([0.001, 0.001, 0.0175/6, 0.0175/6, 0.0175/6, 0.0175/6*T_s, 0.0175/6*T_s].^2);
% EKF measurement noise (delta wheels angles)
R_2 = diag([ENCODER_QUANTIZATION/6,ENCODER_QUANTIZATION/6].^2);
% EKF measurement noise (GPS + delta wheels angles)
R_4 = diag(([0.001, 0.001, ENCODER_QUANTIZATION/6,ENCODER_QUANTIZATION/6]).^2);


%% Sim_localization_gps (manual)

p_loss = 0.99;   % 0.01, 0.09, 0.99

simulink_model_name = 'Sim_localization_gps_1_8'; 
out = sim(simulink_model_name);

q_actual = out.q.signals.values;
q_loc_exact = out.q_loc_exact.signals.values;
z_estimate = out.z_EKF.signals.values;

plot_EKF_results(q_actual, q_loc_exact, z_estimate);

% ANALYSIS
% with GPS data we can see the estimation is close tu the real trajectory
% but it has noise. Turning up the p_loss the estimate is less precise and
% more similar to le simle localization one

%%
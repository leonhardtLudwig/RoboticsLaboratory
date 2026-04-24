% Data Conversion Script: Old EA1 Format to New EA1 Format

clear all;
close all;

%%

old_results = results_part1;

%%
% Define the new empty structure rigorously
empty_struct = struct('T_s', [], ...
                      'Ta', [], ...
                      'Tc', [], ...
                      'q_desired', [], ...
                      'q_loc_exact', [], ...
                      'q_loc_kalman', [], ...
                      'gyro', [], ...
                      'acce', [], ...
                      'q_motion_capture', [], ...
                      'wheels_speed_desired', [], ...
                      'wheels_speed_measured', [], ...
                      'out_backup', []);

num_tests = 3;
new_results = repmat(empty_struct, 1, num_tests);

% Defined Tc values used in the experimental setup
Tc_values = [30, 18, 45];

for i = 1:num_tests
    % Copy existing fields directly
    new_results(i).T_s = old_results(i).T_s;
    new_results(i).Ta = old_results(i).Ta;
    new_results(i).q_desired = old_results(i).q_desired;
    new_results(i).q_loc_exact = old_results(i).q_loc_exact;
    new_results(i).q_loc_kalman = old_results(i).q_loc_kalman;
    new_results(i).gyro = old_results(i).gyro;
    new_results(i).acce = old_results(i).acce;
    new_results(i).out_backup = old_results(i).out_backup;
    new_results(i).q_motion_capture = old_results(i).odometry;
    
    % Assign Tc based on the index
    if i <= length(Tc_values)
        new_results(i).Tc = Tc_values(i);
    else
        new_results(i).Tc = NaN; 
    end
    
    % Extract Simulink output object
    out = old_results(i).out_backup;
    

    % Extract Desired Wheel Speeds
    if isprop(out, 'wheel_speed_desired')
        new_results(i).wheels_speed_desired = out.wheel_speed_desired.signals.values;
    end
    % Extract Measured Wheel Speeds
    if isprop(out, 'wheel_speed_actual')
        new_results(i).wheels_speed_measured = out.wheel_speed_actual.signals.values;
    end
end

% Overwrite the variable and save to the new standard filename
results_part1 = new_results;
save('EA1_Part1_Data.mat', 'results_part1');
fprintf('Data conversion completed successfully. Saved to EA1_Part1_Data.mat\n');

%%



% Data Conversion Script: PART 3

clear all;
close all;

%%

old_results = results_part1;

%%
% Define the new empty structure rigorously
empty_struct = struct('T_s', [], ...
                      'Ta', [], ...
                      'Tc', [], ...
                      'p_loss', [], ...
                      'q_desired', [], ...
                      'q_loc_exact', [], ...
                      'q_loc_kalman', [], ...
                      'gyro', [], ...
                      'acce', [], ...
                      'q_motion_capture', [], ...
                      'wheels_speed_desired', [], ...
                      'wheels_speed_measured', [], ...
                      'out_backup', []);

num_tests = 3;
new_results = repmat(empty_struct, 1, num_tests);

% Defined Tc values used in the experimental setup
p_loss_values = [1.0, 0.99, 0];

for i = 1:num_tests

    new_results(i).T_s = T_s;
    new_results(i).Ta = Ta;
    new_results(i).Tc = Tc;
    new_results(i).p_loss = p_loss_values(i);
    
    new_results(i).gyro = old_results(i).out_backup.gyro;
    new_results(i).acce = old_results(i).out_backup.acce;
    new_results(i).q_motion_capture = old_results(i).out_backup.motion_capture;
    
    new_results(i).wheels_speed_desired = old_results(i).out_backup.wheels_speed_des;
    new_results(i).wheels_speed_measured = old_results(i).out_backup.wheels_speed_meas;
    
    new_results(i).q_loc_exact = old_results(i).out_backup.q_loc_exact1;
    new_results(i).q_loc_kalman = old_results(i).out_backup.z_EKF; 
    
    new_results(i).q_desired = q;
    
    new_results(i).out_backup = old_results(i).out_backup; 
end

% Overwrite the variable and save to the new standard filename
results_part3 = new_results;
save('EA1_Part3_Data.mat', 'results_part3');
fprintf('Data conversion completed successfully. Saved to EA3_Part3_Data.mat\n');

%%

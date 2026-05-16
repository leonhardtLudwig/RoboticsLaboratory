function control_par = update_control_par(controller_index)
if controller_index == 1
    % linear
    xi = 0.9; 
    a = 2;    % 1 to have zero saturation
    control_par = [xi, a, 0];
elseif controller_index ==2
    % nonlinear
    xi = 0.75; 
    b = 10;
    control_par = [xi, b, 0];
elseif controller_index ==3
    % feedback_linearization
    k1 = 1; 
    k2 = 2;
    b = 0.5;   % potrebbe dare problemi (divisione per zero)
    control_par = [k1, k2,b];
end
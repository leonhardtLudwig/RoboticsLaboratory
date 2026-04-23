%% Run trajectory and plot results

function [q] = run_trajectory_chained(qi, qf, T_SIM, T_s, translated)    
    
    % Time law
    t = 0:T_s:T_SIM;
    [s, s_dot] = time_law_constant(t, T_SIM);
    
    % Traj generation
    if translated == 0
        [v,w] = trajectory_plan_chained(qi, qf, s, s_dot);

    elseif translated == 1
        [v,w] = trajectory_plan_chained_translated(qi, qf, s, s_dot);

    else
        disp('translated value not valid (0,1)');
        v = []; w = [];
        return;
    end

    
    % State simulation
    q = simulate_unicycle(qi, v, w, T_s);
    
    % Plot functions for 2D and time
    plot_unicycle_2D(q,50)
    plot_unicycle_wrt_time(q, T_s);

end
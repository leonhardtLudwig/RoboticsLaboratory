%% Run trajectory and plot results

function [q, u] = run_trajectory_cartisian(qi, qf, ki, kf, T_SIM, T_s)    
    
    % Time law
    t = 0:T_s:T_SIM;
    [s, s_dot] = time_law_constant(t, T_SIM);
    
    % Traj generation
    [v,w] = trajectory_plan_cartisian(qi, qf, ki, kf, s,s_dot);
    
    % State simulation
    q = simulate_unicycle(qi, v, w, T_s);
    u = [v;w];
    
    % Plot functions for 2D and time
    plot_unicycle_2D(q,50)
    plot_unicycle_wrt_time(q, u, T_s);

end
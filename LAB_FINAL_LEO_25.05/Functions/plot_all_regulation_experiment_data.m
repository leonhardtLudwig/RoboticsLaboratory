function plot_all_regulation_experiment_data(exp_data, exp_title, exp_num)
    % PLOT_ALL_REGULATION_EXPERIMENT_DATA Genera tutti i grafici per un singolo esperimento.
    %
    % Input:
    %   exp_data  - La struct dell'esperimento (es. creata con create_regulation_exp_struct)
    %   exp_title - Stringa con il titolo dell'esperimento (es. 'Experiment 1: Nominal')
    %   exp_num   - Numero intero dell'esperimento (serve per separare le figure)
    
    % Calcola l'offset per le figure in base al numero dell'esperimento.
    % Ora abbiamo 5 figure per esperimento:
    % exp_num = 1 -> base_fig = 0  -> Figure 1, 2, 3, 4, 5
    % exp_num = 2 -> base_fig = 5  -> Figure 6, 7, 8, 9, 10
    base_fig = (exp_num - 1) * 7;
    
    % Richiama le 5 funzioni di plot usando l'offset calcolato
    plot_exp_errors(exp_data, exp_title, base_fig + 1, 'regulation');
    plot_gaps(exp_data, exp_title, base_fig + 2);
    plot_exp_trajectories(exp_data, exp_title, base_fig + 3,'regulation');
    plot_exp_wheel_velocities(exp_data, exp_title, base_fig + 4);
    
    plot_control_signals_regulation(exp_data, exp_title, base_fig + 5);
    plot_ekf_performance_bounds(exp_data, exp_title, base_fig + 6);
    plot_polar_states(exp_data, exp_title, base_fig + 7);
    
    fprintf('Plot generati per "%s" (Figure %d - %d).\n', exp_title, base_fig + 1, base_fig + 5);
end
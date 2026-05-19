function cfg = moo_config()
% MOO_CONFIG  Multi-Objective Optimization (NSGA-II) configuration.
%   Parameters specific to multi-objective optimization.

    cfg = struct();

    % --- NSGA-II Parameters ---
    cfg.n_objectives    = 3;         % Number of objectives
    cfg.objective_names = {'CL/CD (max)', 'Weight (min)', 'Range (max)'};
    cfg.obj_sense       = [-1, 1, -1]; % -1 = maximize, +1 = minimize

    % --- Crowding Distance ---
    cfg.crowding_inf_replacement = 1e12;  % Replace Inf with large finite value

    % --- Archive ---
    cfg.archive_size    = 200;       % Max Pareto archive size
    cfg.use_archive     = true;      % Maintain external archive of non-dominated solutions

    % --- Constraint Handling ---
    cfg.constraint_handling = 'penalty';  % 'penalty' or 'feasibility_first'
    cfg.penalty_factor  = 1000;      % Penalty multiplier for constraint violations

    % --- Termination ---
    cfg.max_generations = 300;       % Override ga_config for MOO if needed
    cfg.hypervolume_tol = 1e-4;      % Stop if hypervolume improvement < tol
    cfg.hv_stall_gen    = 40;        % Generations of HV stall before stop

    % --- Reference Point (for hypervolume) ---
    % Set to worst acceptable values for each objective (after sense conversion)
    cfg.ref_point       = [0.0, 2000, 0.0];  % [min CL/CD, max weight, min range]

    % --- Visualization ---
    cfg.plot_pareto     = true;      % Plot Pareto front at end
    cfg.plot_interval   = 25;        % Update Pareto plot every N generations

end

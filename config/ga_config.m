function cfg = ga_config()
% GA_CONFIG  Genetic Algorithm configuration parameters.
%   Returns a struct with all GA hyperparameters.
%   Modify this file to tune the optimizer — no hardcoded values elsewhere.

    cfg = struct();

    % --- Population ---
    cfg.pop_size        = 100;       % Population size (must be even for crossover)
    cfg.n_generations   = 200;       % Maximum number of generations

    % --- Selection ---
    cfg.tournament_size = 3;         % Tournament selection pool size

    % --- Crossover (Simulated Binary Crossover - SBX) ---
    cfg.crossover_prob  = 0.9;       % Probability of crossover
    cfg.crossover_eta   = 20;        % Distribution index (higher = children closer to parents)

    % --- Mutation (Polynomial Mutation) ---
    cfg.mutation_prob   = [];        % [] = auto (1/n_vars); set explicitly to override
    cfg.mutation_eta    = 20;        % Distribution index for mutation

    % --- Elitism (single-objective GA) ---
    cfg.elite_fraction  = 0.05;      % Fraction of population preserved as elite

    % --- Convergence ---
    cfg.stall_gen_limit = 30;        % Stop if no improvement for this many generations
    cfg.tol_fitness     = 1e-6;      % Minimum fitness change to count as improvement

    % --- Reproducibility ---
    cfg.rng_seed        = 42;        % Random number generator seed

    % --- Output ---
    cfg.verbose         = true;      % Print progress every generation
    cfg.save_interval   = 10;        % Save checkpoint every N generations

end

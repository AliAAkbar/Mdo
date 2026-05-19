function results = ga_single(obj_func, prob_cfg, ga_cfg)
% GA_SINGLE  Single-objective Genetic Algorithm with elitism.
%   results = ga_single(obj_func, prob_cfg, ga_cfg)
%
%   Inputs:
%     obj_func - Function handle: fitness = obj_func(individual)
%                Must return scalar fitness (minimize).
%     prob_cfg - Problem config struct (from problem_config.m)
%     ga_cfg   - GA config struct (from ga_config.m)
%
%   Output:
%     results  - Struct with fields:
%                .best_individual  - Best design vector found
%                .best_fitness     - Best fitness value
%                .history          - [n_gen x 3] (gen, best, mean)
%                .final_population - Final population matrix
%                .converged        - Logical (true if stall limit hit)
%                .generations_run  - Number of generations completed

    % --- Unpack config ---
    n_vars      = prob_cfg.n_vars;
    lb          = prob_cfg.lb;
    ub          = prob_cfg.ub;
    pop_size    = ga_cfg.pop_size;
    n_gen       = ga_cfg.n_generations;
    t_size      = ga_cfg.tournament_size;
    cx_prob     = ga_cfg.crossover_prob;
    cx_eta      = ga_cfg.crossover_eta;
    mut_eta     = ga_cfg.mutation_eta;
    elite_frac  = ga_cfg.elite_fraction;
    stall_limit = ga_cfg.stall_gen_limit;
    tol_fit     = ga_cfg.tol_fitness;
    verbose     = ga_cfg.verbose;

    % Mutation probability
    mut_prob = ga_cfg.mutation_prob;
    if isempty(mut_prob)
        mut_prob = 1 / n_vars;
    end

    % --- Set RNG for reproducibility ---
    rng(ga_cfg.rng_seed);

    % --- Initialize population ---
    pop = init_population(pop_size, n_vars, lb, ub);

    % --- Evaluate initial population ---
    fitness = zeros(pop_size, 1);
    for i = 1:pop_size
        fitness(i) = obj_func(pop(i, :));
    end

    % --- Tracking ---
    n_elite = max(1, round(elite_frac * pop_size));
    history = zeros(n_gen, 3);  % gen, best, mean
    [best_fit, best_idx] = min(fitness);
    best_ind = pop(best_idx, :);
    stall_counter = 0;
    converged = false;

    % --- Main loop ---
    for gen = 1:n_gen

        % Sort by fitness
        [sorted_fit, sort_idx] = sort(fitness);
        sorted_pop = pop(sort_idx, :);

        % Elitism: preserve top individuals
        elite_pop = sorted_pop(1:n_elite, :);
        elite_fit = sorted_fit(1:n_elite);

        % Generate offspring
        n_offspring = pop_size - n_elite;
        offspring = zeros(n_offspring, n_vars);
        idx_off = 1;

        while idx_off <= n_offspring
            % Selection
            parents_idx = selection(fitness, t_size, 2);
            p1 = pop(parents_idx(1), :);
            p2 = pop(parents_idx(2), :);

            % Crossover
            [c1, c2] = crossover(p1, p2, cx_eta, cx_prob, lb, ub);

            % Mutation
            c1 = mutation(c1, mut_eta, mut_prob, lb, ub);
            c2 = mutation(c2, mut_eta, mut_prob, lb, ub);

            % Add to offspring
            offspring(idx_off, :) = c1;
            idx_off = idx_off + 1;
            if idx_off <= n_offspring
                offspring(idx_off, :) = c2;
                idx_off = idx_off + 1;
            end
        end

        % Evaluate offspring
        offspring_fit = zeros(n_offspring, 1);
        for i = 1:n_offspring
            offspring_fit(i) = obj_func(offspring(i, :));
        end

        % Combine elite + offspring
        pop = [elite_pop; offspring];
        fitness = [elite_fit; offspring_fit];

        % Track convergence
        [gen_best, gen_best_idx] = min(fitness);
        gen_mean = mean(fitness);
        history(gen, :) = [gen, gen_best, gen_mean];

        % Update global best
        if gen_best < best_fit - tol_fit
            best_fit = gen_best;
            best_ind = pop(gen_best_idx, :);
            stall_counter = 0;
        else
            stall_counter = stall_counter + 1;
        end

        % Verbose output
        if verbose
            fprintf('Gen %4d | Best: %.6f | Mean: %.6f | Stall: %d\n', ...
                gen, gen_best, gen_mean, stall_counter);
        end

        % Convergence check
        if stall_counter >= stall_limit
            converged = true;
            history = history(1:gen, :);
            break;
        end
    end

    % --- Build results struct ---
    results = struct();
    results.best_individual  = best_ind;
    results.best_fitness     = best_fit;
    results.history          = history;
    results.final_population = pop;
    results.converged        = converged;
    results.generations_run  = size(history, 1);

end

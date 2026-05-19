function results = ga_multi(obj_func, prob_cfg, ga_cfg, moo_cfg)
% GA_MULTI  Multi-objective GA using NSGA-II algorithm.
%   results = ga_multi(obj_func, prob_cfg, ga_cfg, moo_cfg)
%
%   Inputs:
%     obj_func - Function handle: f = obj_func(individual)
%                Must return [1 x n_obj] vector of objective values.
%                All objectives are MINIMIZED internally.
%                Use obj_sense in moo_cfg to handle maximization.
%     prob_cfg - Problem config struct (from problem_config.m)
%     ga_cfg   - GA config struct (from ga_config.m)
%     moo_cfg  - Multi-objective config struct (from moo_config.m)
%
%   Output:
%     results  - Struct with fields:
%                .pareto_front     - [n_pareto x n_obj] Objective values on Pareto front
%                .pareto_set       - [n_pareto x n_vars] Corresponding design vectors
%                .history          - Struct array with per-generation data
%                .final_population - Final population matrix
%                .final_objectives - Final objective matrix
%                .generations_run  - Number of generations completed

    % --- Unpack config ---
    n_vars   = prob_cfg.n_vars;
    lb       = prob_cfg.lb;
    ub       = prob_cfg.ub;
    pop_size = ga_cfg.pop_size;
    n_gen    = moo_cfg.max_generations;
    t_size   = ga_cfg.tournament_size;
    cx_prob  = ga_cfg.crossover_prob;
    cx_eta   = ga_cfg.crossover_eta;
    mut_eta  = ga_cfg.mutation_eta;
    n_obj    = moo_cfg.n_objectives;
    obj_sense = moo_cfg.obj_sense;  % -1=max, +1=min

    mut_prob = ga_cfg.mutation_prob;
    if isempty(mut_prob)
        mut_prob = 1 / n_vars;
    end

    % --- Set RNG for reproducibility ---
    rng(ga_cfg.rng_seed);

    % --- Initialize population ---
    pop = init_population(pop_size, n_vars, lb, ub);

    % --- Evaluate initial population ---
    obj_vals = zeros(pop_size, n_obj);
    for i = 1:pop_size
        raw = obj_func(pop(i, :));
        obj_vals(i, :) = raw .* obj_sense;  % Convert to minimization
    end

    % --- History tracking ---
    history = struct('gen', {}, 'hypervolume', {}, 'n_pareto', {}, ...
                     'best_objectives', {});
    hv_prev = 0;
    hv_stall = 0;

    % --- Main NSGA-II loop ---
    for gen = 1:n_gen

        % ===== Generate offspring =====
        offspring = zeros(pop_size, n_vars);
        idx_off = 1;

        while idx_off <= pop_size
            % Tournament selection based on rank and crowding
            [ranks, crowd_dist] = non_dominated_sort(obj_vals);
            p1_idx = tournament_moo(ranks, crowd_dist, t_size);
            p2_idx = tournament_moo(ranks, crowd_dist, t_size);

            % Crossover
            [c1, c2] = crossover(pop(p1_idx,:), pop(p2_idx,:), cx_eta, cx_prob, lb, ub);

            % Mutation
            c1 = mutation(c1, mut_eta, mut_prob, lb, ub);
            c2 = mutation(c2, mut_eta, mut_prob, lb, ub);

            offspring(idx_off, :) = c1;
            idx_off = idx_off + 1;
            if idx_off <= pop_size
                offspring(idx_off, :) = c2;
                idx_off = idx_off + 1;
            end
        end

        % Evaluate offspring
        off_obj = zeros(pop_size, n_obj);
        for i = 1:pop_size
            raw = obj_func(offspring(i, :));
            off_obj(i, :) = raw .* obj_sense;
        end

        % ===== Merge parent + offspring =====
        combined_pop = [pop; offspring];
        combined_obj = [obj_vals; off_obj];

        % ===== Non-dominated sorting + crowding distance =====
        [ranks, crowd_dist] = non_dominated_sort(combined_obj);

        % ===== Select next generation (elitist) =====
        [pop, obj_vals] = select_next_gen(combined_pop, combined_obj, ...
                                           ranks, crowd_dist, pop_size);

        % ===== Extract current Pareto front =====
        [pf_ranks, ~] = non_dominated_sort(obj_vals);
        pareto_mask = (pf_ranks == 1);
        pf_obj = obj_vals(pareto_mask, :);
        pf_set = pop(pareto_mask, :);

        % ===== Hypervolume (for convergence tracking) =====
        hv_current = compute_hypervolume_2d(pf_obj, moo_cfg.ref_point);

        % Log history
        h = struct();
        h.gen = gen;
        h.hypervolume = hv_current;
        h.n_pareto = sum(pareto_mask);
        h.best_objectives = min(obj_vals, [], 1) ./ obj_sense;  % Convert back
        history(end+1) = h; %#ok<AGROW>

        % Verbose
        if ga_cfg.verbose
            fprintf('Gen %4d | Pareto size: %3d | HV: %.4f\n', ...
                gen, h.n_pareto, hv_current);
        end

        % Convergence check (hypervolume stall)
        if abs(hv_current - hv_prev) < moo_cfg.hypervolume_tol
            hv_stall = hv_stall + 1;
        else
            hv_stall = 0;
        end
        hv_prev = hv_current;

        if hv_stall >= moo_cfg.hv_stall_gen
            fprintf('Converged: hypervolume stalled for %d generations.\n', hv_stall);
            break;
        end
    end

    % --- Convert Pareto front back to original sense ---
    pareto_front_original = pf_obj ./ repmat(obj_sense, size(pf_obj,1), 1);

    % --- Build results struct ---
    results = struct();
    results.pareto_front     = pareto_front_original;
    results.pareto_set       = pf_set;
    results.history          = history;
    results.final_population = pop;
    results.final_objectives = obj_vals ./ repmat(obj_sense, pop_size, 1);
    results.generations_run  = gen;

end

%% ========================================================================
%  LOCAL FUNCTIONS
%  ========================================================================

function [ranks, crowd_dist] = non_dominated_sort(obj_vals)
% NON_DOMINATED_SORT  Fast non-dominated sorting (NSGA-II).
%   Returns rank (front index) and crowding distance for each individual.

    N = size(obj_vals, 1);
    n_obj = size(obj_vals, 2);
    ranks = zeros(N, 1);
    crowd_dist = zeros(N, 1);

    % Domination count and dominated set
    domination_count = zeros(N, 1);
    dominated_set = cell(N, 1);

    for i = 1:N
        for j = i+1:N
            if dominates(obj_vals(i,:), obj_vals(j,:))
                dominated_set{i}(end+1) = j;
                domination_count(j) = domination_count(j) + 1;
            elseif dominates(obj_vals(j,:), obj_vals(i,:))
                dominated_set{j}(end+1) = i;
                domination_count(i) = domination_count(i) + 1;
            end
        end
    end

    % Build fronts
    current_front = find(domination_count == 0)';
    rank_val = 1;

    while ~isempty(current_front)
        ranks(current_front) = rank_val;

        % Compute crowding distance for this front
        cd = crowding_distance(obj_vals(current_front, :));
        crowd_dist(current_front) = cd;

        next_front = [];
        for i = current_front
            for j = dominated_set{i}
                domination_count(j) = domination_count(j) - 1;
                if domination_count(j) == 0
                    next_front(end+1) = j; %#ok<AGROW>
                end
            end
        end

        current_front = unique(next_front);
        rank_val = rank_val + 1;
    end

    % Assign worst rank to any unranked (shouldn't happen)
    ranks(ranks == 0) = rank_val;
end

function result = dominates(a, b)
% DOMINATES  Returns true if a dominates b (all objectives minimized).
    result = all(a <= b) && any(a < b);
end

function cd = crowding_distance(obj_vals)
% CROWDING_DISTANCE  Compute crowding distance for a set of individuals.
    [N, n_obj] = size(obj_vals);
    cd = zeros(N, 1);

    if N <= 2
        cd(:) = Inf;
        return;
    end

    for m = 1:n_obj
        [~, sorted_idx] = sort(obj_vals(:, m));
        cd(sorted_idx(1)) = Inf;
        cd(sorted_idx(end)) = Inf;

        f_range = obj_vals(sorted_idx(end), m) - obj_vals(sorted_idx(1), m);
        if f_range < 1e-14
            continue;
        end

        for k = 2:N-1
            cd(sorted_idx(k)) = cd(sorted_idx(k)) + ...
                (obj_vals(sorted_idx(k+1), m) - obj_vals(sorted_idx(k-1), m)) / f_range;
        end
    end
end

function idx = tournament_moo(ranks, crowd_dist, t_size)
% TOURNAMENT_MOO  Tournament selection for NSGA-II.
%   Prefers lower rank; if tied, prefers higher crowding distance.
    N = numel(ranks);
    candidates = randi(N, t_size, 1);

    best = candidates(1);
    for k = 2:t_size
        c = candidates(k);
        if ranks(c) < ranks(best)
            best = c;
        elseif ranks(c) == ranks(best) && crowd_dist(c) > crowd_dist(best)
            best = c;
        end
    end
    idx = best;
end

function [new_pop, new_obj] = select_next_gen(pop, obj_vals, ranks, crowd_dist, pop_size)
% SELECT_NEXT_GEN  Elitist selection for next generation.
%   Fill population front-by-front; last front sorted by crowding distance.

    N = size(pop, 1);
    n_vars = size(pop, 2);
    n_obj = size(obj_vals, 2);
    new_pop = zeros(pop_size, n_vars);
    new_obj = zeros(pop_size, n_obj);

    max_rank = max(ranks);
    filled = 0;

    for r = 1:max_rank
        front_idx = find(ranks == r);
        n_front = numel(front_idx);

        if filled + n_front <= pop_size
            % Entire front fits
            new_pop(filled+1:filled+n_front, :) = pop(front_idx, :);
            new_obj(filled+1:filled+n_front, :) = obj_vals(front_idx, :);
            filled = filled + n_front;
        else
            % Partial front: sort by crowding distance (descending)
            [~, cd_sort] = sort(crowd_dist(front_idx), 'descend');
            n_needed = pop_size - filled;
            selected = front_idx(cd_sort(1:n_needed));
            new_pop(filled+1:pop_size, :) = pop(selected, :);
            new_obj(filled+1:pop_size, :) = obj_vals(selected, :);
            filled = pop_size;
            break;
        end

        if filled >= pop_size
            break;
        end
    end
end

function hv = compute_hypervolume_2d(pf_obj, ref_point)
% COMPUTE_HYPERVOLUME_2D  Hypervolume indicator for 2D/3D approximation.
%   Uses exact 2D computation or Monte Carlo for 3D.

    [n_points, n_obj] = size(pf_obj);

    if n_obj == 2
        % Exact 2D hypervolume
        % Sort by first objective
        [sorted_pf, ~] = sortrows(pf_obj, 1);
        hv = 0;
        for i = 1:n_points
            if i == 1
                width = sorted_pf(i, 1) - 0;  % from origin
            else
                width = sorted_pf(i, 1) - sorted_pf(i-1, 1);
            end
            % Height from this point to reference
            height = ref_point(2) - sorted_pf(i, 2);
            if height > 0 && width > 0
                hv = hv + width * height;
            end
        end
        % Add final rectangle
        if n_points > 0
            hv = hv + (ref_point(1) - sorted_pf(end, 1)) * ...
                      (ref_point(2) - sorted_pf(end, 2));
        end
    else
        % Approximate hypervolume using dominated hypervolume
        % Simple bounding approach for 3+ objectives
        hv = 0;
        for i = 1:n_points
            vol_i = prod(max(0, ref_point(1:n_obj) - pf_obj(i, :)));
            hv = hv + vol_i;
        end
        hv = hv / n_points;  % Approximate (not exact for >2D)
    end
end

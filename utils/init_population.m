function pop = init_population(pop_size, n_vars, lb, ub)
% INIT_POPULATION  Initialize population using Latin Hypercube Sampling.
%   pop = init_population(pop_size, n_vars, lb, ub)
%
%   Generates an initial population that provides good coverage of the
%   design space. Uses Latin Hypercube Sampling (LHS) for better space-
%   filling compared to random initialization.
%
%   Inputs:
%     pop_size - Number of individuals
%     n_vars   - Number of design variables
%     lb       - [1 x n_vars] Lower bounds
%     ub       - [1 x n_vars] Upper bounds
%
%   Output:
%     pop - [pop_size x n_vars] Initial population matrix

    % --- Latin Hypercube Sampling ---
    % Divide each dimension into pop_size equal intervals
    % Place one sample per interval, randomly within each interval
    pop = zeros(pop_size, n_vars);

    for j = 1:n_vars
        % Create intervals
        intervals = linspace(0, 1, pop_size + 1);

        % Random point within each interval
        lower_edges = intervals(1:end-1)';
        upper_edges = intervals(2:end)';
        samples = lower_edges + (upper_edges - lower_edges) .* rand(pop_size, 1);

        % Random permutation (decorrelate dimensions)
        perm = randperm(pop_size);
        pop(:, j) = samples(perm);
    end

    % --- Scale to actual bounds ---
    for j = 1:n_vars
        pop(:, j) = lb(j) + pop(:, j) * (ub(j) - lb(j));
    end

    % --- Verify bounds ---
    for j = 1:n_vars
        pop(:, j) = max(lb(j), min(ub(j), pop(:, j)));
    end

end
